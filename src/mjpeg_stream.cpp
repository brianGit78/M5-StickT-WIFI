#include "mjpeg_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <JPEGENC.h>
#include <lwip/sockets.h>   // setsockopt SO_SNDTIMEO

extern uint16_t smallBuffer[FLIR_X * FLIR_Y];

#define MJPEG_PORT        8080
#define JPEG_QUALITY      JPEGE_Q_MED
#define STREAM_W          FLIR_X   // 160
#define STREAM_H          FLIR_Y   // 120
#define JPEG_BUF_BYTES    16000
#define CLIENT_TASK_STACK 10240    // JPEGENC struct ~3.3 KB on stack + WiFiClient + frame send overhead

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

// Palette-mapped RGB565 frame — allocated before WiFi in mjpegAllocBuffers().
// Written inside s_encodeMtx; never accessed outside that lock.
static uint16_t*         s_rgbBuf    = nullptr;

// Serialises pixel-copy + JPEG encode so only one client encodes at a time.
// Per-client JPEG *output* buffers are separate (each client malloc's its own).
static SemaphoreHandle_t s_encodeMtx = nullptr;

// Monotonically increasing counter, incremented by loop() (Core 1) each time
// getRawValues() completes. Client tasks (Core 0) poll this — no per-task
// notifications needed, so any number of clients can wait simultaneously.
static volatile uint32_t s_frameSeq      = 0;
static volatile int      s_activeClients = 0;  // diagnostic only; benign SMP race

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void mjpegAllocBuffers() {
    // Only the shared RGB buffer must be pre-allocated before WiFi fragments
    // the heap. Each client's 16 KB JPEG output buffer is allocated on connect.
    s_rgbBuf = (uint16_t*)malloc(STREAM_W * STREAM_H * sizeof(uint16_t));
    if (!s_rgbBuf)
        Serial.println("[MJPEG] ERROR: rgbBuf pre-alloc failed");
    else
        Serial.printf("[MJPEG] rgbBuf OK heap=%u\n", (unsigned)ESP.getFreeHeap());
}

void mjpegNotifyFrame() {
    s_frameSeq++;   // single writer (Core 1), volatile 32-bit write — safe without mutex
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

// Encode the current thermal frame into caller-supplied jpegBuf.
// s_encodeMtx ensures only one client fills s_rgbBuf at a time.
static size_t encodeFrame(uint8_t* jpegBuf) {
    if (!jpegBuf || !s_rgbBuf || !s_encodeMtx) return 0;

    xSemaphoreTake(s_encodeMtx, portMAX_DELAY);

    // Pixel copy: hold smallBufMutex only for the duration of the copy.
    xSemaphoreTake(smallBufMutex, portMAX_DELAY);
    const uint16_t* palette = currentPalette();
    for (int sy = 0; sy < STREAM_H; sy++)
        for (int sx = 0; sx < STREAM_W; sx++)
            s_rgbBuf[sy * STREAM_W + sx] =
                palette[thermalToIndex(smallBuffer[sy * FLIR_X + sx])];
    xSemaphoreGive(smallBufMutex);

    // JPEG encode from s_rgbBuf into caller's jpegBuf.
    JPEGENC    enc;
    JPEGENCODE jpe;
    size_t     len = 0;
    if (enc.open(jpegBuf, JPEG_BUF_BYTES) == JPEGE_SUCCESS &&
        enc.encodeBegin(&jpe, STREAM_W, STREAM_H,
                         JPEGE_PIXEL_RGB565, JPEGE_SUBSAMPLE_444,
                         JPEG_QUALITY) == JPEGE_SUCCESS) {
        enc.addFrame(&jpe, (uint8_t*)s_rgbBuf, STREAM_W * sizeof(uint16_t));
        len = (size_t)enc.close();
    } else {
        enc.close();
    }

    xSemaphoreGive(s_encodeMtx);
    return len;
}

// ---------------------------------------------------------------------------
// HTTP helpers
// ---------------------------------------------------------------------------

static String readClientLine(WiFiClient& client) {
    uint32_t deadline = millis() + 2000;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
        return client.readStringUntil('\n');
    }
    return "";
}

// Returns the User-Agent value (trimmed) and discards the rest of the headers.
static String readHeaders(WiFiClient& client) {
    String userAgent;
    uint32_t deadline = millis() + 2000;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
        String line = client.readStringUntil('\n');
        String lower = line; lower.toLowerCase();
        if (lower.startsWith("user-agent:"))
            userAgent = line.substring(11);
        if (line == "\r" || line.length() == 0) break;
    }
    userAgent.trim();
    return userAgent;
}

// Writes exactly len bytes; returns false and logs which write failed if short.
static bool writeAll(WiFiClient& client, const uint8_t* buf, size_t len, const char* what) {
    size_t written = client.write(buf, len);
    if (written != len) {
        Serial.printf("[MJPEG] write fail: %s wrote=%u expected=%u\n",
                      what, (unsigned)written, (unsigned)len);
        return false;
    }
    return true;
}

// Prints up to 200 bytes as escaped ASCII to Serial (\\r, \\n, \\xNN for non-printable).
static void logEscaped(const char* tag, const uint8_t* buf, size_t len) {
    Serial.print(tag);
    size_t n = (len < 200) ? len : 200;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = buf[i];
        if      (c == '\r') Serial.print("\\r");
        else if (c == '\n') Serial.print("\\n");
        else if (c >= 0x20 && c < 0x7f) Serial.print((char)c);
        else    { Serial.printf("\\x%02X", c); }
    }
    Serial.println();
}

// ---------------------------------------------------------------------------
// Request handlers
// ---------------------------------------------------------------------------

static void handleStream(WiFiClient& client, uint8_t* jpegBuf, const String& ip,
                         const String& reqLine, const String& userAgent) {
    static const char kHdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";

    Serial.printf("[MJPEG] %s \"%s\" ua=\"%s\" clients=%d\n",
                  ip.c_str(), reqLine.c_str(), userAgent.c_str(), s_activeClients);
    logEscaped("[MJPEG] >hdr: ", (const uint8_t*)kHdr, sizeof(kHdr) - 1);

    if (!writeAll(client, (const uint8_t*)kHdr, sizeof(kHdr) - 1, "resp-hdr")) return;
    client.flush();

    uint32_t framesSent = 0;
    uint32_t lastSeq    = s_frameSeq - 1; // one behind → first encode happens immediately
    uint32_t noFrameMs  = 0;              // cumulative time waiting with no new frame

    while (client.connected()) {
        uint32_t t0       = millis();
        uint32_t deadline = t0 + 600;
        while (s_frameSeq == lastSeq && millis() < deadline)
            vTaskDelay(5 / portTICK_PERIOD_MS);

        if (s_frameSeq == lastSeq) {
            // Lepton produced no frame this interval (FFC or camera stall).
            noFrameMs += millis() - t0;
            if (noFrameMs >= 3000) {
                Serial.printf("[MJPEG] %s no frame for 3 s — closing gracefully\n", ip.c_str());
                break;
            }
            continue;
        }
        noFrameMs = 0;
        lastSeq   = s_frameSeq;

        if (!client.connected()) {
            Serial.printf("[MJPEG] %s client closed before frame %u\n",
                          ip.c_str(), framesSent + 1);
            break;
        }

        size_t jpegLen = encodeFrame(jpegBuf);
        if (jpegLen == 0) { Serial.println("[MJPEG] encode failed"); continue; }

        char fhdr[80];
        int  fhdrLen = snprintf(fhdr, sizeof(fhdr),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            (unsigned)jpegLen);

        if (framesSent == 0) {
            logEscaped("[MJPEG] >frame1-hdr: ", (const uint8_t*)fhdr, (size_t)fhdrLen);
            Serial.printf("[MJPEG] jpeg SOI=%02X%02X EOI=%02X%02X len=%u\n",
                          jpegBuf[0], jpegBuf[1],
                          jpegBuf[jpegLen - 2], jpegBuf[jpegLen - 1],
                          (unsigned)jpegLen);
        }

        if (!writeAll(client, (const uint8_t*)fhdr,   (size_t)fhdrLen, "frame-hdr")    ||
            !writeAll(client, jpegBuf,                  jpegLen,         "jpeg-data")    ||
            !writeAll(client, (const uint8_t*)"\r\n",  2,               "trailing-crlf")) {
            break;
        }
        client.flush();

        framesSent++;
        if (framesSent == 1)
            Serial.printf("[MJPEG] %s first frame sent ok\n", ip.c_str());
        if (framesSent % 40 == 0)
            Serial.printf("[MJPEG] %s frames=%u jpegLen=%u heap=%u\n",
                          ip.c_str(), framesSent, (unsigned)jpegLen, (unsigned)ESP.getFreeHeap());
    }
    Serial.printf("[MJPEG] %s stream end frames=%u heap=%u (%s)\n",
                  ip.c_str(), framesSent, (unsigned)ESP.getFreeHeap(),
                  framesSent == 0 ? "0 frames — client closed immediately" : "done");
}

static void handleSnapshot(WiFiClient& client, uint8_t* jpegBuf, const String& ip) {
    // Wait up to 600 ms for a fresh lepton frame.
    uint32_t lastSeq = s_frameSeq;
    uint32_t deadline = millis() + 600;
    while (s_frameSeq == lastSeq && millis() < deadline)
        vTaskDelay(5 / portTICK_PERIOD_MS);

    size_t jpegLen = encodeFrame(jpegBuf);
    if (jpegLen == 0) {
        client.print("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n");
        client.flush();
        return;
    }

    client.print("HTTP/1.1 200 OK\r\n"
                 "Content-Type: image/jpeg\r\n"
                 "Cache-Control: no-cache\r\n"
                 "Connection: close\r\n");
    client.printf("Content-Length: %u\r\n\r\n", (unsigned)jpegLen);
    client.write(jpegBuf, (int)jpegLen);
    client.flush();
    Serial.printf("[MJPEG] %s snapshot %u bytes\n", ip.c_str(), (unsigned)jpegLen);
}

// ---------------------------------------------------------------------------
// Per-client task
// ---------------------------------------------------------------------------

struct ClientArg {
    WiFiClient client;
    String     ip;
    uint8_t*   jpegBuf;
    ClientArg(WiFiClient c, String addr, uint8_t* buf)
        : client(c), ip(addr), jpegBuf(buf) {}
};

static void clientTask(void* param) {
    ClientArg* arg     = static_cast<ClientArg*>(param);
    WiFiClient client  = arg->client;
    String     ip      = arg->ip;
    uint8_t*   jpegBuf = arg->jpegBuf;
    delete arg;

    s_activeClients++;
    Serial.printf("[MJPEG] %s task start heap=%u clients=%d\n",
                  ip.c_str(), (unsigned)ESP.getFreeHeap(), s_activeClients);

    client.setNoDelay(true);

    // Without a send-side deadline, lwIP's write/flush can block for TCP's full
    // retransmit timeout (~45 s) when the client sends a RST (Ctrl+C).  With
    // SO_SNDTIMEO=3s, writeAll() detects the short write within 3 s, breaks the
    // stream loop, and calls client.stop() — leaving the server ready for the
    // next connection immediately instead of stalling for almost a minute.
    {
        int _fd = client.fd();
        if (_fd >= 0) {
            struct timeval tv = {3, 0};
            setsockopt(_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        }
    }

    String reqLine   = readClientLine(client);
    String userAgent = readHeaders(client);

    if (reqLine.length() == 0) {
        // Connection was reset before we could read the request — log and clean up.
        Serial.printf("[MJPEG] %s empty request line — client closed on arrival\n", ip.c_str());
    } else if (reqLine.indexOf("snapshot") >= 0) {
        Serial.printf("[MJPEG] %s snapshot\n", ip.c_str());
        handleSnapshot(client, jpegBuf, ip);
    } else if (reqLine.startsWith("GET ")) {
        handleStream(client, jpegBuf, ip, reqLine, userAgent);
    } else {
        client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
        client.flush();
        Serial.printf("[MJPEG] %s 404 req=\"%s\"\n", ip.c_str(), reqLine.c_str());
    }

    client.stop();                      // always close before task exit
    s_activeClients--;
    Serial.printf("[MJPEG] %s task end heap=%u clients=%d\n",
                  ip.c_str(), (unsigned)ESP.getFreeHeap(), s_activeClients);
    free(jpegBuf);
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Server accept loop (runs on Core 0, spawns a clientTask per connection)
// ---------------------------------------------------------------------------

static void serverTask(void* /*param*/) {
    s_encodeMtx = xSemaphoreCreateMutex();
    if (!s_encodeMtx) {
        Serial.printf("[MJPEG] FATAL: mutex alloc failed, heap=%u — server not started\n",
                      (unsigned)ESP.getFreeHeap());
        vTaskDelete(NULL);
        return;
    }
    Serial.printf("[MJPEG] mutex OK heap=%u\n", (unsigned)ESP.getFreeHeap());

    WiFiServer server(MJPEG_PORT);
    server.begin();
    Serial.printf("[MJPEG] stream:   http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);
    Serial.printf("[MJPEG] snapshot: http://%s:%d/snapshot.jpg\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);

    for (;;) {
        WiFiClient client = server.available();
        if (!client) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

        // Guard against stale/already-reset sockets that available() occasionally
        // returns immediately after an abrupt Ctrl+C disconnect.
        if (!client.connected()) {
            Serial.println("[MJPEG] stale accept — skipping disconnected client");
            client.stop();
            continue;
        }

        String ip = client.remoteIP().toString();
        Serial.printf("[MJPEG] connect %s heap=%u clients=%d\n",
                      ip.c_str(), (unsigned)ESP.getFreeHeap(), s_activeClients);

        uint8_t* jpegBuf = (uint8_t*)malloc(JPEG_BUF_BYTES);
        if (!jpegBuf) {
            Serial.printf("[MJPEG] no heap for %s, dropping\n", ip.c_str());
            client.stop();
            continue;
        }

        ClientArg* arg = new ClientArg(client, ip, jpegBuf);
        if (!arg) {
            Serial.printf("[MJPEG] no heap for ClientArg, dropping %s\n", ip.c_str());
            client.stop();
            free(jpegBuf);
            continue;
        }
        if (xTaskCreatePinnedToCore(clientTask, "mjpeg_c",
                                     CLIENT_TASK_STACK, arg,
                                     1, nullptr, 0) != pdPASS) {
            Serial.printf("[MJPEG] task create failed for %s heap=%u\n",
                          ip.c_str(), (unsigned)ESP.getFreeHeap());
            delete arg;
            free(jpegBuf);
        }
        // Server loop returns here immediately — ready to accept the next client.
    }
}

void mjpegBegin() {
    xTaskCreatePinnedToCore(serverTask, "mjpeg_srv", 8192, nullptr, 1, nullptr, 0);
}
