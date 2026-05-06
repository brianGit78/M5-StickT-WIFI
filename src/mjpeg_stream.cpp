#include "mjpeg_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <JPEGENC.h>
#include <lwip/sockets.h>   // setsockopt SO_SNDTIMEO

extern uint16_t smallBuffer[FLIR_X * FLIR_Y];

#define MJPEG_PORT        8080
#define JPEG_QUALITY      JPEGE_Q_MED
#define STREAM_W          320      // 2x nearest-neighbour upscale from FLIR 160
#define STREAM_H          240      // 2x nearest-neighbour upscale from FLIR 120
#define JPEG_BUF_BYTES    32000    // 320×240 @ medium quality; 16 KB was sized for 160×120
#define CLIENT_TASK_STACK 16384    // JPEGENC+JPEGENCODE on stack ~3.3KB + WiFiClient + String + headroom

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

// Exclusive stream slot: at most one /stream client at a time.
// Checked and set atomically under s_streamMux; cleared immediately when the
// stream client disconnects or any write fails.
static portMUX_TYPE    s_streamMux    = portMUX_INITIALIZER_UNLOCKED;
static volatile bool   s_streamActive = false;

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

// Returns true only if 'h' is non-null AND within ESP32 internal DRAM.
// Catches the class of crash where a semaphore handle is corrupted with a
// non-null value outside valid memory (e.g. 0x1ded1ded), which xSemaphoreTake
// would load-fault on at offset +8 (Queue_t::pcWriteTo).
static bool validHandle(const void* h) {
    uintptr_t a = (uintptr_t)h;
    // ESP32 internal SRAM: ~0x3FFA0000–0x3FFFFFFF (320 KB usable)
    return (a >= 0x3FFA0000UL && a < 0x40000000UL);
}

// Encode the current thermal frame into caller-supplied jpegBuf.
// s_encodeMtx ensures only one client fills s_rgbBuf at a time.
static size_t encodeFrame(uint8_t* jpegBuf) {
    if (!jpegBuf || !s_rgbBuf) return 0;
    if (!validHandle(s_encodeMtx) || !validHandle(smallBufMutex)) {
        Serial.printf("[MJPEG] encodeFrame: corrupt handle"
                      " s_encodeMtx=%p smallBufMutex=%p — skipping\n",
                      s_encodeMtx, smallBufMutex);
        return 0;
    }

    // 5-second timeouts prevent a clientTask from blocking indefinitely when
    // the lepton mutex is held across a slow getRawValues() call, which would
    // hold s_encodeMtx and starve all subsequent clientTask encode attempts.
    if (xSemaphoreTake(s_encodeMtx, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[MJPEG] encodeFrame: s_encodeMtx timeout");
        return 0;
    }
    if (xSemaphoreTake(smallBufMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[MJPEG] encodeFrame: smallBufMutex timeout");
        xSemaphoreGive(s_encodeMtx);
        return 0;
    }

    // Pixel copy with 2x nearest-neighbour upscale (160×120 → 320×240).
    // Each sensor pixel maps to a 2×2 block in the output; FLIR_X/Y are fixed
    // at 160/120 so the divisors are compile-time constants (no division needed).
    const uint16_t* palette = currentPalette();
    for (int sy = 0; sy < STREAM_H; sy++)
        for (int sx = 0; sx < STREAM_W; sx++)
            s_rgbBuf[sy * STREAM_W + sx] =
                palette[thermalToIndex(smallBuffer[(sy >> 1) * FLIR_X + (sx >> 1)])];
    xSemaphoreGive(smallBufMutex);

    // JPEG encode from s_rgbBuf into caller's jpegBuf.
    JPEGENC    enc;
    JPEGENCODE jpe;
    size_t     len = 0;
    int openRes   = enc.open(jpegBuf, JPEG_BUF_BYTES);
    int beginRes  = (openRes == JPEGE_SUCCESS)
                    ? enc.encodeBegin(&jpe, STREAM_W, STREAM_H,
                                      JPEGE_PIXEL_RGB565, JPEGE_SUBSAMPLE_444,
                                      JPEG_QUALITY)
                    : -1;
    if (openRes == JPEGE_SUCCESS && beginRes == JPEGE_SUCCESS) {
        enc.addFrame(&jpe, (uint8_t*)s_rgbBuf, STREAM_W * sizeof(uint16_t));
        int n = enc.close();
        if (n > 0) len = (size_t)n;
        else Serial.printf("[MJPEG] encodeFrame: JPEG close returned %d\n", n);
    } else {
        Serial.printf("[MJPEG] encodeFrame: JPEG init failed open=%d begin=%d\n",
                      openRes, beginRes);
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

// Extract the path component from "GET /path HTTP/1.1" → "/path".
static String extractPath(const String& reqLine) {
    int sp1 = reqLine.indexOf(' ');
    if (sp1 < 0) return "";
    int sp2 = reqLine.indexOf(' ', sp1 + 1);
    String path = (sp2 > sp1) ? reqLine.substring(sp1 + 1, sp2)
                               : reqLine.substring(sp1 + 1);
    // Strip query string so "/?foo=1" and "/" both match "/".
    int q = path.indexOf('?');
    if (q >= 0) path = path.substring(0, q);
    return path;
}

// GET / — short status/index page; lets a browser confirm the device is alive
// without accidentally opening a long-lived stream.
static void handleStatus(WiFiClient& client, const String& ip) {
    char body[256];
    int bodyLen = snprintf(body, sizeof(body),
        "Thermal camera online\r\n"
        "Stream:   /stream\r\n"
        "Snapshot: /snapshot.jpg\r\n"
        "Heap:     %u bytes\r\n"
        "Clients:  %d\r\n",
        (unsigned)ESP.getFreeHeap(), s_activeClients);
    char hdr[128];
    int hdrLen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n", bodyLen);
    client.write((const uint8_t*)hdr,  (size_t)hdrLen);
    client.write((const uint8_t*)body, (size_t)bodyLen);
    client.flush();
    Serial.printf("[MJPEG] %s 200 status\n", ip.c_str());
}

static void handleStream(WiFiClient& client, uint8_t* jpegBuf, const String& ip,
                         const String& reqLine, const String& userAgent) {
    // Atomically claim the single stream slot.
    bool claimed = false;
    portENTER_CRITICAL(&s_streamMux);
    if (!s_streamActive) { s_streamActive = true; claimed = true; }
    portEXIT_CRITICAL(&s_streamMux);

    if (!claimed) {
        static const char k503[] =
            "HTTP/1.1 503 Service Unavailable\r\n"
            "Content-Type: text/plain\r\n"
            "Connection: close\r\n"
            "Retry-After: 2\r\n"
            "\r\n"
            "MJPEG stream already in use";
        client.write((const uint8_t*)k503, sizeof(k503) - 1);
        client.flush();
        Serial.printf("[MJPEG] %s 503 stream busy clients=%d\n", ip.c_str(), s_activeClients);
        return;
    }

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

    if (!writeAll(client, (const uint8_t*)kHdr, sizeof(kHdr) - 1, "resp-hdr")) {
        portENTER_CRITICAL(&s_streamMux);
        s_streamActive = false;
        portEXIT_CRITICAL(&s_streamMux);
        return;
    }
    client.flush();

    uint32_t framesSent   = 0;
    uint32_t lastSeq      = s_frameSeq - 1; // one behind → first encode happens immediately
    uint32_t noFrameMs    = 0;              // cumulative ms with no new lepton frame
    uint32_t encodeFails  = 0;             // consecutive JPEG encode failures

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
        if (jpegLen == 0) {
            if (++encodeFails >= 5) {
                Serial.printf("[MJPEG] %s %u consecutive encode failures — closing\n",
                              ip.c_str(), encodeFails);
                break;
            }
            continue;
        }
        encodeFails = 0;

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

    // Release stream slot immediately so the next client doesn't get a 503.
    portENTER_CRITICAL(&s_streamMux);
    s_streamActive = false;
    portEXIT_CRITICAL(&s_streamMux);

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
    ClientArg* arg   = static_cast<ClientArg*>(param);
    uint8_t* jpegBuf = arg->jpegBuf;

    s_activeClients++;

    // WiFiClient and String MUST be destroyed before vTaskDelete(NULL):
    // FreeRTOS does not call C++ destructors during task teardown, so any
    // WiFiClient left alive on the stack leaks its lwIP PCB reference.
    // After a few Ctrl+C reconnects the lwIP connection table fills up and
    // every new connection gets an immediate RST.  Wrapping them in an inner
    // scope guarantees their destructors run at the closing brace, before
    // the task stack is freed.
    {
        WiFiClient client = arg->client;
        String     ip     = arg->ip;
        delete arg;

        Serial.printf("[MJPEG] %s task start heap=%u clients=%d\n",
                      ip.c_str(), (unsigned)ESP.getFreeHeap(), s_activeClients);

        client.setNoDelay(true);

        // Without a send-side deadline, lwIP's write/flush can block for TCP's
        // full retransmit timeout (~45 s) when the client sends RST (Ctrl+C).
        // SO_SNDTIMEO=3s lets writeAll() detect a short-write within 3 s.
        {
            int _fd = client.fd();
            if (_fd >= 0) {
                struct timeval tv = {3, 0};
                setsockopt(_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
            }
        }

        String reqLine   = readClientLine(client);
        String userAgent = readHeaders(client);
        String path      = extractPath(reqLine);

        Serial.printf("[MJPEG] %s req=\"%s\" path=\"%s\" ua=\"%s\" stream=%s clients=%d\n",
                      ip.c_str(), reqLine.c_str(), path.c_str(), userAgent.c_str(),
                      s_streamActive ? "active" : "idle", s_activeClients);

        if (reqLine.length() == 0) {
            Serial.printf("[MJPEG] %s → empty request — client closed on arrival\n", ip.c_str());
        } else if (path == "/") {
            handleStatus(client, ip);
        } else if (path == "/snapshot.jpg") {
            handleSnapshot(client, jpegBuf, ip);
        } else if (path == "/stream") {
            handleStream(client, jpegBuf, ip, reqLine, userAgent);
        } else {
            static const char k404[] =
                "HTTP/1.1 404 Not Found\r\n"
                "Content-Type: text/plain\r\n"
                "Connection: close\r\n"
                "\r\n"
                "Not Found";
            client.write((const uint8_t*)k404, sizeof(k404) - 1);
            client.flush();
            Serial.printf("[MJPEG] %s → 404 path=\"%s\"\n", ip.c_str(), path.c_str());
        }

        client.stop();
        s_activeClients--;
        Serial.printf("[MJPEG] %s task end heap=%u clients=%d stackHWM=%u\n",
                      ip.c_str(), (unsigned)ESP.getFreeHeap(), s_activeClients,
                      (unsigned)uxTaskGetStackHighWaterMark(NULL));
    } // WiFiClient::~WiFiClient() and String::~String() run here

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
    Serial.printf("[MJPEG] server start heap=%u stackHWM=%u\n",
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)uxTaskGetStackHighWaterMark(NULL));

    WiFiServer server(MJPEG_PORT);
    server.begin();
    Serial.printf("[MJPEG] stream:   http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);
    Serial.printf("[MJPEG] snapshot: http://%s:%d/snapshot.jpg\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);

    static uint32_t lastSrvLog = 0;
    for (;;) {
        uint32_t _now = millis();
        if (_now - lastSrvLog > 30000) {
            lastSrvLog = _now;
            Serial.printf("[MJPEG] srv heartbeat heap=%u stackHWM=%u clients=%d\n",
                          (unsigned)ESP.getFreeHeap(),
                          (unsigned)uxTaskGetStackHighWaterMark(NULL),
                          s_activeClients);
        }
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
