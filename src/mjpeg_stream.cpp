#include "mjpeg_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <JPEGENC.h>

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
static volatile uint32_t s_frameSeq = 0;

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

static void drainHeaders(WiFiClient& client) {
    uint32_t deadline = millis() + 2000;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) return;
    }
}

// ---------------------------------------------------------------------------
// Request handlers
// ---------------------------------------------------------------------------

static void handleStream(WiFiClient& client, uint8_t* jpegBuf, const String& ip) {
    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "\r\n"
    );
    // Flush immediately — without this lwIP may buffer the headers and the
    // client sees no response until the first JPEG arrives (up to 115 ms later).
    client.flush();

    uint32_t framesSent = 0;
    uint32_t lastSeq    = s_frameSeq;

    while (client.connected()) {
        // Wait up to 600 ms for a new lepton frame.
        // 600 ms covers the longest FFC (flat-field correction) pause.
        uint32_t deadline = millis() + 600;
        while (s_frameSeq == lastSeq && millis() < deadline)
            vTaskDelay(5 / portTICK_PERIOD_MS);

        // Capture the current seq AFTER waking: if multiple frames arrived
        // while we were encoding, skip the intermediate ones.
        lastSeq = s_frameSeq;

        if (!client.connected()) break;

        size_t jpegLen = encodeFrame(jpegBuf);
        if (jpegLen == 0) { Serial.println("[MJPEG] encode failed"); continue; }

        client.print("--frame\r\n"
                     "Content-Type: image/jpeg\r\n");
        client.printf("Content-Length: %u\r\n\r\n", (unsigned)jpegLen);
        client.write(jpegBuf, (int)jpegLen);
        client.print("\r\n");
        client.flush();

        framesSent++;
        if (framesSent % 40 == 0)
            Serial.printf("[MJPEG] %s frames=%u jpegLen=%u heap=%u\n",
                          ip.c_str(), framesSent,
                          (unsigned)jpegLen, (unsigned)ESP.getFreeHeap());
    }
    Serial.printf("[MJPEG] %s stream closed after %u frames\n", ip.c_str(), framesSent);
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
    ClientArg* arg    = static_cast<ClientArg*>(param);
    WiFiClient client = arg->client;
    String     ip     = arg->ip;
    uint8_t*   jpegBuf = arg->jpegBuf;
    delete arg;

    client.setNoDelay(true);

    String reqLine = readClientLine(client);
    drainHeaders(client);
    Serial.printf("[MJPEG] %s %s\n", ip.c_str(), reqLine.c_str());

    if (reqLine.indexOf("snapshot") >= 0) {
        handleSnapshot(client, jpegBuf, ip);
    } else if (reqLine.startsWith("GET ")) {
        handleStream(client, jpegBuf, ip);
    } else {
        client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
        client.flush();
        Serial.printf("[MJPEG] %s 404\n", ip.c_str());
    }

    client.stop();
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

        String ip = client.remoteIP().toString();
        Serial.printf("[MJPEG] connect %s\n", ip.c_str());

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
