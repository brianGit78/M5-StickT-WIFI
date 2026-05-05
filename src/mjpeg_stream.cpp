#include "mjpeg_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <JPEGENC.h>

extern uint16_t smallBuffer[FLIR_X * FLIR_Y];

#define MJPEG_PORT     8080
#define JPEG_QUALITY   JPEGE_Q_MED
#define STREAM_W       FLIR_X   // 160
#define STREAM_H       FLIR_Y   // 120 — full sensor; 4:4:4 uses 8×8 MCUs so any multiple of 8 is valid
#define JPEG_BUF_BYTES 16000    // 4:4:4 encodes all three channels fully; headroom vs 4:2:0

static uint8_t*     s_jpegBuf   = nullptr;
static uint16_t*    s_rgbBuf    = nullptr;
static TaskHandle_t s_mjpegTask = nullptr;

void mjpegAllocBuffers() {
    s_jpegBuf = (uint8_t*)malloc(JPEG_BUF_BYTES);
    s_rgbBuf  = (uint16_t*)malloc(STREAM_W * STREAM_H * sizeof(uint16_t));
    if (!s_jpegBuf || !s_rgbBuf)
        Serial.println("[MJPEG] ERROR: buffer pre-alloc failed");
    else
        Serial.printf("[MJPEG] Buffers OK, heap free: %u\n", (unsigned)ESP.getFreeHeap());
}

static size_t encodeFrame() {
    if (!s_jpegBuf || !s_rgbBuf) return 0;

    // Hold the mutex only for the pixel copy so we get a coherent snapshot of
    // smallBuffer. Without this, the Lepton write (Core 1) races the MJPEG read
    // (Core 0) mid-frame, producing horizontal black bands in the stream.
    xSemaphoreTake(smallBufMutex, portMAX_DELAY);
    const uint16_t* palette = currentPalette();
    for (int sy = 0; sy < STREAM_H; sy++)
        for (int sx = 0; sx < STREAM_W; sx++)
            s_rgbBuf[sy * STREAM_W + sx] =
                palette[thermalToIndex(smallBuffer[sy * FLIR_X + sx])];
    xSemaphoreGive(smallBufMutex);

    JPEGENC    enc;
    JPEGENCODE jpe;
    if (enc.open(s_jpegBuf, JPEG_BUF_BYTES) != JPEGE_SUCCESS) return 0;
    if (enc.encodeBegin(&jpe, STREAM_W, STREAM_H,
                         JPEGE_PIXEL_RGB565,
                         JPEGE_SUBSAMPLE_444,
                         JPEG_QUALITY) != JPEGE_SUCCESS) {
        enc.close();
        return 0;
    }
    enc.addFrame(&jpe, (uint8_t*)s_rgbBuf, STREAM_W * sizeof(uint16_t));
    return (size_t)enc.close();
}

// Read one line from the client with a deadline, returning "" on timeout/disconnect.
static String readClientLine(WiFiClient& client) {
    uint32_t deadline = millis() + 2000;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
        return client.readStringUntil('\n');
    }
    return "";
}

// Drain all remaining HTTP request headers until the blank line.
static void drainHeaders(WiFiClient& client) {
    uint32_t deadline = millis() + 2000;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) return;
    }
}

// Serve a continuous MJPEG multipart stream.
static void handleStream(WiFiClient& client) {
    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "\r\n"
    );
    // Flush before blocking — without this, lwIP may hold the header bytes in
    // its send buffer until more data arrives, causing curl/Blue Iris to see no
    // response at all while VLC (which sends a keep-alive ping) gets unblocked.
    client.flush();

    uint32_t framesSent = 0;
    while (client.connected()) {
        // Block until loop() signals that fresh lepton data has landed.
        // Timeout of 500 ms handles FFC pauses without hanging indefinitely.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        if (!client.connected()) break;

        size_t jpegLen = encodeFrame();
        if (jpegLen == 0) { Serial.println("[MJPEG] encode failed"); continue; }

        client.print("--frame\r\n"
                     "Content-Type: image/jpeg\r\n");
        client.printf("Content-Length: %u\r\n\r\n", (unsigned)jpegLen);
        client.write(s_jpegBuf, (int)jpegLen);
        client.print("\r\n");
        client.flush();

        framesSent++;
        if (framesSent % 40 == 0)
            Serial.printf("[MJPEG] frames=%u jpegLen=%u heap=%u\n",
                          framesSent, (unsigned)jpegLen, (unsigned)ESP.getFreeHeap());
    }
}

// Serve a single JPEG snapshot and close.
static void handleSnapshot(WiFiClient& client) {
    // Wait up to 500 ms for a fresh lepton frame before encoding.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

    size_t jpegLen = encodeFrame();
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
    client.write(s_jpegBuf, (int)jpegLen);
    client.flush();

    Serial.printf("[MJPEG] snapshot jpegLen=%u\n", (unsigned)jpegLen);
}

static void mjpegTask(void* /*param*/) {
    WiFiServer server(MJPEG_PORT);
    server.begin();
    Serial.printf("[MJPEG] stream:   http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);
    Serial.printf("[MJPEG] snapshot: http://%s:%d/snapshot.jpg\n",
                  WiFi.localIP().toString().c_str(), MJPEG_PORT);

    for (;;) {
        WiFiClient client = server.available();
        if (!client) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

        Serial.printf("[MJPEG] client %s\n", client.remoteIP().toString().c_str());
        client.setNoDelay(true);

        // Read the request line (e.g. "GET /stream HTTP/1.1\r"), then drain headers.
        String reqLine = readClientLine(client);
        drainHeaders(client);

        Serial.printf("[MJPEG] req: %s\n", reqLine.c_str());

        if (reqLine.indexOf("snapshot") >= 0) {
            handleSnapshot(client);
        } else if (reqLine.startsWith("GET ")) {
            // Accepts GET /, GET /stream, and any other GET path → MJPEG stream
            handleStream(client);
        } else {
            client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
            client.flush();
        }

        client.stop();
        Serial.println("[MJPEG] client disconnected");
    }
}

void mjpegBegin() {
    xTaskCreatePinnedToCore(
        mjpegTask,
        "mjpeg",
        8192,   // actual usage ~5 KB (JPEGENC struct 3.3 KB + WiFiClient + overhead)
        nullptr,
        1,
        &s_mjpegTask,
        0
    );
}

void mjpegNotifyFrame() {
    if (s_mjpegTask)
        xTaskNotifyGive(s_mjpegTask);
}
