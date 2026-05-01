#include "rtsp_stream.h"
#include "thermal_state.h"
#include "lepton.h"

#include <WiFi.h>
#include <CRtspSession.h>
#include <CStreamer.h>
#include <JPEGENC.h>

extern uint16_t smallBuffer[FLIR_X * FLIR_Y];
extern uint16_t raw_max, raw_min, fpa_temp;

#define RTSP_PORT        554
#define STREAM_FPS       8
#define STREAM_PERIOD_MS (1000 / STREAM_FPS)
#define JPEG_QUALITY     60
// Half-resolution, height MUST be a multiple of 8 (RFC 2435 width/height fields are in 8px units).
// 80x56 (7*8) — drops last 8 of 120 source rows (barely visible crop).
#define STREAM_W         (FLIR_X / 2)   // 80
#define STREAM_H         56              // 7*8; FLIR_Y/2=60 is not a multiple of 8
#define JPEG_BUF_BYTES   8000

// Allocated by rtspAllocBuffers() after WiFi connects.
static uint8_t*  s_jpegBuf = nullptr;
static uint16_t* s_rgbBuf  = nullptr;

void rtspAllocBuffers() {
    s_jpegBuf = (uint8_t*)malloc(JPEG_BUF_BYTES);
    s_rgbBuf  = (uint16_t*)malloc(STREAM_W * STREAM_H * sizeof(uint16_t));
    if (!s_jpegBuf || !s_rgbBuf)
        Serial.println("[RTSP] ERROR: buffer pre-alloc failed");
    else
        Serial.printf("[RTSP] Buffers OK, heap free: %u\n", ESP.getFreeHeap());
}

class LeptonStreamer : public CStreamer {
    uint32_t _lastFrameMs;

    size_t encodeCurrentFrame() {
        if (!s_jpegBuf || !s_rgbBuf) return 0;

        const uint16_t* palette = currentPalette();
        // Subsample 160x120 → 80x60 by taking every other pixel.
        for (int sy = 0; sy < STREAM_H; sy++)
            for (int sx = 0; sx < STREAM_W; sx++)
                s_rgbBuf[sy * STREAM_W + sx] =
                    palette[thermalToIndex(smallBuffer[(sy * 2) * FLIR_X + (sx * 2)])];

        JPEGENC    enc;
        JPEGENCODE jpe;

        if (enc.open(s_jpegBuf, JPEG_BUF_BYTES) != JPEGE_SUCCESS)
            return 0;
        // SUBSAMPLE_420 matches RTP-JPEG type 1 (4:2:0) in CStreamer.cpp.
        if (enc.encodeBegin(&jpe, STREAM_W, STREAM_H,
                             JPEGE_PIXEL_RGB565,
                             JPEGE_SUBSAMPLE_420,
                             JPEG_QUALITY) != JPEGE_SUCCESS) {
            enc.close();
            return 0;
        }
        enc.addFrame(&jpe, (uint8_t*)s_rgbBuf, STREAM_W * sizeof(uint16_t));
        return (size_t)enc.close();
    }

public:
    LeptonStreamer() : CStreamer(STREAM_W, STREAM_H), _lastFrameMs(0) {}

    void streamImage(uint32_t curmsec) override {
        if (curmsec - _lastFrameMs < STREAM_PERIOD_MS)
            return;
        _lastFrameMs = curmsec;
        size_t jpegLen = encodeCurrentFrame();
        static uint32_t framesSent = 0;
        static uint32_t lastPrintMs = 0;
        if (jpegLen > 0) {
            framesSent++;
            uint32_t now = millis();
            if (now - lastPrintMs > 5000) {
                lastPrintMs = now;
                Serial.printf("[RTSP] frames=%u jpegLen=%u\n", framesSent, (unsigned)jpegLen);
            }
            streamFrame((unsigned const char*)s_jpegBuf, (uint32_t)jpegLen, curmsec);
        } else {
            Serial.println("[RTSP] JPEG encode failed");
        }
    }
};

static void rtspTask(void* /*param*/) {
    LeptonStreamer streamer;
    streamer.setURI(WiFi.localIP().toString() + ":554");

    WiFiServer server(RTSP_PORT);
    server.begin();
    Serial.printf("[RTSP] Listening on port %d\n", RTSP_PORT);

    for (;;) {
        WiFiClient* client = new WiFiClient(server.available());
        if (*client) {
            Serial.printf("[RTSP] Client connected from %s\n",
                          client->remoteIP().toString().c_str());
            streamer.addSession(client);

            while (streamer.anySessions()) {
                streamer.handleRequests(0);
                if (streamer.anySessions())
                    streamer.streamImage(millis());
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            Serial.println("[RTSP] Client disconnected");
        }
        delete client;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void rtspBegin() {
    xTaskCreatePinnedToCore(
        rtspTask,
        "rtsp",
        32768,
        nullptr,
        1,
        nullptr,
        0
    );
}
