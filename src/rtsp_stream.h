#pragma once
#include <Arduino.h>

// Starts the RTSP server on port 554 (runs on Core 0 as a FreeRTOS task).
// Call after WiFi is connected.
// Stream URL: rtsp://<device-ip>/mjpeg/1
//         or: rtsp://m5stickt.local/mjpeg/1  (if mDNS is running)
// Call BEFORE wifiBegin() to grab heap before WiFi takes its ~100KB.
void rtspAllocBuffers();

// Starts the RTSP server. Call after WiFi is connected.
void rtspBegin();
