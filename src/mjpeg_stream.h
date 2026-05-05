#pragma once
#include <Arduino.h>

// Call BEFORE wifiBegin() to grab heap before WiFi takes its ~100KB.
void rtspAllocBuffers();

// Starts the MJPEG-over-HTTP server on port 8080 (runs on Core 0).
// Call after WiFi is connected.
// Stream URL: http://<device-ip>:8080/
void rtspBegin();
