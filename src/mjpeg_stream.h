#pragma once
#include <Arduino.h>

// Call BEFORE wifiBegin() to grab heap before WiFi takes its ~100KB.
void mjpegAllocBuffers();

// Starts the MJPEG-over-HTTP server on port 8080 (runs on Core 0).
// Call after WiFi is connected.
// Stream URL: http://<device-ip>:8080/
void mjpegBegin();

// Call from loop() after lepton.getRawValues() to wake the MJPEG task immediately.
// Safe to call before mjpegBegin() (no-op if task hasn't started yet).
void mjpegNotifyFrame();
