#pragma once
#include <Arduino.h>

// Pre-allocates the shared RGB frame buffer (~38 KB) before WiFi fragments the heap.
// Per-client JPEG output buffers (16 KB each) are allocated at connection time.
// Call BEFORE wifiBegin().
void mjpegAllocBuffers();

// Starts the MJPEG HTTP server on port 8080 (Core 0 accept loop + per-client tasks).
// Call after WiFi is connected.
//   Stream:   http://<ip>:8080/stream
//   Snapshot: http://<ip>:8080/snapshot.jpg
void mjpegBegin();

// Call from loop() immediately after lepton.getRawValues() completes.
// Increments the frame-sequence counter that all client tasks poll.
void mjpegNotifyFrame();
