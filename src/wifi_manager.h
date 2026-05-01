#pragma once
#include <Arduino.h>

// Reads /wifi.cfg from LittleFS and begins a non-blocking WiFi connection.
// Returns true if a config file was found with a non-empty SSID.
// Safe to call even if no wifi.cfg exists — silently does nothing.
bool wifiBegin();

bool wifiIsConnected();

// Returns the IP address string, or an empty string if not connected.
String wifiGetIP();

// Returns the mDNS hostname (e.g. "m5stickt").
String wifiGetHostname();

// Registers mDNS hostname and advertises RTSP service on port 554.
// Call once after wifiIsConnected() returns true.
void wifiSetupMDNS();
