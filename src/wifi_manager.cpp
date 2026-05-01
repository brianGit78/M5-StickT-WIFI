#include "wifi_manager.h"
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>

static String _ssid;
static String _password;
static const char* HOSTNAME = "m5stickt";

// WiFi credentials are stored in NVS via the Preferences library.
// On first boot they are seeded from the values below.
// To change credentials: update these constants and reflash, or
// write new values to NVS key "wifi" (ssid / password) externally.
static const char* DEFAULT_SSID     = "BriFi";
static const char* DEFAULT_PASSWORD = "2032885908";

static bool readConfig()
{
    Preferences prefs;
    prefs.begin("wifi", true); // read-only
    _ssid     = prefs.getString("ssid",     "");
    _password = prefs.getString("password", "");
    prefs.end();

    if (_ssid.length() == 0) {
        // Seed NVS with defaults on first run
        _ssid     = DEFAULT_SSID;
        _password = DEFAULT_PASSWORD;
        Preferences rw;
        rw.begin("wifi", false);
        rw.putString("ssid",     _ssid);
        rw.putString("password", _password);
        rw.end();
        Serial.printf("[WiFi] NVS: seeded defaults (ssid=%s)\n", _ssid.c_str());
    } else {
        Serial.printf("[WiFi] NVS: loaded ssid=%s\n", _ssid.c_str());
    }
    return true;
}

bool wifiBegin()
{
    if (!readConfig())
        return false;

    Serial.printf("[WiFi] Connecting to %s\n", _ssid.c_str());
    WiFi.setHostname(HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_ssid.c_str(), _password.c_str());
    // Non-blocking — caller can poll wifiIsConnected()
    return true;
}

bool wifiIsConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

String wifiGetIP()
{
    if (!wifiIsConnected())
        return "";
    return WiFi.localIP().toString();
}

String wifiGetHostname()
{
    return String(HOSTNAME);
}

void wifiSetupMDNS()
{
    if (!MDNS.begin(HOSTNAME)) {
        Serial.println("[WiFi] mDNS start failed");
        return;
    }
    MDNS.addService("rtsp", "tcp", 554);
    Serial.printf("[WiFi] mDNS: %s.local  RTSP on port 554\n", HOSTNAME);
}
