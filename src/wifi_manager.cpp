#include "wifi_manager.h"
#include <WiFi.h>
#include <LittleFS.h>

static String _ssid;
static String _password;

static bool readConfig()
{
    // Format=true means auto-format on first mount (creates a fresh filesystem)
    if (!LittleFS.begin(true))
    {
        Serial.println("[WiFi] LittleFS mount failed");
        return false;
    }

    File f = LittleFS.open("/wifi.cfg", "r");
    if (!f)
    {
        Serial.println("[WiFi] /wifi.cfg not found — running without WiFi");
        LittleFS.end();
        return false;
    }

    while (f.available())
    {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#"))
            continue;

        int eq = line.indexOf('=');
        if (eq < 0)
            continue;

        String key = line.substring(0, eq);
        String val = line.substring(eq + 1);
        key.trim();
        val.trim();

        if (key == "ssid")
            _ssid = val;
        else if (key == "password")
            _password = val;
    }

    f.close();
    LittleFS.end();

    if (_ssid.length() == 0)
    {
        Serial.println("[WiFi] No SSID in wifi.cfg");
        return false;
    }

    return true;
}

bool wifiBegin()
{
    if (!readConfig())
        return false;

    Serial.printf("[WiFi] Connecting to %s\n", _ssid.c_str());
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
