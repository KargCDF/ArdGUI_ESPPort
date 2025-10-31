#include "WebBridge.h"

#include <WiFi.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

#define AP_SSID  "ArdGUI_ESP32"
#define MDNS_HOSTNAME "am"  // This makes it accessible as "am.local"

/* ---------- internal: server + websocket objects ---------- */
static AsyncWebServer  server(80);
static AsyncWebSocket  ws("/ws");

/*  Your existing frame parser lives elsewhere (main.cpp)  */
extern void processIncomingFrame(uint8_t *buf, size_t len);

/* ---------------- WebSocket event bridge ------------------ */
static void onWsEvent(AsyncWebSocket* /*srv*/, AsyncWebSocketClient* /*c*/,
                      AwsEventType type, void *arg,
                      uint8_t *data, size_t len)
{
    if (type == WS_EVT_DATA) {
        auto *info = reinterpret_cast<AwsFrameInfo*>(arg);
        if (info->opcode == WS_BINARY) {
            processIncomingFrame(data, len);          // reuse MCU parser
        }
    }
}

/* --------------------- public API ------------------------- */
void WebBridge_begin()
{
    /* 1️Mount SPIFFS so we can serve the UI later */
    if (!SPIFFS.begin(true))
        Serial.println("[SPIFFS] mount failed");

    /* 2️Start a simple open Access-Point (change to STA if preferred) */
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    Serial.printf("[WiFi] AP SSID '%s'  IP %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    /* 3️Setup mDNS */
    if (MDNS.begin(MDNS_HOSTNAME)) {
        Serial.printf("[mDNS] Started - accessible at http://%s.local\n", MDNS_HOSTNAME);
        
        // Add service advertisement
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 80);  // WebSocket service
        
        // Add some service details
        MDNS.addServiceTxt("http", "tcp", "board", "ESP32");
        MDNS.addServiceTxt("http", "tcp", "path", "/");
    } else {
        Serial.println("[mDNS] Failed to start");
    }

    /* 4️Web server & WebSocket */
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    /* ---- serve everything under SPIFFS root ------------------- */
    server.serveStatic("/", SPIFFS, "/")
          .setDefaultFile("index.html");           // auto-redirect "/"

    /* nice-to-have 404 handler (optional) */
    server.onNotFound([](AsyncWebServerRequest *r){
        r->send(404, "text/plain", "404 – Not Found");
    });

    server.begin();
    Serial.println("[WEB] HTTP + WS started");
}

void WebBridge_loop()
{
    static unsigned long lastCleanup = 0;
    constexpr unsigned long CLEANUP_INTERVAL_MS = 100;  // Cleanup every 100ms

    unsigned long now = millis();
    if (now - lastCleanup >= CLEANUP_INTERVAL_MS) {
        ws.cleanupClients();
        lastCleanup = now;
    }
    // ESP32 mDNS runs automatically - no update() needed
}

void WebBridge_sendFrame(const uint8_t *b, size_t n)
{
    ws.binaryAll(const_cast<uint8_t *>(b), n);   // cast to match overload
}
