#pragma once
#include <cstddef>
#include <cstdint>

/*  Initialise Wi-Fi + WebSocket layer (call once in setup)  */
void WebBridge_begin();

/*  House-keeping: call every loop() so AsyncWebServer can service clients */
void WebBridge_loop();

/*  Broadcast a pre-built binary frame to every connected browser tab  */
void WebBridge_sendFrame(const uint8_t *buf, size_t len);
