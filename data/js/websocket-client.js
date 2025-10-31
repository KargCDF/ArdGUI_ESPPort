/**
 * WebSocket Client Module
 * Handles WebSocket connection, frame communication, and logging
 */

import { buildFrame, parseFrame, CMD } from './proto.js';

// WebSocket instance
let ws = null;
let logCallback = null;

// Parameter name mapping for logging
const ParamNames = {
  0x01: "Rise Time", 0x02: "Fall Time", 0x03: "Hold High", 0x04: "Hold Low",
  0x05: "Min Angle", 0x06: "Max Angle", 0x07: "Steps/Sec",
  0x08: "Servo PWM Min", 0x09: "Servo PWM Max", 0x10: "Run Yoke", 0x11: "Run Conveyor"
};

/**
 * Initialize WebSocket connection
 * @param {string} url - WebSocket URL (defaults to ws://host/ws)
 * @param {Object} handlers - Event handlers {onOpen, onMessage, onClose, onError}
 * @param {Function} logFunc - Logging function
 */
export function initWebSocket(url, handlers = {}, logFunc = null) {
  logCallback = logFunc;

  ws = new WebSocket(url || `ws://${location.host}/ws`);
  ws.binaryType = "arraybuffer";

  ws.addEventListener("open", () => {
    log(`✓ Connected to ${ws.url}`);
    if (handlers.onOpen) handlers.onOpen();
  });

  ws.addEventListener("message", (ev) => {
    if (!(ev.data instanceof ArrayBuffer)) return;

    const info = parseFrame(new Uint8Array(ev.data));
    if (!info || info.cmd !== CMD.ACK) return;

    const paramName = ParamNames[info.id] || `0x${info.id.toString(16).padStart(2,"0")}`;
    log(`← ACK ${paramName}: ${info.value}`);

    if (handlers.onMessage) handlers.onMessage(info);

    // Emit global event for other modules
    window.dispatchEvent(new CustomEvent('esp32-ack', {
      detail: { id: info.id, value: info.value }
    }));
  });

  ws.addEventListener("close", () => {
    log("✗ WebSocket connection closed");
    if (handlers.onClose) handlers.onClose();
  });

  ws.addEventListener("error", (err) => {
    log(`✗ WebSocket error: ${err.message || 'Unknown error'}`);
    if (handlers.onError) handlers.onError(err);
  });

  return ws;
}

/**
 * Send a frame to the MCU
 * @param {number} cmd - Command byte
 * @param {number} id - Parameter ID
 * @param {number} val - Value to send
 */
export function sendFrame(cmd, id, val = 0) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    console.error("WebSocket not connected");
    return;
  }

  ws.send(buildFrame(cmd, id, val));
  const paramName = ParamNames[id] || `0x${id.toString(16).padStart(2,"0")}`;
  log(`→ ${paramName}: ${val}`);
}

/**
 * Get WebSocket instance
 */
export function getWebSocket() {
  return ws;
}

/**
 * Check if WebSocket is connected
 */
export function isConnected() {
  return ws && ws.readyState === WebSocket.OPEN;
}

/**
 * Internal logging function
 */
function log(message) {
  if (logCallback) {
    logCallback(message);
  } else {
    console.log(message);
  }
}

// Re-export CMD for convenience
export { CMD };
