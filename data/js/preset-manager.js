/**
 * Preset Manager Module
 * Handles preset save/load/delete operations with ESP32
 */

import { sendFrame, CMD } from './websocket-client.js';
import { ParameterModel, IDToName } from './ParameterModel.js';

// State
let nextPresetId = 1;
let esp32Presets = new Map();
let logCallback = null;

/**
 * Initialize preset manager
 * @param {Function} logFunc - Logging function
 */
export function initPresetManager(logFunc) {
  logCallback = logFunc;
}

/**
 * Create a full preset object from current UI state
 * @param {Object} domElements - Object containing DOM element references
 * @returns {Object} Preset data
 */
export function createFullPreset(domElements) {
  const preset = {};

  // Get all input field values
  document.querySelectorAll("input[type=number]").forEach(input => {
    preset[input.name] = parseFloat(input.value) || 0;
  });

  // Get checkbox values
  preset.runYoke = domElements.runYoke.checked;
  preset.runConveyor = domElements.runConveyor.checked;

  return preset;
}

/**
 * Apply a full preset to the model and UI
 * @param {Object} data - Preset data
 * @param {ParameterModel} model - Parameter model
 * @param {Function} updateCallbacks - Callback object with {updateRunAllState, updateDerived}
 */
export function applyFullPreset(data, model, updateCallbacks) {
  // Update model
  for (const [field, value] of Object.entries(data)) {
    if (field in model) {
      model.update(field, Number(value));
    }
  }

  // Update UI inputs
  for (const [field, value] of Object.entries(data)) {
    const ip = document.querySelector(`input[name="${field}"]`);
    if (ip) {
      if (ip.type === "checkbox") ip.checked = !!value;
      else ip.value = value;
    }
  }

  // Update run states
  const runYoke = document.getElementById("runYoke");
  const runConveyor = document.getElementById("runConveyor");
  if (runYoke) runYoke.checked = !!model.runYoke;
  if (runConveyor) runConveyor.checked = !!model.runConveyor;

  // Trigger UI updates
  if (updateCallbacks.updateRunAllState) updateCallbacks.updateRunAllState();
  if (updateCallbacks.updateDerived) updateCallbacks.updateDerived(model);

  // Send to MCU
  const serialDict = model.toSerialDict();
  for (const [id, v] of Object.entries(serialDict)) {
    sendFrame(CMD.SET, Number(id), v);
  }
}

/**
 * Save a full preset to ESP32 with all fields
 * @param {number} presetId - Preset ID
 * @param {string} presetName - Preset name
 * @param {Object} presetData - Preset data object
 * @returns {Promise} Resolves when save is complete
 */
export async function saveFullPresetToESP32(presetId, presetName, presetData) {
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      reject(new Error("Timeout waiting for ESP32 preset save"));
    }, 10000);

    let fieldsToSend = 0;
    let fieldsReceived = 0;

    const handleSaveAck = (event) => {
      const { id, value } = event.detail;

      if (id === 0xF9 && value === presetId) {
        // ESP32 ready to receive field data
        sendAllFieldsToESP32(presetData, presetName);
        return;
      }

      if (id === 0xFA) {
        // Field received confirmation
        fieldsReceived++;
        if (fieldsReceived >= fieldsToSend) {
          // All fields sent, complete the save
          sendFrame(CMD.SAVE_PRESET_COMPLETE, 0x00, presetId);
        }
        return;
      }

      if (id === 0xF1 && value === presetId) {
        // Save complete
        clearTimeout(timeout);
        window.removeEventListener('esp32-ack', handleSaveAck);
        resolve();
      }
    };

    const sendAllFieldsToESP32 = (data, name) => {
      const fieldMap = {
        feedRate: 0x20,
        loopHeightInput: 0x21,
        loopLength: 0x22,
        retractLength: 0x23,
        degSec: 0x24,
        yokeLength: 0x25,
        muSteps: 0x26,
        degStep: 0x27,
        driveDiameter: 0x28,
        minAngleRaw: 0x29,
        servoPwmMin: 0x2A,
        servoPwmMax: 0x2B
      };
      const nameChunks = 8;
      fieldsToSend = Object.keys(fieldMap).length + nameChunks;

      // Send each field
      Object.entries(fieldMap).forEach(([fieldName, fieldId]) => {
        const value = data[fieldName] || 0;
        sendFrame(CMD.SAVE_PRESET_FIELD, fieldId, value);
      });

      // Send name in 8 chunks of 4 bytes
      const enc = new TextEncoder();
      const bytes = new Uint8Array(32);
      bytes.fill(0);
      const encBytes = enc.encode(name);
      bytes.set(encBytes.slice(0, 32));
      for (let i = 0; i < nameChunks; i++) {
        const dv = new DataView(bytes.buffer, i * 4, 4);
        const floatVal = dv.getFloat32(0, true);
        sendFrame(CMD.SAVE_PRESET_FIELD, 0x30 + i, floatVal);
      }
    };

    window.addEventListener('esp32-ack', handleSaveAck);

    // Start the save process
    sendFrame(CMD.SAVE_PRESET_WITH_DATA, 0x00, presetId);
  });
}

/**
 * Load a preset from ESP32
 * @param {number} presetId - Preset ID to load
 * @param {ParameterModel} model - Parameter model
 * @param {Function} applyCallback - Callback to apply preset data
 * @returns {Promise} Resolves when load is complete
 */
export async function loadPresetFromESP32(presetId, model, applyCallback) {
  return new Promise((resolve, reject) => {
    let receivedData = {};

    const timeout = setTimeout(() => {
      reject(new Error("Timeout waiting for ESP32 preset load"));
    }, 5000);

    const handleLoadAck = (event) => {
      const { id, value } = event.detail;

      if (id === 0xF2 && value === presetId) {
        // Load complete
        clearTimeout(timeout);
        window.removeEventListener('esp32-ack', handleLoadAck);

        // Apply ALL received data
        applyCallback(receivedData);
        resolve();
        return;
      }

      // Map received field IDs back to field names
      const fieldMap = {
        0x20: 'feedRate',
        0x21: 'loopHeightInput',
        0x22: 'loopLength',
        0x23: 'retractLength',
        0x24: 'degSec',
        0x25: 'yokeLength',
        0x26: 'muSteps',
        0x27: 'degStep',
        0x28: 'driveDiameter',
        0x29: 'minAngleRaw',
        0x2A: 'servoPwmMin',
        0x2B: 'servoPwmMax'
      };

      if (fieldMap[id]) {
        receivedData[fieldMap[id]] = value;
      }

      // Also handle MCU parameters
      const mcuField = IDToName[id];
      if (mcuField) {
        receivedData[mcuField] = value;
      }
    };

    window.addEventListener('esp32-ack', handleLoadAck);
    sendFrame(CMD.LOAD_PRESET, 0x00, presetId);
  });
}

/**
 * Load the list of presets from ESP32
 * @param {HTMLSelectElement} dropdown - Preset dropdown element
 * @returns {Promise} Resolves when list is loaded
 */
export async function loadESP32PresetList(dropdown) {
  return new Promise((resolve, reject) => {
    esp32Presets.clear();

    const timeout = setTimeout(() => {
      reject(new Error("Timeout waiting for ESP32 preset list"));
    }, 5000);

    let pendingId = null;
    let nameBytes = [];

    const handlePresetListAck = (event) => {
      const { id, value } = event.detail;

      if (id === 0xF3) { // PRESET_COUNT
        // Count received - preset IDs will follow
        return;
      }
      else if (id === 0xF5) { // PRESET_EXISTS
        pendingId = value;
        nameBytes = [];
      }
      else if (id === 0xF6 && pendingId !== null) {
        const dv = new DataView(new ArrayBuffer(4));
        dv.setFloat32(0, value, true);
        for (let i = 0; i < 4; i++) nameBytes.push(dv.getUint8(i));
        if (nameBytes.length >= 32) {
          const dec = new TextDecoder();
          const name = dec.decode(new Uint8Array(nameBytes)).replace(/\0.*$/, '') || `ESP32 Preset ${pendingId}`;
          esp32Presets.set(pendingId, name);
          const existingOption = dropdown.querySelector(`option[data-preset-id="${pendingId}"]`);
          if (!existingOption) {
            const opt = document.createElement("option");
            opt.value = opt.textContent = name;
            opt.dataset.presetId = pendingId;
            opt.dataset.source = "esp32";
            dropdown.appendChild(opt);
          } else {
            existingOption.textContent = name;
            existingOption.value = name;
          }
          pendingId = null;
          nameBytes = [];
        }
      }
      else if (id === 0xF8) { // LIST_END
        clearTimeout(timeout);
        window.removeEventListener('esp32-ack', handlePresetListAck);

        // Determine the next available preset ID
        const ids = Array.from(esp32Presets.keys());
        nextPresetId = ids.length > 0 ? Math.max(...ids) + 1 : 1;

        resolve();
      }
    };

    window.addEventListener('esp32-ack', handlePresetListAck);
    sendFrame(CMD.LIST_PRESETS, 0x00, 0);
  });
}

/**
 * Delete a preset from ESP32
 * @param {number} presetId - Preset ID to delete
 */
export function deletePreset(presetId) {
  sendFrame(CMD.DELETE_PRESET, 0, presetId);
}

/**
 * Setup delete handler to remove from UI when ACK received
 * @param {HTMLSelectElement} dropdown - Preset dropdown element
 */
export function setupDeleteHandler(dropdown) {
  window.addEventListener('esp32-ack', (ev) => {
    const { id, value } = ev.detail;
    if (id === 0xF4) { // DELETE_ACK
      esp32Presets.delete(value);
      const opt = dropdown.querySelector(`option[data-preset-id="${value}"]`);
      if (opt) opt.remove();
    }
  });
}

/**
 * Get next available preset ID
 */
export function getNextPresetId() {
  return nextPresetId;
}

/**
 * Increment next preset ID
 */
export function incrementNextPresetId() {
  nextPresetId++;
}

/**
 * Add preset to map and dropdown
 * @param {number} presetId - Preset ID
 * @param {string} name - Preset name
 * @param {HTMLSelectElement} dropdown - Preset dropdown
 */
export function addPresetToUI(presetId, name, dropdown) {
  esp32Presets.set(presetId, name);

  const opt = document.createElement("option");
  opt.value = opt.textContent = name;
  opt.dataset.presetId = presetId;
  opt.dataset.presetName = name;
  dropdown.appendChild(opt);
}

/**
 * Get preset count
 */
export function getPresetCount() {
  return esp32Presets.size;
}

/**
 * Internal logging function
 */
function log(message) {
  if (logCallback) {
    logCallback(message);
  }
}
