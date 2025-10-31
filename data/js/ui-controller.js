/**
 * UI Controller Module
 * Handles UI initialization, event handlers, and form management
 */

import { ParameterModel, NameToID } from './ParameterModel.js';
import { sendFrame, CMD } from './websocket-client.js';

// Pending updates for debounced sending
const pendingDict = {};
let sendTimeout = null;
const DEBOUNCE_DELAY = 300;

/**
 * Initialize UI controller
 * @param {ParameterModel} model - Parameter model
 * @param {Object} domElements - DOM element references
 * @returns {Object} UI control functions
 */
export function initUI(model, domElements) {
  const {
    logBox, runAll, runYoke, runConveyor,
    out_loopHeight, out_holdHigh, out_holdLow,
    out_stepsSec, out_minTime, out_maxAngle
  } = domElements;

  // UI update functions
  function updateDerived(m) {
    out_loopHeight.textContent = m.loopHeight.toFixed(2);
    out_holdHigh.textContent = m.holdHighCalc.toFixed(3);
    out_holdLow.textContent = m.holdLowCalc.toFixed(3);
    out_stepsSec.textContent = m.stepsSecCalc.toFixed(2);
    out_minTime.textContent = m.minTime.toFixed(4);
    out_maxAngle.textContent = m.maxAngle.toFixed(2);
  }

  function updateRunAllState() {
    const yokeRunning = runYoke.checked;
    const conveyorRunning = runConveyor.checked;

    if (yokeRunning && conveyorRunning) {
      runAll.checked = true;
      runAll.indeterminate = false;
    } else if (!yokeRunning && !conveyorRunning) {
      runAll.checked = false;
      runAll.indeterminate = false;
    } else {
      runAll.checked = false;
      runAll.indeterminate = true;
    }
  }

  function bindModelToForm(m) {
    Object.entries(m.toJSON()).forEach(([k, v]) => {
      const ip = document.querySelector(`input[name="${k}"]`);
      if (!ip) return;
      if (ip.type === "checkbox") ip.checked = !!v;
      else ip.value = v;
    });
    runYoke.checked = !!m.runYoke;
    runConveyor.checked = !!m.runConveyor;
    updateRunAllState();
    updateDerived(m);
  }

  function setDisabled(state) {
    document.querySelectorAll("input,button").forEach(el => el.disabled = state);
  }

  function log(line) {
    logBox.value += line + "\n";
    logBox.scrollTop = logBox.scrollHeight;
  }

  // Debounced sending
  function scheduleSend() {
    if (sendTimeout) clearTimeout(sendTimeout);
    sendTimeout = setTimeout(() => {
      flushPending();
      sendTimeout = null;
    }, DEBOUNCE_DELAY);
  }

  function flushPending() {
    for (const [idStr, v] of Object.entries(pendingDict)) {
      const id = Number(idStr);
      sendFrame(CMD.SET, id, v);
    }
    Object.keys(pendingDict).forEach(k => delete pendingDict[k]);
  }

  function queueToggle(id, val) {
    sendFrame(CMD.TOGGLE, id, val ? 1 : 0);
  }

  function updateDerivedMCUParams() {
    const derivedUpdates = {
      [NameToID.holdHighRaw]: model.holdHighCalc,
      [NameToID.holdLowRaw]: model.holdLowCalc,
      [NameToID.stepsPerSecondRaw]: model.stepsSecCalc,
      [NameToID.maxAngleRaw]: model.maxAngle
    };

    Object.assign(pendingDict, derivedUpdates);
    scheduleSend();

    model.holdHighRaw = model.holdHighCalc;
    model.holdLowRaw = model.holdLowCalc;
    model.stepsPerSecondRaw = model.stepsSecCalc;
    model.maxAngleRaw = model.maxAngle;
  }

  // Initialize UI with model
  bindModelToForm(model);

  // Setup model event listeners
  model.addEventListener("preset", () => bindModelToForm(model));
  model.addEventListener("change", (e) => {
    updateDerived(model);

    if (['minAngleRaw', 'maxAngleRaw', 'degSec', 'yokeLength', 'loopHeightInput'].includes(e.detail.field)) {
      document.querySelector('input[name="riseTimeRaw"]').value = model.riseTimeRaw;
      document.querySelector('input[name="fallTimeRaw"]').value = model.fallTimeRaw;
    }
  });

  return {
    updateDerived,
    updateRunAllState,
    bindModelToForm,
    setDisabled,
    log,
    queueToggle,
    updateDerivedMCUParams,
    flushPending
  };
}

/**
 * Setup input handlers
 * @param {ParameterModel} model - Parameter model
 * @param {Object} uiControls - UI control functions
 */
export function setupInputHandlers(model, uiControls) {
  const { updateDerived, updateDerivedMCUParams } = uiControls;

  document.querySelectorAll("input[type=number]").forEach(ip => {
    ip.addEventListener("input", () => {
      updateDerived(model);
    });

    ip.addEventListener("change", () => {
      const newValue = Number(ip.value);
      if (isNaN(newValue)) return;

      model.update(ip.name, newValue);
      ip.value = model[ip.name];

      if (NameToID[ip.name]) {
        pendingDict[NameToID[ip.name]] = model[ip.name];
        scheduleSend();
      } else if (['feedRate', 'loopHeightInput', 'loopLength', 'retractLength', 'degSec', 'yokeLength', 'muSteps', 'degStep', 'driveDiameter'].includes(ip.name)) {
        updateDerivedMCUParams();
      }
    });
  });

  // Debounced sending helper
  function scheduleSend() {
    if (sendTimeout) clearTimeout(sendTimeout);
    sendTimeout = setTimeout(() => {
      for (const [idStr, v] of Object.entries(pendingDict)) {
        const id = Number(idStr);
        sendFrame(CMD.SET, id, v);
      }
      Object.keys(pendingDict).forEach(k => delete pendingDict[k]);
      sendTimeout = null;
    }, DEBOUNCE_DELAY);
  }
}

/**
 * Setup checkbox handlers
 * @param {Object} domElements - DOM element references
 * @param {Object} uiControls - UI control functions
 */
export function setupCheckboxHandlers(domElements, uiControls) {
  const { runAll, runYoke, runConveyor } = domElements;
  const { queueToggle, updateRunAllState } = uiControls;

  runYoke.addEventListener("change", e => {
    queueToggle(NameToID.runYoke, e.target.checked);
    updateRunAllState();
  });

  runConveyor.addEventListener("change", e => {
    queueToggle(NameToID.runConveyor, e.target.checked);
    updateRunAllState();
  });

  runAll.addEventListener("change", e => {
    const isRunning = e.target.checked;
    runYoke.checked = isRunning;
    runConveyor.checked = isRunning;
    queueToggle(NameToID.runYoke, isRunning);
    queueToggle(NameToID.runConveyor, isRunning);
  });
}

/**
 * Setup button handlers
 * @param {ParameterModel} model - Parameter model
 * @param {Object} uiControls - UI control functions
 */
export function setupButtonHandlers(model, uiControls) {
  const { log } = uiControls;

  document.getElementById("sendAll").addEventListener("click", () => {
    const serialDict = model.toSerialDict();
    for (const [id, v] of Object.entries(serialDict)) {
      sendFrame(CMD.SET, Number(id), v);
    }
  });

  document.getElementById("queryMcu").addEventListener("click", () =>
    sendFrame(CMD.GET, 0x00, 0));

  document.getElementById("clearLog").addEventListener("click", () =>
    (document.getElementById("log").value = ""));

  document.getElementById("loadConfig").addEventListener("click", () => {
    sendFrame(CMD.LOAD_NVS, 0, 0);
    log("→ LOAD_NVS");
  });
}
