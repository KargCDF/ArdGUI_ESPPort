/*  ────────────────────────────────────────────────────────────────
    Stepper-Servo controller  ·  ESP32 version  ·  Binary protocol
    Adds a WebSocket transport (see WebBridge.*)
    ──────────────────────────────────────────────────────────────── */

#include <Arduino.h>
#include <cstring>
#include "WebBridge.h"          // Wi-Fi + WS bridge (Phase-1)
#include <Preferences.h>

/* =====================  HARDWARE LIBS  ========================= */
#include <ESP32Servo.h>
#include <AccelStepper.h>

/* =====================  BUILD CONSTANTS  ======================= */
constexpr bool     yokeInverted = false;   // true if linkage is flipped

/* --------── PINS ------------------------------------------------ */
constexpr uint8_t PWM_PIN  = 13;   // SG90 / MG90S PWM
constexpr uint8_t STEP_PIN = 25;   // TB6600 PUL-
constexpr uint8_t DIR_PIN  = 26;   // TB6600 DIR-
constexpr uint8_t EN_PIN   = 27;   // TB6600 EN-
constexpr uint8_t ENDSTOP_PIN = 33;          // NC switch → GND

/* —— debounce for ENDSTOP —— */
constexpr unsigned long DEBOUNCE_US = 1000000UL; // 1 s
volatile  unsigned long lastEndstopMicros = 0;
volatile  bool          endstopEvent      = false;

/* =====================  RUN-TIME VARIABLES  ===================== */
float riseTime   = 120.4873f, fallTime  = 120.4873f;
float stayHigh   = 159.0253f, stayLow   = 715.0f;
float minAngle   = 23.0f,     maxAngle  = 83.2437f;
float speedStepsPerSec = 39.7887f;

int   servoPwmMin = 600,      servoPwmMax = 2400;

volatile bool yokeRunning     = false;
volatile bool conveyorRunning = false;

/* ADD SERVO STATE MACHINE VARIABLES HERE (before they're used) */
enum SweepState { S_IDLE, S_RISE, S_HIGH, S_FALL, S_LOW };
SweepState sState = S_IDLE;
unsigned long lastEvent = 0;
unsigned long stepDelay = 0;
float currAngle = 0.0f;  // Current servo angle

/* =====================  OBJECTS  ================================ */
Servo        myServo;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Preferences prefs;           // global NVS storage

/* =====================  PROTOCOL  =============================== */
enum ParamID : uint8_t {
  ParamID_RISE_TIME_MS      = 0x01,
  ParamID_FALL_TIME_MS      = 0x02,
  ParamID_HOLD_HIGH_MS      = 0x03,
  ParamID_HOLD_LOW_MS       = 0x04,
  ParamID_MIN_ANGLE_DEG     = 0x05,
  ParamID_MAX_ANGLE_DEG     = 0x06,
  ParamID_STEPS_PER_SEC     = 0x07,
  ParamID_SERVO_PWM_MIN_US  = 0x08,
  ParamID_SERVO_PWM_MAX_US  = 0x09,
  ParamID_RUN_YOKE          = 0x10,
  ParamID_RUN_CONVEYOR      = 0x11
};

constexpr uint8_t SYNC      = 0xAA;
constexpr uint8_t CMD_SET   = 0x01;
constexpr uint8_t CMD_TOGGLE= 0x02;
constexpr uint8_t CMD_GET   = 0x03;    // host → MCU
constexpr uint8_t CMD_ACK   = 0x81;    // MCU  → host
constexpr uint8_t CMD_SAVE_NVS = 0x20;   // host  → MCU : store current params
constexpr uint8_t CMD_LOAD_NVS = 0x21;   // host  → MCU : reload params, ACK dump
constexpr uint8_t CMD_SAVE_PRESET = 0x30;   // host → MCU: save current state as preset
constexpr uint8_t CMD_LOAD_PRESET = 0x31;   // host → MCU: load preset by ID
constexpr uint8_t CMD_LIST_PRESETS = 0x32;  // host → MCU: get count of saved presets
constexpr uint8_t CMD_DELETE_PRESET = 0x33; // host → MCU: delete preset by ID
constexpr uint8_t CMD_SAVE_PRESET_WITH_DATA = 0x34; // New command
constexpr uint8_t CMD_SAVE_PRESET_FIELD = 0x35;
constexpr uint8_t CMD_SAVE_PRESET_COMPLETE = 0x36;

union FloatBytes { float f; uint8_t b[4]; };

struct Frame {
  uint8_t   sync, cmd, id;
  FloatBytes data;
  uint8_t   crc;
};

/* =====================  PRESET STRUCTURE (IMPROVEMENT)  ========= */
#pragma pack(1)  // Ensure no padding
struct Preset {
  // MCU parameters
  float riseTime;
  float fallTime;
  float stayHigh;
  float stayLow;
  float minAngle;
  float maxAngle;
  float speedStepsPerSec;
  float servoPwmMin;
  float servoPwmMax;
  
  // Browser editable fields (ALL user inputs)
  float feedRate;
  float loopHeightInput;
  float loopLength;
  float retractLength;
  float degSec;
  float yokeLength;
  float muSteps;
  float degStep;
  float driveDiameter;
  float minAngleRaw;
  float servoPwmMinUser;
  float servoPwmMaxUser;
  
  // Checkboxes
  uint8_t runYoke;
  uint8_t runConveyor;

  char name[32];
};
#pragma pack()

/* =====================  PRESET TRANSFER GLOBALS  =============== */
uint8_t currentPresetId = 0;
Preset currentPresetData;

/* =====================  FORWARD DECLARATIONS  =================== */
void applyParam(uint8_t id, float val);
float readParam(uint8_t id);
void processSerial();
void processIncomingFrame(uint8_t *buf, size_t len);
void updateServo();
void IRAM_ATTR endstopISR();
void sendFrame(uint8_t cmd, uint8_t id, float val);
void saveToNVS();
void loadFromNVS();
void savePresetToNVS(uint8_t presetId);
void loadPresetFromNVS(uint8_t presetId);
uint8_t getPresetCount();
void deletePresetFromNVS(uint8_t presetId);
void savePresetToNVSWithData(uint8_t presetId, const Preset& presetData);

/* =====================  TRANSPORT LAYER  ======================== *
 * Build a frame once → ship it over Serial **and** WebSocket      */
static void transmitFrame(const uint8_t *buf, size_t len)
{
  Serial.write(buf, len);              // USB-Serial
  WebBridge_sendFrame(buf, len);       // Wi-Fi / browser
}

/* =====================  PARAM HELPERS  ========================== */
inline void writeYoke(int logicalDeg)
{
  myServo.write(yokeInverted ? 180 - logicalDeg : logicalDeg);
}

float readParam(uint8_t id)
{
  switch (id) {
    case ParamID_RISE_TIME_MS:        return riseTime;
    case ParamID_FALL_TIME_MS:        return fallTime;
    case ParamID_HOLD_HIGH_MS:        return stayHigh;
    case ParamID_HOLD_LOW_MS:         return stayLow;
    case ParamID_MIN_ANGLE_DEG:       return minAngle;
    case ParamID_MAX_ANGLE_DEG:       return maxAngle;
    case ParamID_STEPS_PER_SEC:       return speedStepsPerSec;
    case ParamID_SERVO_PWM_MIN_US:    return servoPwmMin;
    case ParamID_SERVO_PWM_MAX_US:    return servoPwmMax;
    case ParamID_RUN_YOKE:            return yokeRunning;
    case ParamID_RUN_CONVEYOR:        return conveyorRunning;
    default:                          return 0.0f;
  }
}

/* =====================  APPLY PARAM  ============================ */
void applyParam(uint8_t id, float val)
{
  switch (id) {
    case ParamID_RISE_TIME_MS:   riseTime = val; break;
    case ParamID_FALL_TIME_MS:   fallTime = val; break;
    case ParamID_HOLD_HIGH_MS:   stayHigh = val; break;
    case ParamID_HOLD_LOW_MS:    stayLow  = val; break;

    case ParamID_MIN_ANGLE_DEG:
      minAngle = constrain(val, 5.0f, 180.0f);
      // If not running, move to new minAngle for live preview
      if (!yokeRunning) {
        currAngle = minAngle;
        writeYoke(currAngle);
      }
      break;

    case ParamID_MAX_ANGLE_DEG:
      maxAngle = constrain(val, 6.0f, 180.0f);
      // If not running and currently at max, update position
      if (!yokeRunning && currAngle >= maxAngle - 1.0f) {
        currAngle = maxAngle;
        writeYoke(currAngle);
      }
      break;

    case ParamID_STEPS_PER_SEC:
      speedStepsPerSec = max(1.0f, val);
      stepper.setMaxSpeed(speedStepsPerSec);
      stepper.setSpeed(speedStepsPerSec);
      break;

    case ParamID_RUN_YOKE:
      if (val != 0) {
        yokeRunning = true;
        myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
        sState = S_IDLE;  // Reset state machine
      } else {
        yokeRunning = false;
        // Move to minAngle when stopping
        currAngle = minAngle;
        writeYoke(currAngle);
        sState = S_IDLE;
      }
      break;

    case ParamID_RUN_CONVEYOR:
      conveyorRunning = (val != 0);
      if (conveyorRunning) stepper.enableOutputs();
      else                 stepper.disableOutputs();
      break;

    case ParamID_SERVO_PWM_MIN_US:
      servoPwmMin = constrain((int)val, 100, 3000);
      myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
      break;

    case ParamID_SERVO_PWM_MAX_US:
      servoPwmMax = constrain((int)val, 100, 3000);
      myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
      break;
  }
}

/* =====================  SEND FRAME  ============================ */
void sendFrame(uint8_t cmd, uint8_t id, float val)
{
  uint8_t frame[1 + 1 + 1 + 4 + 1];  // SYNC,cmd,id,val,crc
  uint8_t pos = 0;

  frame[pos++] = SYNC;
  frame[pos++] = cmd;
  frame[pos++] = id;

  FloatBytes fb{ .f = val };
  memcpy(&frame[pos], fb.b, 4);  pos += 4;

  uint8_t crc = cmd ^ id;
  for (uint8_t b : fb.b) crc ^= b;
  frame[pos++] = crc;

  transmitFrame(frame, pos);
}

/* =====================  WS → MCU PARSER  (NEW) ================= */
void processIncomingFrame(uint8_t *buf, size_t len)
{
  if (len != 8 || buf[0] != SYNC) return;

  uint8_t cmd = buf[1], id = buf[2];
  FloatBytes fb; memcpy(fb.b, &buf[3], 4);
  uint8_t rxCrc = buf[7];

  uint8_t calc = cmd ^ id;
  for (uint8_t b : fb.b) calc ^= b;
  if (calc != rxCrc) return;

  if (cmd == CMD_SAVE_NVS) {
      saveToNVS();
      sendFrame(CMD_ACK, 0xF0, 1);
      return;
  }
  else if (cmd == CMD_LOAD_NVS) {
      loadFromNVS();
      for (uint8_t id = ParamID_RISE_TIME_MS; id <= ParamID_RUN_CONVEYOR; ++id)
          sendFrame(CMD_ACK, id, readParam(id));
      return;
  }
  else if (cmd == CMD_SAVE_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, 1, 100); // Safety check
    savePresetToNVS(presetId);
    sendFrame(CMD_ACK, 0xF1, presetId);
    return;
  }
  else if (cmd == CMD_LOAD_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, 1, 100); // Safety check
    loadPresetFromNVS(presetId);
    
    // Send all parameters back
    sendFrame(CMD_ACK, ParamID_RISE_TIME_MS, riseTime);
    sendFrame(CMD_ACK, ParamID_FALL_TIME_MS, fallTime);
    sendFrame(CMD_ACK, ParamID_HOLD_HIGH_MS, stayHigh);
    sendFrame(CMD_ACK, ParamID_HOLD_LOW_MS, stayLow);
    sendFrame(CMD_ACK, ParamID_MIN_ANGLE_DEG, minAngle);
    sendFrame(CMD_ACK, ParamID_MAX_ANGLE_DEG, maxAngle);
    sendFrame(CMD_ACK, ParamID_STEPS_PER_SEC, speedStepsPerSec);
    sendFrame(CMD_ACK, ParamID_SERVO_PWM_MIN_US, servoPwmMin);
    sendFrame(CMD_ACK, ParamID_SERVO_PWM_MAX_US, servoPwmMax);
    sendFrame(CMD_ACK, ParamID_RUN_YOKE, yokeRunning ? 1.0f : 0.0f);
    sendFrame(CMD_ACK, ParamID_RUN_CONVEYOR, conveyorRunning ? 1.0f : 0.0f);
    
    sendFrame(CMD_ACK, 0xF2, presetId);  // End marker
    return;
  }
  else if (cmd == CMD_LIST_PRESETS) {
    prefs.begin("presets", true);
    uint8_t count = prefs.getUChar("count", 0);
    
    Serial.printf("[DEBUG] Sending preset count: %d\n", count);
    sendFrame(CMD_ACK, 0xF3, count);
    
    // Send each existing preset ID and name
    for (uint8_t i = 1; i <= 100; i++) {
      if (prefs.isKey(String(i).c_str())) {
        Preset preset = {};
        prefs.getBytes(String(i).c_str(), &preset, sizeof(Preset));
        Serial.printf("[DEBUG] Found preset ID: %d name: %s\n", i, preset.name);
        sendFrame(CMD_ACK, 0xF5, i);  // 0xF5 = PRESET_EXISTS
        for (uint8_t c = 0; c < 8; c++) {
          FloatBytes fbName;
          memcpy(fbName.b, &preset.name[c*4], 4);
          sendFrame(CMD_ACK, 0xF6, fbName.f);  // send name chunk
        }
      }
    }
    
    Serial.println("[DEBUG] Sending list end marker");
    sendFrame(CMD_ACK, 0xF8, 0);  // 0xF8 = LIST_END
    
    prefs.end();
    return;
  }
  else if (cmd == CMD_DELETE_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, 1, 100); // Safety check
    deletePresetFromNVS(presetId);
    sendFrame(CMD_ACK, 0xF4, presetId);
    return;
  }
  else if (cmd == CMD_SAVE_PRESET_WITH_DATA) {
    // This command will be followed by individual field updates
    uint8_t presetId = constrain((uint8_t)fb.f, 1, 100);
    
    // Store the preset ID for the upcoming data
    currentPresetId = presetId;
    currentPresetData = {}; // Reset struct
    memset(currentPresetData.name, 0, sizeof(currentPresetData.name));
    
    // Copy current MCU values
    currentPresetData.riseTime = riseTime;
    currentPresetData.fallTime = fallTime;
    currentPresetData.stayHigh = stayHigh;
    currentPresetData.stayLow = stayLow;
    currentPresetData.minAngle = minAngle;
    currentPresetData.maxAngle = maxAngle;
    currentPresetData.speedStepsPerSec = speedStepsPerSec;
    currentPresetData.servoPwmMin = (float)servoPwmMin;
    currentPresetData.servoPwmMax = (float)servoPwmMax;
    currentPresetData.runYoke = yokeRunning ? 1 : 0;
    currentPresetData.runConveyor = conveyorRunning ? 1 : 0;
    
    sendFrame(CMD_ACK, 0xF9, presetId); // Ready to receive data
    return;
  }
  else if (cmd == CMD_SAVE_PRESET_FIELD) {
    // Save individual field to current preset data
    uint8_t fieldId = id;
    float value = fb.f;
    
    switch(fieldId) {
      case 0x20: currentPresetData.feedRate = value; break;
      case 0x21: currentPresetData.loopHeightInput = value; break;
      case 0x22: currentPresetData.loopLength = value; break;
      case 0x23: currentPresetData.retractLength = value; break;
      case 0x24: currentPresetData.degSec = value; break;
      case 0x25: currentPresetData.yokeLength = value; break;
      case 0x26: currentPresetData.muSteps = value; break;
      case 0x27: currentPresetData.degStep = value; break;
      case 0x28: currentPresetData.driveDiameter = value; break;
      case 0x29: currentPresetData.minAngleRaw = value; break;
      case 0x2A: currentPresetData.servoPwmMinUser = value; break;
      case 0x2B: currentPresetData.servoPwmMaxUser = value; break;
      case 0x30: memcpy(&currentPresetData.name[0], fb.b, 4); break;
      case 0x31: memcpy(&currentPresetData.name[4], fb.b, 4); break;
      case 0x32: memcpy(&currentPresetData.name[8], fb.b, 4); break;
      case 0x33: memcpy(&currentPresetData.name[12], fb.b, 4); break;
      case 0x34: memcpy(&currentPresetData.name[16], fb.b, 4); break;
      case 0x35: memcpy(&currentPresetData.name[20], fb.b, 4); break;
      case 0x36: memcpy(&currentPresetData.name[24], fb.b, 4); break;
      case 0x37: memcpy(&currentPresetData.name[28], fb.b, 4); break;
    }
    
    sendFrame(CMD_ACK, 0xFA, fieldId); // Field received
    return;
  }
  else if (cmd == CMD_SAVE_PRESET_COMPLETE) {
    // Finalize the preset save
    uint8_t presetId = constrain((uint8_t)fb.f, 1, 100);
    savePresetToNVSWithData(presetId, currentPresetData);
    sendFrame(CMD_ACK, 0xF1, presetId); // Save complete
    return;
  }

  // Handle regular commands
  if (cmd == CMD_SET || cmd == CMD_TOGGLE) {
    applyParam(id, fb.f);
    sendFrame(CMD_ACK, id, readParam(id));
  }
  else if (cmd == CMD_GET) {
    if (id == 0x00) {
      for (uint8_t pid = ParamID_RISE_TIME_MS; pid <= ParamID_RUN_CONVEYOR; ++pid)
        sendFrame(CMD_ACK, pid, readParam(pid));
    } else {
      sendFrame(CMD_ACK, id, readParam(id));
    }
  }
}

/* =====================  SERIAL PARSER  (unchanged) ============== */
enum RxState { WAIT_SYNC, READ_HDR, READ_DATA, READ_CRC };
RxState   rxState = WAIT_SYNC;
Frame     rxFrame;
uint8_t   idx     = 0;
uint8_t   calcCrc = 0;

void processSerial()
{
  while (Serial.available()) {
    uint8_t byteIn = Serial.read();

    switch (rxState) {
      case WAIT_SYNC:
        if (byteIn == SYNC) { rxState = READ_HDR; idx = 0; calcCrc = 0; }
        break;

      case READ_HDR:
        if (idx == 0) rxFrame.cmd = byteIn; else rxFrame.id = byteIn;
        calcCrc ^= byteIn;
        if (++idx == 2) { rxState = READ_DATA; idx = 0; }
        break;

      case READ_DATA:
        rxFrame.data.b[idx++] = byteIn;
        calcCrc ^= byteIn;
        if (idx == 4) rxState = READ_CRC;
        break;

      case READ_CRC:
        if (calcCrc == byteIn) {
          if (rxFrame.cmd == CMD_SET || rxFrame.cmd == CMD_TOGGLE) {
            applyParam(rxFrame.id, rxFrame.data.f);
            sendFrame(CMD_ACK, rxFrame.id, readParam(rxFrame.id));
          }
          else if (rxFrame.cmd == CMD_GET) {
            if (rxFrame.id == 0x00) {
              for (uint8_t pid = ParamID_RISE_TIME_MS; pid <= ParamID_RUN_CONVEYOR; ++pid)
                sendFrame(CMD_ACK, pid, readParam(pid));
            } else {
              sendFrame(CMD_ACK, rxFrame.id, readParam(rxFrame.id));
            }
          }
        }
        rxState = WAIT_SYNC;
        break;
    }
  }
}

/* =====================  Load Presets (NVS)  ===================== */
void loadFromNVS()
{
  prefs.begin("params", true);        // read-only
  auto get = [&](const char* key, float fallback){
    return prefs.isKey(key) ? prefs.getFloat(key) : fallback;
  };

  riseTime         = get("rise",  riseTime);
  fallTime         = get("fall",  fallTime);
  stayHigh         = get("hHigh", stayHigh);
  stayLow          = get("hLow",  stayLow);
  minAngle         = get("angMin",minAngle);
  maxAngle         = get("angMax",maxAngle);
  speedStepsPerSec = get("steps", speedStepsPerSec);
  servoPwmMin      = get("pwmMin", servoPwmMin);
  servoPwmMax      = get("pwmMax", servoPwmMax);
  prefs.end();
}

/* =====================  Save Presets (NVS)  ===================== */
void saveToNVS()
{
  prefs.begin("params", false);   // read-write
  prefs.putFloat("rise",  riseTime);
  prefs.putFloat("fall",  fallTime);
  prefs.putFloat("hHigh", stayHigh);
  prefs.putFloat("hLow",  stayLow);
  prefs.putFloat("angMin",minAngle);
  prefs.putFloat("angMax",maxAngle);
  prefs.putFloat("steps", speedStepsPerSec);
  prefs.putFloat("pwmMin", servoPwmMin);
  prefs.putFloat("pwmMax", servoPwmMax);
  prefs.end();
}

/* =====================  SERVO STATE MACHINE  ==================== */
void updateServo()
{
  if (!yokeRunning) { 
    sState = S_IDLE; 
    // When stopping, move to minAngle position
    currAngle = minAngle;
    writeYoke(currAngle);
    return; 
  }

  // const float angleRange = maxAngle - minAngle;
  const float dirUp   = yokeInverted ? -1.0f : +1.0f;
  const float dirDown = -dirUp;
  unsigned long now = millis();

  // Calculate step size for smoother movement (1 degree steps max)
  // const float stepSize = min(1.0f, angleRange / 10.0f);

  switch (sState) {
    case S_IDLE:
      currAngle = yokeInverted ? maxAngle : minAngle;
      writeYoke(currAngle);
      stepDelay = riseTime / max(1.0f, maxAngle - minAngle);
      lastEvent = now;
      sState    = S_RISE;
      break;

    case S_RISE:
      if (now - lastEvent >= stepDelay) {
        lastEvent = now;
        currAngle += dirUp;
        
        // Check if we've reached maxAngle
        if ((!yokeInverted && currAngle >= maxAngle) ||
            ( yokeInverted && currAngle <= minAngle)) {
          currAngle = yokeInverted ? minAngle : maxAngle;
          writeYoke(currAngle);
          sState = S_HIGH; 
          lastEvent = now;
        } else {
          writeYoke(currAngle);
        }
      }
      break;

    case S_HIGH:
      if (now - lastEvent >= stayHigh) {
        stepDelay = fallTime / max(1.0f, maxAngle - minAngle);
        sState = S_FALL; 
        lastEvent = now;
      }
      break;

    case S_FALL:
      if (now - lastEvent >= stepDelay) {
        lastEvent = now;
        currAngle += dirDown;
        
        // Check if we've reached minAngle
        if ((!yokeInverted && currAngle <= minAngle) ||
            ( yokeInverted && currAngle >= maxAngle)) {
          currAngle = yokeInverted ? maxAngle : minAngle;
          writeYoke(currAngle);
          sState = S_LOW; 
          lastEvent = now;
        } else {
          writeYoke(currAngle);
        }
      }
      break;

    case S_LOW:
      if (now - lastEvent >= stayLow) {
        stepDelay = fallTime / max(1.0f, maxAngle - minAngle);
        sState = S_RISE; 
        lastEvent = now;
      }
      break;
  }
}

/* =====================  IMPROVED PRESET MANAGEMENT (PURE BINARY) ============= */
void savePresetToNVS(uint8_t presetId) {
  if (presetId == 0 || presetId > 100) return;
  
  prefs.begin("presets", false);
  
  // Use struct for cleaner code - no strings, no magic numbers
  Preset preset = {
    .riseTime = riseTime,
    .fallTime = fallTime,
    .stayHigh = stayHigh,
    .stayLow = stayLow,
    .minAngle = minAngle,
    .maxAngle = maxAngle,
    .speedStepsPerSec = speedStepsPerSec,
    .servoPwmMin = (float)servoPwmMin,
    .servoPwmMax = (float)servoPwmMax
  };

  snprintf(preset.name, sizeof(preset.name), "Preset %u", presetId);
  
  // Save binary preset data directly using preset ID as key
  prefs.putBytes(String(presetId).c_str(), &preset, sizeof(Preset));
  Serial.printf("[DEBUG] Saved preset ID %d, size: %d bytes\n", presetId, sizeof(Preset));
  
  // Verify it was saved
  if (prefs.isKey(String(presetId).c_str())) {
    Serial.printf("[DEBUG] Preset %d successfully saved to NVS\n", presetId);
  } else {
    Serial.printf("[DEBUG] ERROR: Preset %d NOT found in NVS after save!\n", presetId);
  }
  
  // Update preset count
  uint8_t presetCount = prefs.getUChar("count", 0);
  bool found = false;
  for (uint8_t i = 1; i <= presetCount; i++) {
    if (prefs.isKey(String(i).c_str())) {
      if (i == presetId) {
        found = true;
        break;
      }
    }
  }
  if (!found && presetCount < 100) {  // Limit to 100 presets
    prefs.putUChar("count", presetCount + 1);
  }
  
  prefs.end();
}

void loadPresetFromNVS(uint8_t presetId) {
  if (presetId == 0 || presetId > 100) return;
  
  prefs.begin("presets", true);
  size_t dataSize = prefs.getBytesLength(String(presetId).c_str());
  
  if (dataSize == sizeof(Preset) || dataSize == 86) {
    Preset preset = {};
    size_t bytesRead = prefs.getBytes(String(presetId).c_str(), &preset, min(dataSize, sizeof(Preset)));
    prefs.end();

    if (bytesRead == dataSize) {
      // Apply MCU values
      riseTime = constrain(preset.riseTime, 1.0f, 10000.0f);
      fallTime = constrain(preset.fallTime, 1.0f, 10000.0f);
      stayHigh = constrain(preset.stayHigh, 0.0f, 10000.0f);
      stayLow = constrain(preset.stayLow, 0.0f, 10000.0f);
      minAngle = constrain(preset.minAngle, 0.0f, 180.0f);
      maxAngle = constrain(preset.maxAngle, 0.0f, 180.0f);
      speedStepsPerSec = constrain(preset.speedStepsPerSec, 1.0f, 10000.0f);
      servoPwmMin = constrain((int)preset.servoPwmMin, 100, 3000);
      servoPwmMax = constrain((int)preset.servoPwmMax, 100, 3000);
      yokeRunning = preset.runYoke != 0;
      conveyorRunning = preset.runConveyor != 0;
      
      // Update hardware
      myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
      stepper.setMaxSpeed(speedStepsPerSec);
      stepper.setSpeed(speedStepsPerSec);
      
      // Send MCU parameters
      sendFrame(CMD_ACK, ParamID_RISE_TIME_MS, riseTime);
      sendFrame(CMD_ACK, ParamID_FALL_TIME_MS, fallTime);
      sendFrame(CMD_ACK, ParamID_HOLD_HIGH_MS, stayHigh);
      sendFrame(CMD_ACK, ParamID_HOLD_LOW_MS, stayLow);
      sendFrame(CMD_ACK, ParamID_MIN_ANGLE_DEG, minAngle);
      sendFrame(CMD_ACK, ParamID_MAX_ANGLE_DEG, maxAngle);
      sendFrame(CMD_ACK, ParamID_STEPS_PER_SEC, speedStepsPerSec);
      sendFrame(CMD_ACK, ParamID_SERVO_PWM_MIN_US, servoPwmMin);
      sendFrame(CMD_ACK, ParamID_SERVO_PWM_MAX_US, servoPwmMax);
      sendFrame(CMD_ACK, ParamID_RUN_YOKE, yokeRunning ? 1.0f : 0.0f);
      sendFrame(CMD_ACK, ParamID_RUN_CONVEYOR, conveyorRunning ? 1.0f : 0.0f);
      
      // Send browser fields
      sendFrame(CMD_ACK, 0x20, preset.feedRate);
      sendFrame(CMD_ACK, 0x21, preset.loopHeightInput);
      sendFrame(CMD_ACK, 0x22, preset.loopLength);
      sendFrame(CMD_ACK, 0x23, preset.retractLength);
      sendFrame(CMD_ACK, 0x24, preset.degSec);
      sendFrame(CMD_ACK, 0x25, preset.yokeLength);
      sendFrame(CMD_ACK, 0x26, preset.muSteps);
      sendFrame(CMD_ACK, 0x27, preset.degStep);
      sendFrame(CMD_ACK, 0x28, preset.driveDiameter);
      sendFrame(CMD_ACK, 0x29, preset.minAngleRaw);
      sendFrame(CMD_ACK, 0x2A, preset.servoPwmMinUser);
      sendFrame(CMD_ACK, 0x2B, preset.servoPwmMaxUser);
      
      sendFrame(CMD_ACK, 0xF2, presetId);  // End marker
    }
  } else {
    prefs.end();
  }
}

uint8_t getPresetCount() {
  prefs.begin("presets", true);
  uint8_t count = prefs.getUChar("count", 0);
  prefs.end();
  return min(count, (uint8_t)100); // Cap at 100
}

void deletePresetFromNVS(uint8_t presetId) {
  if (presetId == 0 || presetId > 100) return; // Safety check
  
  prefs.begin("presets", false);
  prefs.remove(String(presetId).c_str());
  
  // Update count (scan for remaining presets)
  uint8_t activeCount = 0;
  for (uint8_t i = 1; i <= 100; i++) {
    if (prefs.isKey(String(i).c_str())) {
      activeCount++;
    }
  }
  prefs.putUChar("count", activeCount);
  prefs.end();
}

/* =====================  ISR  ================================== */
void IRAM_ATTR endstopISR()
{
  unsigned long now = micros();
  if (now - lastEndstopMicros >= DEBOUNCE_US) {
    lastEndstopMicros = now;
    endstopEvent = true;
  }
}

/* =====================  SETUP  ================================= */
void setup()
{
  Serial.begin(115200);

  loadFromNVS();

  myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
  currAngle = minAngle;  // Initialize current angle
  myServo.write(currAngle);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(true, true, false);
  stepper.enableOutputs();
  stepper.setMinPulseWidth(20);
  stepper.setMaxSpeed(speedStepsPerSec);
  stepper.setSpeed(speedStepsPerSec);

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), endstopISR, RISING);

  WebBridge_begin();          // ★ bring up Wi-Fi + WS
  sendFrame(CMD_ACK, 0xFF, 0.0f);   // “READY”
}

/* =====================  LOOP  ================================= */
void loop()
{
  processSerial();
  updateServo();
  if (conveyorRunning) stepper.runSpeed();

  if (endstopEvent) {
    noInterrupts();
    endstopEvent = false;
    interrupts();

    float newYokeState = yokeRunning ? 0.0f : 1.0f;
    applyParam(ParamID_RUN_YOKE, newYokeState);

    float newConvState = conveyorRunning ? 0.0f : 1.0f;
    applyParam(ParamID_RUN_CONVEYOR, newConvState);

    sendFrame(CMD_ACK, ParamID_RUN_YOKE,      newYokeState);
    sendFrame(CMD_ACK, ParamID_RUN_CONVEYOR,  newConvState);
  }

  WebBridge_loop();
}


void savePresetToNVSWithData(uint8_t presetId, const Preset& presetData) {
  if (presetId == 0 || presetId > 100) return;

  prefs.begin("presets", false);

  // Save the complete preset data
  Preset tmp = presetData;
  if (tmp.name[0] == '\0')
    snprintf(tmp.name, sizeof(tmp.name), "Preset %u", presetId);
  prefs.putBytes(String(presetId).c_str(), &tmp, sizeof(Preset));
  Serial.printf("[DEBUG] Saved full preset ID %d, size: %d bytes\n", presetId, sizeof(Preset));
  
  // Update preset count
  uint8_t presetCount = prefs.getUChar("count", 0);
  bool found = false;
  for (uint8_t i = 1; i <= presetCount; i++) {
    if (prefs.isKey(String(i).c_str())) {
      if (i == presetId) {
        found = true;
        break;
      }
    }
  }
  if (!found && presetCount < 100) {
    prefs.putUChar("count", presetCount + 1);
  }
  
  prefs.end();
}
