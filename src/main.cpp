/*  ────────────────────────────────────────────────────────────────
    Stepper-Servo controller  ·  ESP32 version  ·  Binary protocol
    Adds a WebSocket transport (see WebBridge.*)
    ──────────────────────────────────────────────────────────────── */

#include <Arduino.h>
#include <atomic>
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

/* =====================  CONFIGURATION CONSTANTS  ================ */
namespace Config {
  // Timing
  constexpr unsigned long ENDSTOP_DEBOUNCE_US = 1000000UL;  // 1 second

  // Preset limits
  constexpr uint8_t MAX_PRESET_ID = 100;
  constexpr uint8_t MIN_PRESET_ID = 1;

  // Servo PWM range (microseconds)
  constexpr int DEFAULT_PWM_MIN = 600;
  constexpr int DEFAULT_PWM_MAX = 2400;
  constexpr int PWM_ABS_MIN     = 100;
  constexpr int PWM_ABS_MAX     = 3000;

  // Legacy compatibility
  constexpr size_t LEGACY_PRESET_SIZE = 86;  // Old preset format size

  // Protocol special IDs
  namespace ProtocolID {
    constexpr uint8_t NVS_SAVE_ACK      = 0xF0;
    constexpr uint8_t PRESET_SAVE_ACK   = 0xF1;
    constexpr uint8_t PRESET_LOAD_END   = 0xF2;
    constexpr uint8_t PRESET_COUNT      = 0xF3;
    constexpr uint8_t PRESET_DELETE_ACK = 0xF4;
    constexpr uint8_t PRESET_EXISTS     = 0xF5;
    constexpr uint8_t PRESET_NAME_CHUNK = 0xF6;
    constexpr uint8_t LIST_END          = 0xF8;
    constexpr uint8_t PRESET_READY      = 0xF9;
    constexpr uint8_t FIELD_ACK         = 0xFA;
    constexpr uint8_t READY_ACK         = 0xFF;
  }
}

/* =====================  LOGGING SYSTEM  ========================== */
enum LogLevel { LOG_ERROR = 0, LOG_WARN = 1, LOG_INFO = 2, LOG_DEBUG = 3 };

#ifndef CURRENT_LOG_LEVEL
  #define CURRENT_LOG_LEVEL LOG_INFO
#endif

#define LOG(level, fmt, ...) \
  do { \
    if ((level) <= CURRENT_LOG_LEVEL) { \
      const char* level_str = \
        (level) == LOG_ERROR ? "ERROR" : \
        (level) == LOG_WARN  ? "WARN " : \
        (level) == LOG_INFO  ? "INFO " : "DEBUG"; \
      Serial.printf("[%s] " fmt "\n", level_str, ##__VA_ARGS__); \
    } \
  } while(0)

/* —— debounce for ENDSTOP —— */
constexpr unsigned long DEBOUNCE_US = Config::ENDSTOP_DEBOUNCE_US;
std::atomic<unsigned long> lastEndstopMicros{0};
std::atomic<bool>          endstopEvent{false};

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
void broadcastAllParameters();
void broadcastBrowserFields(const Preset& preset);
uint8_t scanActivePresetCount();

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
      servoPwmMin = constrain((int)val, Config::PWM_ABS_MIN, Config::PWM_ABS_MAX);
      myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
      break;

    case ParamID_SERVO_PWM_MAX_US:
      servoPwmMax = constrain((int)val, Config::PWM_ABS_MIN, Config::PWM_ABS_MAX);
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

/* =====================  HELPER FUNCTIONS  ======================= */
void broadcastAllParameters()
{
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
}

void broadcastBrowserFields(const Preset& preset)
{
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
}

uint8_t scanActivePresetCount()
{
  uint8_t count = 0;
  for (uint8_t i = Config::MIN_PRESET_ID; i <= Config::MAX_PRESET_ID; i++) {
    if (prefs.isKey(String(i).c_str())) {
      count++;
    }
  }
  return count;
}

/* =====================  WS → MCU PARSER  (NEW) ================= */
void processIncomingFrame(uint8_t *buf, size_t len)
{
  if (len != 8) {
    LOG(LOG_ERROR, "Bad frame length: %d (expected 8)", len);
    return;
  }

  if (buf[0] != SYNC) {
    LOG(LOG_ERROR, "Bad sync byte: 0x%02X (expected 0xAA)", buf[0]);
    return;
  }

  uint8_t cmd = buf[1], id = buf[2];
  FloatBytes fb; memcpy(fb.b, &buf[3], 4);
  uint8_t rxCrc = buf[7];

  uint8_t calc = cmd ^ id;
  for (uint8_t b : fb.b) calc ^= b;

  if (calc != rxCrc) {
    LOG(LOG_ERROR, "CRC mismatch: got 0x%02X, expected 0x%02X (cmd=0x%02X, id=0x%02X)",
        rxCrc, calc, cmd, id);
    return;
  }

  LOG(LOG_DEBUG, "RX cmd=0x%02X id=0x%02X val=%.2f", cmd, id, fb.f);

  if (cmd == CMD_SAVE_NVS) {
      saveToNVS();
      sendFrame(CMD_ACK, Config::ProtocolID::NVS_SAVE_ACK, 1);
      return;
  }
  else if (cmd == CMD_LOAD_NVS) {
      loadFromNVS();
      for (uint8_t id = ParamID_RISE_TIME_MS; id <= ParamID_RUN_CONVEYOR; ++id)
          sendFrame(CMD_ACK, id, readParam(id));
      return;
  }
  else if (cmd == CMD_SAVE_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    savePresetToNVS(presetId);
    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_SAVE_ACK, presetId);
    return;
  }
  else if (cmd == CMD_LOAD_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    loadPresetFromNVS(presetId);
    broadcastAllParameters();
    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_LOAD_END, presetId);
    return;
  }
  else if (cmd == CMD_LIST_PRESETS) {
    if (!prefs.begin("presets", true)) {
      LOG(LOG_ERROR, "Failed to open NVS namespace 'presets' for listing");
      return;
    }

    uint8_t count = prefs.getUChar("count", 0);
    LOG(LOG_INFO, "Listing %d presets", count);

    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_COUNT, count);

    // Send each existing preset ID and name
    for (uint8_t i = Config::MIN_PRESET_ID; i <= Config::MAX_PRESET_ID; i++) {
      if (prefs.isKey(String(i).c_str())) {
        Preset preset = {};
        prefs.getBytes(String(i).c_str(), &preset, sizeof(Preset));
        LOG(LOG_DEBUG, "Found preset ID %d: '%s'", i, preset.name);
        sendFrame(CMD_ACK, Config::ProtocolID::PRESET_EXISTS, i);
        for (uint8_t c = 0; c < 8; c++) {
          FloatBytes fbName;
          memcpy(fbName.b, &preset.name[c*4], 4);
          sendFrame(CMD_ACK, Config::ProtocolID::PRESET_NAME_CHUNK, fbName.f);
        }
      }
    }

    LOG(LOG_DEBUG, "Preset list complete");
    sendFrame(CMD_ACK, Config::ProtocolID::LIST_END, 0);

    prefs.end();
    return;
  }
  else if (cmd == CMD_DELETE_PRESET) {
    uint8_t presetId = constrain((uint8_t)fb.f, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    deletePresetFromNVS(presetId);
    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_DELETE_ACK, presetId);
    return;
  }
  else if (cmd == CMD_SAVE_PRESET_WITH_DATA) {
    // This command will be followed by individual field updates
    uint8_t presetId = constrain((uint8_t)fb.f, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    
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

    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_READY, presetId);
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

    sendFrame(CMD_ACK, Config::ProtocolID::FIELD_ACK, fieldId);
    return;
  }
  else if (cmd == CMD_SAVE_PRESET_COMPLETE) {
    // Finalize the preset save
    uint8_t presetId = constrain((uint8_t)fb.f, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    savePresetToNVSWithData(presetId, currentPresetData);
    sendFrame(CMD_ACK, Config::ProtocolID::PRESET_SAVE_ACK, presetId);
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
  if (!prefs.begin("params", true)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'params' for reading");
    return;
  }

  LOG(LOG_INFO, "Loading parameters from NVS");

  auto get = [&](const char* key, float fallback){
    if (!prefs.isKey(key)) {
      LOG(LOG_WARN, "Key '%s' not found in NVS, using default %.2f", key, fallback);
      return fallback;
    }
    return prefs.getFloat(key);
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

  LOG(LOG_INFO, "Parameters loaded successfully");
}

/* =====================  Save Presets (NVS)  ===================== */
void saveToNVS()
{
  if (!prefs.begin("params", false)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'params' for writing");
    return;
  }

  LOG(LOG_INFO, "Saving parameters to NVS");

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

  LOG(LOG_INFO, "Parameters saved successfully");
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
  if (presetId < Config::MIN_PRESET_ID || presetId > Config::MAX_PRESET_ID) {
    LOG(LOG_ERROR, "Invalid preset ID: %d (must be %d-%d)",
        presetId, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    return;
  }

  if (!prefs.begin("presets", false)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'presets' for writing");
    return;
  }

  LOG(LOG_INFO, "Saving preset ID %d to NVS", presetId);
  
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

  // Check if preset already exists BEFORE writing (to track if it's new)
  bool presetExisted = prefs.isKey(String(presetId).c_str());

  // Save binary preset data directly using preset ID as key
  size_t written = prefs.putBytes(String(presetId).c_str(), &preset, sizeof(Preset));

  // Verify it was saved
  if (written != sizeof(Preset) || !prefs.isKey(String(presetId).c_str())) {
    LOG(LOG_ERROR, "Failed to save preset %d to NVS", presetId);
    prefs.end();
    return;
  }

  LOG(LOG_DEBUG, "Preset %d saved successfully (%d bytes)", presetId, sizeof(Preset));

  // Update count only if this is a NEW preset (didn't exist before)
  if (!presetExisted) {
    uint8_t currentCount = prefs.getUChar("count", 0);
    if (currentCount < Config::MAX_PRESET_ID) {
      prefs.putUChar("count", currentCount + 1);
      LOG(LOG_DEBUG, "Incremented preset count to %d", currentCount + 1);
    }
  }

  prefs.end();
}

void loadPresetFromNVS(uint8_t presetId) {
  if (presetId < Config::MIN_PRESET_ID || presetId > Config::MAX_PRESET_ID) {
    LOG(LOG_ERROR, "Invalid preset ID: %d (must be %d-%d)",
        presetId, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    return;
  }

  if (!prefs.begin("presets", true)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'presets' for reading");
    return;
  }

  LOG(LOG_INFO, "Loading preset ID %d from NVS", presetId);

  size_t dataSize = prefs.getBytesLength(String(presetId).c_str());

  if (dataSize == 0) {
    LOG(LOG_ERROR, "Preset %d not found in NVS", presetId);
    prefs.end();
    return;
  }

  if (dataSize != sizeof(Preset) && dataSize != Config::LEGACY_PRESET_SIZE) {
    LOG(LOG_ERROR, "Preset %d has invalid size: %d bytes (expected %d or %d)",
        presetId, dataSize, sizeof(Preset), Config::LEGACY_PRESET_SIZE);
    prefs.end();
    return;
  }

  Preset preset = {};
  size_t bytesRead = prefs.getBytes(String(presetId).c_str(), &preset, min(dataSize, sizeof(Preset)));
  prefs.end();

  if (bytesRead != dataSize) {
    LOG(LOG_ERROR, "Failed to read preset %d: got %d bytes, expected %d",
        presetId, bytesRead, dataSize);
    return;
  }

  if (bytesRead == dataSize) {
      // Apply MCU values
      riseTime = constrain(preset.riseTime, 1.0f, 10000.0f);
      fallTime = constrain(preset.fallTime, 1.0f, 10000.0f);
      stayHigh = constrain(preset.stayHigh, 0.0f, 10000.0f);
      stayLow = constrain(preset.stayLow, 0.0f, 10000.0f);
      minAngle = constrain(preset.minAngle, 0.0f, 180.0f);
      maxAngle = constrain(preset.maxAngle, 0.0f, 180.0f);
      speedStepsPerSec = constrain(preset.speedStepsPerSec, 1.0f, 10000.0f);
      servoPwmMin = constrain((int)preset.servoPwmMin, Config::PWM_ABS_MIN, Config::PWM_ABS_MAX);
      servoPwmMax = constrain((int)preset.servoPwmMax, Config::PWM_ABS_MIN, Config::PWM_ABS_MAX);
      yokeRunning = preset.runYoke != 0;
      conveyorRunning = preset.runConveyor != 0;
      
      // Update hardware
      myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
      stepper.setMaxSpeed(speedStepsPerSec);
      stepper.setSpeed(speedStepsPerSec);

      // Broadcast all parameters
      broadcastAllParameters();
      broadcastBrowserFields(preset);
      sendFrame(CMD_ACK, Config::ProtocolID::PRESET_LOAD_END, presetId);

      LOG(LOG_INFO, "Preset %d loaded successfully", presetId);
    }
}

uint8_t getPresetCount() {
  prefs.begin("presets", true);
  uint8_t count = prefs.getUChar("count", 0);
  prefs.end();
  return min(count, Config::MAX_PRESET_ID);
}

void deletePresetFromNVS(uint8_t presetId) {
  if (presetId < Config::MIN_PRESET_ID || presetId > Config::MAX_PRESET_ID) return;
  
  if (!prefs.begin("presets", false)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'presets' for deletion");
    return;
  }

  // Check if preset exists BEFORE deleting (to know if we should decrement count)
  bool presetExisted = prefs.isKey(String(presetId).c_str());

  if (!presetExisted) {
    LOG(LOG_WARN, "Preset %d does not exist, nothing to delete", presetId);
    prefs.end();
    return;
  }

  LOG(LOG_INFO, "Deleting preset ID %d", presetId);
  prefs.remove(String(presetId).c_str());

  // Decrement count since preset existed
  uint8_t currentCount = prefs.getUChar("count", 0);
  if (currentCount > 0) {
    prefs.putUChar("count", currentCount - 1);
    LOG(LOG_INFO, "Preset %d deleted, %d presets remaining", presetId, currentCount - 1);
  }

  prefs.end();
}

/* =====================  ISR  ================================== */
void IRAM_ATTR endstopISR()
{
  unsigned long now = micros();
  unsigned long last = lastEndstopMicros.load(std::memory_order_relaxed);

  if (now - last >= DEBOUNCE_US) {
    lastEndstopMicros.store(now, std::memory_order_relaxed);
    endstopEvent.store(true, std::memory_order_release);
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
  sendFrame(CMD_ACK, Config::ProtocolID::READY_ACK, 0.0f);
}

/* =====================  LOOP  ================================= */
void loop()
{
  processSerial();
  updateServo();
  if (conveyorRunning) stepper.runSpeed();

  if (endstopEvent.load(std::memory_order_acquire)) {
    endstopEvent.store(false, std::memory_order_relaxed);

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
  if (presetId < Config::MIN_PRESET_ID || presetId > Config::MAX_PRESET_ID) {
    LOG(LOG_ERROR, "Invalid preset ID: %d (must be %d-%d)",
        presetId, Config::MIN_PRESET_ID, Config::MAX_PRESET_ID);
    return;
  }

  if (!prefs.begin("presets", false)) {
    LOG(LOG_ERROR, "Failed to open NVS namespace 'presets' for writing");
    return;
  }

  LOG(LOG_INFO, "Saving full preset ID %d to NVS", presetId);

  // Check if preset already exists BEFORE writing
  bool presetExisted = prefs.isKey(String(presetId).c_str());

  // Save the complete preset data
  Preset tmp = presetData;
  if (tmp.name[0] == '\0')
    snprintf(tmp.name, sizeof(tmp.name), "Preset %u", presetId);

  size_t written = prefs.putBytes(String(presetId).c_str(), &tmp, sizeof(Preset));

  // Verify save
  if (written != sizeof(Preset)) {
    LOG(LOG_ERROR, "Failed to save full preset %d to NVS", presetId);
    prefs.end();
    return;
  }

  LOG(LOG_DEBUG, "Full preset %d saved successfully (%d bytes)", presetId, sizeof(Preset));

  // Update count only if this is a NEW preset
  if (!presetExisted) {
    uint8_t currentCount = prefs.getUChar("count", 0);
    if (currentCount < Config::MAX_PRESET_ID) {
      prefs.putUChar("count", currentCount + 1);
      LOG(LOG_DEBUG, "Incremented preset count to %d", currentCount + 1);
    }
  }

  prefs.end();
}
