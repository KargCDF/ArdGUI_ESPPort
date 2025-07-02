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

union FloatBytes { float f; uint8_t b[4]; };

struct Frame {
  uint8_t   sync, cmd, id;
  FloatBytes data;
  uint8_t   crc;
};

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
  if (len != 8 || buf[0] != SYNC) return;         // size / preamble check

  uint8_t cmd = buf[1], id = buf[2];
  FloatBytes fb; memcpy(fb.b, &buf[3], 4);
  uint8_t rxCrc = buf[7];

  uint8_t calc = cmd ^ id;
  for (uint8_t b : fb.b) calc ^= b;
  if (calc != rxCrc) return;                      // CRC fail

  if (cmd == CMD_SAVE_NVS) {
      saveToNVS();
      sendFrame(CMD_ACK, 0xF0, 1);            // 0xF0 = “SAVED”
      return;
  }
  else if (cmd == CMD_LOAD_NVS) {
      loadFromNVS();
      for (uint8_t id = ParamID_RISE_TIME_MS; id <= ParamID_RUN_CONVEYOR; ++id)
          sendFrame(CMD_ACK, id, readParam(id));   // dump back to host
      return;
  }

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

/* =====================  SETUP  ================================= */
void setup()
{
  Serial.begin(115200);

  prefs.begin("params", /*rw=*/false);   // open read-only first
  loadFromNVS();                         // <— new helper (below)
  prefs.end();

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

  /* ------ ENDSTOP event (debounced in ISR) ------ */
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

  WebBridge_loop();           // ★ house-keeping for AsyncWebServer
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
