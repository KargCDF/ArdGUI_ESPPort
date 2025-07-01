// #include <Arduino.h>
// #include <ESP32Servo.h>

// Servo myServo;
// const int servoPin    = 13;    // change to your servo pin
// const int stepSize    = 10;    // µs per step (try 20–50 for finer resolution)
// const int delayMs     = 1000;   // ms between steps

// void setup() {
//   Serial.begin(115200);
//   while(!Serial){}                // wait for Serial
//   myServo.attach(servoPin);       // use default range for now
//   Serial.println("=== Servo calibration sweep ===");
//   delay(1000);
// }
// // 2400 max, 540 min
// void loop() {
//   // Sweep from 500 → 2500 µs
// //   for (int pulse = 1500; pulse <= 2380; pulse += stepSize) {
// //     myServo.writeMicroseconds(pulse);
// //     Serial.print("Pulse = ");
// //     Serial.println(pulse);
// //     delay(delayMs);
// //   }
// //   Then back down 2500 → 500
//   for (int pulse = 700; pulse >= 500; pulse -= stepSize) {
//     myServo.writeMicroseconds(pulse);
//     Serial.print("Pulse = ");
//     Serial.println(pulse);
//     delay(delayMs);
//   }

//   // once it’s run once, you can stop here if you like:
//   while(true);
// }

#include <Arduino.h>

/*  Stepper-Servo controller – binary-protocol version  ------------------- */

#include <ESP32Servo.h>
#include <AccelStepper.h>

bool yokeInverted = false;          // ← set to true for your build

/* ----------------------- hardware pins --------------------------------- */
constexpr uint8_t PWM_PIN = 13;   // MG90S
constexpr uint8_t STEP_PIN = 25;  // TB6600 PUL-
constexpr uint8_t DIR_PIN = 26;   // TB6600 DIR-
constexpr uint8_t EN_PIN = 27;    // TB6600 EN-
constexpr uint8_t ENDSTOP_PIN = 33;               // INT1, NC switch to GND

/* ---------- debounce constants & state (top of file) ------------------ */
constexpr unsigned long DEBOUNCE_US = 1000000;        // 1 s
volatile unsigned long  lastEndstopMicros = 0;       // time of previous hit

/* ----------------------- run-time variables ---------------------------- */
float riseTime = 120.48734593882432, fallTime = 120.48734593882432;
float stayHigh = 159.02530812235136, stayLow = 715.0;
float minAngle = 23, maxAngle = 83.24367296941217;
float speedStepsPerSec = 39.78873577297383;
volatile bool yokeRunning     = false;           // now volatile – seen in ISR
volatile bool conveyorRunning = false;
volatile bool endstopEvent    = false;           // “service me” flag
int servoPwmMin = 600;
int servoPwmMax = 2400;

/* ----------------------- objects --------------------------------------- */
Servo myServo;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ---------- binary-protocol parameter IDs -------------------------------- */
enum ParamID : uint8_t {
  ParamID_RISE_TIME_MS = 0x01,
  ParamID_FALL_TIME_MS = 0x02,
  ParamID_HOLD_HIGH_MS = 0x03,
  ParamID_HOLD_LOW_MS = 0x04,
  ParamID_MIN_ANGLE_DEG = 0x05,
  ParamID_MAX_ANGLE_DEG = 0x06,
  ParamID_STEPS_PER_SEC = 0x07,
  /* new */
  ParamID_SERVO_PWM_MIN_US = 0x08,
  ParamID_SERVO_PWM_MAX_US = 0x09,
  /* run / stop */
  ParamID_RUN_YOKE = 0x10,
  ParamID_RUN_CONVEYOR = 0x11
};

/* ----------------------- helpers --------------------------------------- */
union FloatBytes {
  float f;
  uint8_t b[4];
};

struct Frame {
  uint8_t sync, cmd, id;
  FloatBytes data;
  uint8_t crc;
};

constexpr uint8_t SYNC = 0xAA;
constexpr uint8_t CMD_SET = 0x01;
constexpr uint8_t CMD_TOGGLE = 0x02;
constexpr uint8_t CMD_GET   = 0x03;   // host → MCU : ask for a value
constexpr uint8_t CMD_ACK   = 0x81;   // MCU → host : answer / acknowledge

inline void writeYoke(int logicalDeg)
{
  // If the mechanics are flipped, logical  0°..180°  becomes physical 180°..0°
  myServo.write(yokeInverted ? 180 - logicalDeg : logicalDeg);
}

float readParam(uint8_t id)
{
  switch (id) {
    case ParamID_RISE_TIME_MS:   return riseTime;
    case ParamID_FALL_TIME_MS:   return fallTime;
    case ParamID_HOLD_HIGH_MS:   return stayHigh;
    case ParamID_HOLD_LOW_MS:    return stayLow;
    case ParamID_MIN_ANGLE_DEG:  return minAngle;
    case ParamID_MAX_ANGLE_DEG:  return maxAngle;
    case ParamID_STEPS_PER_SEC:  return speedStepsPerSec;
    case ParamID_SERVO_PWM_MIN_US:return servoPwmMin;
    case ParamID_SERVO_PWM_MAX_US:return servoPwmMax;
    case ParamID_RUN_YOKE:       return yokeRunning;
    case ParamID_RUN_CONVEYOR:   return conveyorRunning;
    default:                     return 0.0f;
  }
}

/* ----------------------- forward decl. --------------------------------- */
void applyParam(uint8_t id, float val);
void updateServo();
void sendFrame(uint8_t cmd, uint8_t id, float val);
void IRAM_ATTR endstopISR();

/* ====================== SETUP ========================================== */
void setup() {
  Serial.begin(115200);

  myServo.attach(PWM_PIN, servoPwmMin, servoPwmMax);
  myServo.write(minAngle);

  stepper.setPinsInverted(true, true, false);
  stepper.setEnablePin(EN_PIN);
  stepper.setMinPulseWidth(20);
  stepper.enableOutputs();
  stepper.setMaxSpeed(speedStepsPerSec);
  stepper.setSpeed(speedStepsPerSec);
  sendFrame(CMD_ACK, 0xFF, 0.0f);   // 0xFF = “READY”
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);            // NC → GND, pull-up inside
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN),
                  endstopISR,
                  RISING);                       // LOW→HIGH when switch opens
}

/* ----------------------- INTERRUPT HANDLER ----------------------------- */
// void endstopISR()
void IRAM_ATTR endstopISR()
{
  unsigned long now = micros();                      // ~4 µs call
  if (now - lastEndstopMicros >= DEBOUNCE_US) {      // ignore bounce
    lastEndstopMicros = now;
    endstopEvent = true;                             // serviced in loop()
  }
}

/* ====================== PACKET PARSER ================================== */

enum RxState { WAIT_SYNC,
               READ_HDR,
               READ_DATA,
               READ_CRC };
RxState rxState = WAIT_SYNC;
Frame rxFrame;
uint8_t idx = 0;
uint8_t calcCrc = 0;

void processSerial() {
  while (Serial.available()) {
    uint8_t byteIn = Serial.read();

    switch (rxState) {
      case WAIT_SYNC:
        if (byteIn == SYNC) {
          rxState = READ_HDR;
          idx = 0;
          calcCrc = 0;
        }
        break;

      case READ_HDR:
          if (idx == 0) rxFrame.cmd = byteIn;
          else          rxFrame.id  = byteIn;
          calcCrc ^= byteIn;
          if (++idx == 2) { rxState = READ_DATA; idx = 0; }
          break;

      case READ_DATA:  // 4 data bytes
        rxFrame.data.b[idx++] = byteIn;
        calcCrc ^= byteIn;
        if (idx == 4) rxState = READ_CRC;
        break;

      case READ_CRC:
        if (calcCrc == byteIn) {                 // ← good packet
          if      (rxFrame.cmd == CMD_SET || rxFrame.cmd == CMD_TOGGLE) {
            applyParam(rxFrame.id, rxFrame.data.f);
            sendFrame(CMD_ACK, rxFrame.id, readParam(rxFrame.id));   // <—
          }
          else if (rxFrame.cmd == CMD_GET) {
            if (rxFrame.id == 0x00) {            // 0 → dump everything
              for (uint8_t id = ParamID_RISE_TIME_MS; id <= ParamID_RUN_CONVEYOR; ++id)
                sendFrame(CMD_ACK, id, readParam(id));
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

/* ====================== SERVO STATE MACHINE ============================ */

enum SweepState { S_IDLE, S_RISE, S_HIGH, S_FALL, S_LOW };
SweepState sState = S_IDLE;
unsigned long lastEvent = 0;
unsigned long stepDelay = 0;
int currAngle = 0;

void updateServo()
{
  if (!yokeRunning) { sState = S_IDLE; return; }

  const int dirUp   = yokeInverted ? -1 : +1;   // which way is “rise”?
  const int dirDown = -dirUp;                   // opposite for “fall”

  unsigned long now = millis();

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
        if ((!yokeInverted && currAngle >= maxAngle) ||
            ( yokeInverted && currAngle <= minAngle)) {
          currAngle = yokeInverted ? minAngle : maxAngle;
          writeYoke(currAngle);
          sState = S_HIGH; lastEvent = now;
        } else {
          writeYoke(currAngle);
        }
      }
      break;

    case S_HIGH:
      if (now - lastEvent >= stayHigh) {
        stepDelay = fallTime / max(1.0f, maxAngle - minAngle);
        sState = S_FALL; lastEvent = now;
      }
      break;

    case S_FALL:
      if (now - lastEvent >= stepDelay) {
        lastEvent = now;
        currAngle += dirDown;
        if ((!yokeInverted && currAngle <= minAngle) ||
            ( yokeInverted && currAngle >= maxAngle)) {
          currAngle = yokeInverted ? maxAngle : minAngle;
          writeYoke(currAngle);
          sState = S_LOW; lastEvent = now;
        } else {
          writeYoke(currAngle);
        }
      }
      break;

    case S_LOW:
      if (now - lastEvent >= stayLow) {
        stepDelay = riseTime / max(1.0f, maxAngle - minAngle);
        sState = S_RISE; lastEvent = now;
      }
      break;
  }
}

/* ====================== PARAM HANDLER ================================== */

 void applyParam(uint8_t id, float val) {
   switch (id) {
     case ParamID_RISE_TIME_MS: 
       riseTime = val; 
       break;
     case ParamID_FALL_TIME_MS: 
       fallTime = val; 
       break;
     case ParamID_HOLD_HIGH_MS: 
       stayHigh = val; 
       break;
     case ParamID_HOLD_LOW_MS: 
       stayLow = val; 
       break;

     case ParamID_MIN_ANGLE_DEG:
       minAngle = constrain(val, 0.0f, 180.0f);
       // preview bottom position immediately
       writeYoke(minAngle);
       break;

     case ParamID_MAX_ANGLE_DEG:
       maxAngle = constrain(val, 0.0f, 180.0f);
       // preview top position immediately
       writeYoke(maxAngle);
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
         sState = S_IDLE;
       } else {
         yokeRunning = false;
         myServo.detach();
       }
       break;

     case ParamID_RUN_CONVEYOR:
       conveyorRunning = (val != 0);
       if (conveyorRunning) stepper.enableOutputs();
       else               stepper.disableOutputs();
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

/* ====================== SEND FRAME ====================================== */

void sendFrame(uint8_t cmd, uint8_t id, float val)
{
  uint8_t crc = cmd ^ id;
  FloatBytes fb{ .f = val };
  for (uint8_t b : fb.b) crc ^= b;

  Serial.write(SYNC);
  Serial.write(cmd);
  Serial.write(id);
  Serial.write(fb.b, 4);
  Serial.write(crc);
}

/* ====================== MAIN LOOP ====================================== */
void loop() {
  processSerial();
  updateServo();
  if (conveyorRunning) stepper.runSpeed();

  /* --------- handle end-stop event with the *existing* logic --------- */
  if (endstopEvent) {
    noInterrupts();                // atomic clear of the flag
    endstopEvent = false;
    interrupts();

    /* toggle Yoke ------------------------------------------------------ */
    float newYokeState = yokeRunning ? 0.0f : 1.0f;
    applyParam(ParamID_RUN_YOKE, newYokeState);

    /* toggle Conveyor -------------------------------------------------- */
    float newConvState = conveyorRunning ? 0.0f : 1.0f;
    applyParam(ParamID_RUN_CONVEYOR, newConvState);

    /* (optional) tell the host what just happened --------------------- */
    sendFrame(CMD_ACK, ParamID_RUN_YOKE, newYokeState);
    sendFrame(CMD_ACK, ParamID_RUN_CONVEYOR, newConvState);
  }
}