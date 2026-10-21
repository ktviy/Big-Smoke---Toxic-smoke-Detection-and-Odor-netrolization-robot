/*
  =============================================================================
  Big Smoke - Toxic Smoke Detection and Odor Neutralization Robot
  =============================================================================

  A mobile air purifier on a mecanum-wheel base (Arduino Mega 2560).

  Sensing   MQ-2 (LPG, smoke), MQ-7 (CO), MQ-135 (air quality)   -> ADC
            SHT31 temperature / humidity                         -> I2C
            Plantower PMS (PM2.5, PM10)                          -> UART
            HC-SR04 ultrasonic on a sweeping servo + 2 IR        -> GPIO / interrupt
  Acting    4 DC motors (mecanum), mist maker, mist fan,
            filter fan, ionizer, buzzer, RGB status LED
  Links     Bluetooth (HC-05) and Wi-Fi (ESP) remote, USB console

  Architecture
  ------------
  loop() never blocks. Every subsystem is a small task that is polled on each
  pass and decides from millis()/micros() whether it has work to do:

      sonarTask      interrupt-timed echo, one ping every 60 ms
      gasTask        ADC sampling + smoothing every 100 ms
      shtTask        two-phase I2C read (start conversion, collect 16 ms later)
      pmsTask        drains the PMS UART byte by byte
      sensorTask     1 Hz: gas ppm, warm-up/calibration, purifier decisions
      inputTask      buttons + remote commands
      driveTask      drive-mode state machine (IDLE / REMOTE / AVOID / FOLLOW)
      outputTask     relays (staggered), buzzer pattern, status LED
      reportTask     non-blocking serial output + loop timing statistics

  Two independent state machines run side by side: the drive mode decides how
  the robot moves, the purifier mode (AUTO / MANUAL) decides what the
  actuators do. Gas alarms always sound the buzzer, in every mode.

  Remote protocol (single character, same on Bluetooth, Wi-Fi and USB)
  --------------------------------------------------------------------
      F forward      B back        L strafe left    R strafe right
      I fwd-right    J fwd-left    K back-right     M back-left
      Q rotate left  E rotate right                  S / T stop
      0 IDLE   1 REMOTE   2 AVOID   3 FOLLOW         (mode select)
      P purifier AUTO/MANUAL   N manual mist   O manual filter+ionizer

  Pins, thresholds and timings: see config.h
*/

#include <Wire.h>
#include <Servo.h>
#include <AFMotor.h>
#include <PMS.h>
#include "config.h"

// =============================================================================
//  Types
// =============================================================================

enum DriveMode : uint8_t { MODE_IDLE, MODE_REMOTE, MODE_AVOID, MODE_FOLLOW, MODE_COUNT };
const char* const DRIVE_MODE_NAMES[MODE_COUNT] = { "IDLE", "REMOTE", "AVOID", "FOLLOW" };

enum PurifierMode : uint8_t { PURIFIER_AUTO, PURIFIER_MANUAL };

// Sub-states of the obstacle-avoidance behaviour
enum AvoidState : uint8_t {
  AVOID_CRUISE,      // drive forward, watch the sonar
  AVOID_BACKUP,      // short reverse after an obstacle
  AVOID_LOOK_LEFT,   // servo turning left, then measure
  AVOID_LOOK_RIGHT,  // servo turning right, then measure
  AVOID_TURN,        // rotate toward the open side
  AVOID_PAUSE        // settle before cruising again
};

// A body velocity for the mecanum base. Each axis is -1, 0 or +1.
struct Motion {
  int8_t vy;  // +forward
  int8_t vx;  // +right (strafe)
  int8_t w;   // +clockwise (rotate right)
};

const Motion MOVE_STOP     = {  0,  0,  0 };
const Motion MOVE_FORWARD  = {  1,  0,  0 };
const Motion MOVE_BACK     = { -1,  0,  0 };
const Motion MOVE_LEFT     = {  0, -1,  0 };
const Motion MOVE_RIGHT    = {  0,  1,  0 };
const Motion MOVE_ROT_LEFT = {  0,  0, -1 };
const Motion MOVE_ROT_RIGHT= {  0,  0,  1 };
const Motion MOVE_FWD_LEFT = {  1, -1,  0 };
const Motion MOVE_FWD_RIGHT= {  1,  1,  0 };
const Motion MOVE_BK_LEFT  = { -1, -1,  0 };
const Motion MOVE_BK_RIGHT = { -1,  1,  0 };

struct Button {
  uint8_t  pin;
  bool     rawPressed;
  bool     stablePressed;
  uint32_t changedMs;
};

struct GasSensor {
  uint8_t pin;
  float   adcAvg;   // smoothed ADC reading
  float   r0;       // clean-air resistance (kOhm)
};

// Non-blocking line output: a message is built in RAM and trickled into the
// UART only as fast as its TX buffer has room, so printing never stalls loop().
class LineWriter {
 public:
  explicit LineWriter(HardwareSerial& port) : port_(port) {}
  bool busy() const { return pos_ < len_; }
  void start() { len_ = 0; pos_ = 0; buf_[0] = '\0'; }
  void add(const char* s) {
    while (*s && len_ < sizeof(buf_) - 1) buf_[len_++] = *s++;
    buf_[len_] = '\0';
  }
  void addInt(long v) { char t[12]; ltoa(v, t, 10); add(t); }
  void addFloat(float v, uint8_t decimals, const char* ifInvalid) {
    if (isnan(v) || isinf(v)) { add(ifInvalid); return; }
    char t[16]; dtostrf(v, 0, decimals, t); add(t);
  }
  void pump() {
    int room = port_.availableForWrite();
    while (room-- > 0 && pos_ < len_) port_.write(buf_[pos_++]);
  }
 private:
  HardwareSerial& port_;
  char     buf_[220];
  uint16_t len_ = 0;
  uint16_t pos_ = 0;
};

// =============================================================================
//  Hardware objects
// =============================================================================

AF_DCMotor motors[4] = {
  AF_DCMotor(1, MOTOR12_1KHZ), AF_DCMotor(2, MOTOR12_1KHZ),
  AF_DCMotor(3, MOTOR34_1KHZ), AF_DCMotor(4, MOTOR34_1KHZ)
};
Servo scanner;
PMS pms(PMS_SERIAL);
PMS::DATA pmsFrame;

LineWriter debugOut(DEBUG_SERIAL);
LineWriter wifiOut(WIFI_SERIAL);

Button btnMode     = { PIN_BTN_MODE,     false, false, 0 };
Button btnPurifier = { PIN_BTN_PURIFIER, false, false, 0 };
Button btnMist     = { PIN_BTN_MIST,     false, false, 0 };
Button btnFan      = { PIN_BTN_FAN,      false, false, 0 };

GasSensor mq2   = { PIN_MQ2,   0, MQ2_R0_KOHM };
GasSensor mq7   = { PIN_MQ7,   0, MQ7_R0_KOHM };
GasSensor mq135 = { PIN_MQ135, 0, MQ135_R0_KOHM };

// =============================================================================
//  State
// =============================================================================

// ---- Drive ----
DriveMode  driveMode      = MODE_IDLE;
Motion     currentMotion  = MOVE_STOP;
uint8_t    currentSpeed   = 255;  // != 0 so the first stopMotors() latches
AvoidState avoidState     = AVOID_CRUISE;
uint32_t   stateSinceMs   = 0;   // when the current AVOID sub-state started
uint32_t   holdUntilMs    = 0;   // FOLLOW: keep the current turn until this time
uint32_t   lastRemoteMs   = 0;
uint32_t   turnMs         = AVOID_TURN_MS;
bool       scanSettled    = false; // servo has arrived for the current look step
uint32_t   scanSettledSeq = 0;     // sonar sequence number when it arrived
uint16_t   scanLeftCm     = 0;
uint16_t   scanRightCm    = 0;

// ---- Ultrasonic (written by the ISR) ----
volatile uint32_t echoRiseUs  = 0;
volatile uint32_t echoWidthUs = 0;
volatile bool     echoReady   = false;

uint16_t sonarCm        = SONAR_MAX_CM;
uint32_t sonarSeq       = 0;      // increments on every completed measurement
uint32_t lastPingMs     = 0;
bool     pingPending    = false;
uint8_t  sonarMisses    = 0;

// ---- Environment ----
float    temperatureC   = NAN;
float    humidityPct    = NAN;
bool     shtMeasuring   = false;
uint32_t shtStartMs     = 0;
uint8_t  shtFailures    = 0;

uint16_t pm25 = 0, pm10 = 0;
uint32_t lastPmsFrameMs = 0;
bool     pmsSeen        = false;

float lpgPpm = NAN, smokePpm = NAN, coPpm = NAN, aqPpm = NAN;
bool  gasReady = false;           // warm-up done and R0 known
uint32_t lastGasSampleMs = 0;
bool     gasPrimed       = false; // first ADC sample taken
bool     printR0Pending  = false;

// ---- Purifier ----
PurifierMode purifierMode = PURIFIER_AUTO;
bool dustHigh = false, smokeHigh = false, aqHigh = false;
bool coAlarm = false, lpgAlarm = false, humidityLow = false;
uint32_t manualMistUntil = 0, manualFanUntil = 0;

const uint8_t RELAY_PINS[4] = { PIN_RELAY_MIST, PIN_RELAY_MIST_FAN, PIN_RELAY_FILTER, PIN_RELAY_IONIZER };
enum { RELAY_MIST, RELAY_MIST_FAN, RELAY_FILTER, RELAY_IONIZER };
bool relayWanted[4] = { false, false, false, false };
bool relayActual[4] = { false, false, false, false };
uint32_t lastRelaySwitchMs = 0;
bool buzzerWanted = false;

// ---- Scheduling / statistics ----
uint32_t lastSensorMs = 0, lastReportMs = 0, lastTelemetryMs = 0;
uint32_t loopMaxUs = 0, loopSumUs = 0, loopCount = 0;

// =============================================================================
//  Small helpers
// =============================================================================

// Wrap-safe "has this deadline passed?"
inline bool reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

// Hysteresis for "too high" readings: switch on above `on`, off below `off`.
bool aboveWithHysteresis(bool state, float value, float on, float off) {
  if (isnan(value)) return state;
  return state ? (value >= off) : (value > on);
}

// Hysteresis for "too low" readings: switch on below `on`, off above `off`.
bool belowWithHysteresis(bool state, float value, float on, float off) {
  if (isnan(value)) return state;
  return state ? (value <= off) : (value < on);
}

// =============================================================================
//  Drive: mecanum inverse kinematics
// =============================================================================

void setPort(uint8_t port, int16_t pwm) {
  AF_DCMotor& m = motors[port - 1];
  pwm *= MOTOR_POLARITY[port - 1];
  if (pwm == 0) {
    m.setSpeed(0);
    m.run(RELEASE);
  } else {
    m.setSpeed((uint8_t)abs(pwm));
    m.run(pwm > 0 ? FORWARD : BACKWARD);
  }
}

// Turns a body motion into four wheel speeds (X-configuration mecanum):
//   FL = vy + vx + w    FR = vy - vx - w
//   RL = vy - vx + w    RR = vy + vx - w
// then scales so the fastest wheel runs at `speed`.
void setMotion(const Motion& mv, uint8_t speed) {
  if (mv.vy == currentMotion.vy && mv.vx == currentMotion.vx &&
      mv.w == currentMotion.w && speed == currentSpeed) {
    return;  // nothing changed, don't re-latch the shift register
  }
  currentMotion = mv;
  currentSpeed  = speed;

  int16_t fl = mv.vy + mv.vx + mv.w;
  int16_t fr = mv.vy - mv.vx - mv.w;
  int16_t rl = mv.vy - mv.vx + mv.w;
  int16_t rr = mv.vy + mv.vx - mv.w;
  int16_t peak = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
  if (peak == 0 || speed == 0) {
    fl = fr = rl = rr = 0;
    peak = 1;
  }
  setPort(MOTOR_PORT_FL, fl * speed / peak);
  setPort(MOTOR_PORT_FR, fr * speed / peak);
  setPort(MOTOR_PORT_RL, rl * speed / peak);
  setPort(MOTOR_PORT_RR, rr * speed / peak);
}

inline void stopMotors() { setMotion(MOVE_STOP, 0); }

// =============================================================================
//  Ultrasonic: interrupt-timed echo, never waits
// =============================================================================

void onEchoChange() {
  uint32_t now = micros();
  if (digitalRead(PIN_ECHO) == HIGH) {
    echoRiseUs = now;
  } else {
    echoWidthUs = now - echoRiseUs;
    echoReady = true;
  }
}

void sonarTask(uint32_t now) {
  // 1. Collect a finished measurement as soon as the ISR reports it.
  if (echoReady) {
    noInterrupts();
    uint32_t width = echoWidthUs;
    echoReady = false;
    interrupts();
    uint32_t cm = width / 58;  // speed of sound, round trip
    sonarCm = (cm > SONAR_MAX_CM) ? SONAR_MAX_CM : (uint16_t)cm;
    sonarSeq++;
    sonarMisses = 0;
    pingPending = false;
  }

  // 2. Fire the next ping on schedule.
  if (now - lastPingMs >= SONAR_PERIOD_MS) {
    if (pingPending && sonarMisses < 255) sonarMisses++;  // previous ping never echoed
    lastPingMs = now;
    pingPending = true;
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);  // 10 us trigger pulse required by the HC-SR04
    digitalWrite(PIN_TRIG, LOW);
  }
}

inline bool sonarHealthy() { return sonarMisses < SONAR_MAX_MISSES; }

// =============================================================================
//  SHT31: non-blocking two-phase read
// =============================================================================

uint8_t sht31Crc(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

void shtFailed() {
  shtMeasuring = false;
  if (shtFailures < 255) shtFailures++;
  if (shtFailures >= 3) { temperatureC = NAN; humidityPct = NAN; }
}

void shtStartMeasurement(uint32_t now) {
  Wire.beginTransmission(SHT31_ADDR);
  Wire.write(0x24);  // single shot, high repeatability,
  Wire.write(0x00);  // no clock stretching
  if (Wire.endTransmission() == 0) {
    shtMeasuring = true;
    shtStartMs = now;
  } else {
    shtFailed();
  }
}

void shtTask(uint32_t now) {
  if (!shtMeasuring || now - shtStartMs < SHT31_MEASURE_MS) return;
  shtMeasuring = false;
  uint8_t d[6];
  if (Wire.requestFrom((uint8_t)SHT31_ADDR, (uint8_t)6) != 6) { shtFailed(); return; }
  for (uint8_t i = 0; i < 6; i++) d[i] = Wire.read();
  if (sht31Crc(d, 2) != d[2] || sht31Crc(d + 3, 2) != d[5]) { shtFailed(); return; }
  uint16_t rawT = ((uint16_t)d[0] << 8) | d[1];
  uint16_t rawH = ((uint16_t)d[3] << 8) | d[4];
  temperatureC = -45.0f + 175.0f * rawT / 65535.0f;
  humidityPct  = 100.0f * rawH / 65535.0f;
  shtFailures = 0;
}

// =============================================================================
//  PMS dust sensor
// =============================================================================

void pmsTask(uint32_t now) {
  // read() consumes one byte per call, so drain whatever has arrived.
  while (PMS_SERIAL.available()) {
    if (pms.read(pmsFrame)) {
      pm25 = pmsFrame.PM_AE_UG_2_5;
      pm10 = pmsFrame.PM_AE_UG_10_0;
      lastPmsFrameMs = now;
      pmsSeen = true;
    }
  }
}

inline bool pmsFresh(uint32_t now) { return pmsSeen && now - lastPmsFrameMs < PMS_STALE_MS; }

// =============================================================================
//  MQ gas sensors
// =============================================================================

// Sensor resistance from the divider formed with the load resistor.
float sensorResistance(float adc) {
  if (adc < 1.0f) adc = 1.0f;
  return MQ_RL_KOHM * (1023.0f - adc) / adc;
}

// MQ-135 temperature/humidity correction (from G. Krocker's MQ135 library):
// the sensor's resistance drifts with T and RH, so divide it out before using
// the calibration curve. Returns 1.0 at 20 C / 33 % RH.
float mq135Correction(float t, float h) {
  if (isnan(t) || isnan(h)) return 1.0f;
  if (t < 20.0f) return 0.00035f * t * t - 0.02718f * t + 1.39538f - (h - 33.0f) * 0.0018f;
  return -0.003333333f * t - 0.001923077f * h + 1.130128205f;
}

float mq135CorrectedResistance() {
  return sensorResistance(mq135.adcAvg) / mq135Correction(temperatureC, humidityPct);
}

inline float curvePpm(float rs, float r0, float a, float b) { return a * pow(rs / r0, b); }

void gasSampleTask(uint32_t now) {
  if (now - lastGasSampleMs < MQ_SAMPLE_MS) return;
  lastGasSampleMs = now;
  GasSensor* all[3] = { &mq2, &mq7, &mq135 };
  for (uint8_t i = 0; i < 3; i++) {
    float raw = analogRead(all[i]->pin);
    all[i]->adcAvg = gasPrimed ? all[i]->adcAvg + MQ_EMA_ALPHA * (raw - all[i]->adcAvg) : raw;
  }
  gasPrimed = true;
}

// Called once when warm-up ends: anything not set in config.h is calibrated
// from the current (assumed clean) air.
void calibrateGasSensors() {
  if (mq2.r0 <= 0)   mq2.r0 = sensorResistance(mq2.adcAvg) / MQ2_CLEAN_AIR_RATIO;
  if (mq7.r0 <= 0)   mq7.r0 = sensorResistance(mq7.adcAvg) / MQ7_CLEAN_AIR_RATIO;
  if (mq135.r0 <= 0) {
    // Rs/R0 in clean air follows from the curve at atmospheric CO2
    float cleanRatio = pow(MQ135_ATMO_CO2_PPM / MQ135_A, 1.0f / MQ135_B);
    mq135.r0 = mq135CorrectedResistance() / cleanRatio;
  }
  gasReady = true;
  printR0Pending = true;  // the report task prints it when the port is free
}

void updateGasPpm() {
  if (!gasReady) return;
  float rs2 = sensorResistance(mq2.adcAvg);
  lpgPpm   = curvePpm(rs2, mq2.r0, MQ2_LPG_A, MQ2_LPG_B);
  smokePpm = curvePpm(rs2, mq2.r0, MQ2_SMOKE_A, MQ2_SMOKE_B);
  coPpm    = curvePpm(sensorResistance(mq7.adcAvg), mq7.r0, MQ7_CO_A, MQ7_CO_B);
  aqPpm    = curvePpm(mq135CorrectedResistance(), mq135.r0, MQ135_A, MQ135_B);
}

// =============================================================================
//  Purifier: decide which actuators should be on
// =============================================================================

void updatePurifier(uint32_t now) {
  // Dust and humidity sensors need no warm-up.
  float dust = pmsFresh(now) ? (float)max(pm25, pm10) : NAN;
  dustHigh    = aboveWithHysteresis(dustHigh, dust, DUST_ON_UGM3, DUST_OFF_UGM3);
  humidityLow = belowWithHysteresis(humidityLow, humidityPct, HUMIDITY_LOW_ON_PCT, HUMIDITY_LOW_OFF_PCT);

  if (gasReady) {
    coAlarm   = aboveWithHysteresis(coAlarm,   coPpm,    CO_ON_PPM,    CO_OFF_PPM);
    lpgAlarm  = aboveWithHysteresis(lpgAlarm,  lpgPpm,   LPG_ON_PPM,   LPG_OFF_PPM);
    smokeHigh = aboveWithHysteresis(smokeHigh, smokePpm, SMOKE_ON_PPM, SMOKE_OFF_PPM);
    aqHigh    = aboveWithHysteresis(aqHigh,    aqPpm,    AQ_ON_PPM,    AQ_OFF_PPM);
  }

  bool gasAlarm = coAlarm || lpgAlarm;
  buzzerWanted = gasAlarm;  // safety: the alarm sounds in every mode

  if (purifierMode == PURIFIER_AUTO) {
    bool mist = gasAlarm || humidityLow;
    relayWanted[RELAY_MIST]     = mist;
    relayWanted[RELAY_MIST_FAN] = mist;
    relayWanted[RELAY_FILTER]   = gasAlarm || dustHigh || smokeHigh || aqHigh;
    relayWanted[RELAY_IONIZER]  = dustHigh || smokeHigh || aqHigh;
  } else {
    bool mist = !reached(now, manualMistUntil);
    bool fan  = !reached(now, manualFanUntil);
    relayWanted[RELAY_MIST]     = mist;
    relayWanted[RELAY_MIST_FAN] = mist;
    relayWanted[RELAY_FILTER]   = fan;
    relayWanted[RELAY_IONIZER]  = fan;
  }
}

// 1 Hz: kick off an SHT31 conversion, refresh ppm values, run the purifier.
void sensorTask(uint32_t now) {
  if (now - lastSensorMs < SENSOR_PERIOD_MS) return;
  lastSensorMs = now;

  if (!shtMeasuring) shtStartMeasurement(now);
  if (!gasReady && now >= MQ_WARMUP_MS && gasPrimed) calibrateGasSensors();
  updateGasPpm();
  updatePurifier(now);
}

// =============================================================================
//  Drive-mode state machine
// =============================================================================

void enterDriveMode(DriveMode mode, uint32_t now) {
  driveMode = mode;
  stopMotors();
  scanner.write(SERVO_CENTER_DEG);
  avoidState = AVOID_CRUISE;
  stateSinceMs = now;
  holdUntilMs = now;
  lastRemoteMs = now;
}

void setAvoidState(AvoidState s, uint32_t now) {
  avoidState = s;
  stateSinceMs = now;
  scanSettled = false;
}

// True once the servo has arrived AND a ping that started after that has
// completed (the ping in flight on arrival may have been aimed mid-sweep).
bool scanReadingReady(uint32_t now) {
  if (!scanSettled) {
    if (now - stateSinceMs < SERVO_SETTLE_MS) return false;
    scanSettled = true;
    scanSettledSeq = sonarSeq;
  }
  return sonarSeq >= scanSettledSeq + 2;
}

void runAvoid(uint32_t now) {
  switch (avoidState) {
    case AVOID_CRUISE:
      if (sonarCm <= AVOID_DIST_CM) {
        setMotion(MOVE_BACK, AUTO_SPEED);
        setAvoidState(AVOID_BACKUP, now);
      } else {
        setMotion(MOVE_FORWARD, AUTO_SPEED);
      }
      break;

    case AVOID_BACKUP:
      if (now - stateSinceMs >= AVOID_BACKUP_MS) {
        stopMotors();
        scanner.write(SERVO_LEFT_DEG);
        setAvoidState(AVOID_LOOK_LEFT, now);
      }
      break;

    case AVOID_LOOK_LEFT:
      if (scanReadingReady(now)) {
        scanLeftCm = sonarCm;
        scanner.write(SERVO_RIGHT_DEG);
        setAvoidState(AVOID_LOOK_RIGHT, now);
      }
      break;

    case AVOID_LOOK_RIGHT:
      if (scanReadingReady(now)) {
        scanRightCm = sonarCm;
        scanner.write(SERVO_CENTER_DEG);
        bool leftBlocked  = scanLeftCm  <= AVOID_DIST_CM;
        bool rightBlocked = scanRightCm <= AVOID_DIST_CM;
        // Turn toward the side with more room; both blocked -> turn around.
        setMotion(scanLeftCm > scanRightCm ? MOVE_ROT_LEFT : MOVE_ROT_RIGHT, AUTO_SPEED);
        turnMs = (leftBlocked && rightBlocked) ? AVOID_TURNAROUND_MS : AVOID_TURN_MS;
        setAvoidState(AVOID_TURN, now);
      }
      break;

    case AVOID_TURN:
      if (now - stateSinceMs >= turnMs) {
        stopMotors();
        setAvoidState(AVOID_PAUSE, now);
      }
      break;

    case AVOID_PAUSE:
      // also gives the servo time to return to center before we trust the sonar
      if (now - stateSinceMs >= AVOID_PAUSE_MS) setAvoidState(AVOID_CRUISE, now);
      break;
  }
}

void runFollow(uint32_t now) {
  if (!reached(now, holdUntilMs)) return;  // finish the current turn pulse
  bool rightIr = digitalRead(PIN_IR_RIGHT);
  bool leftIr  = digitalRead(PIN_IR_LEFT);

  if (sonarCm > FOLLOW_MIN_CM && sonarCm < FOLLOW_MAX_CM) {
    setMotion(MOVE_FORWARD, AUTO_SPEED);
  } else if (!rightIr && leftIr) {
    setMotion(MOVE_ROT_LEFT, AUTO_SPEED);
    holdUntilMs = now + FOLLOW_TURN_MS;
  } else if (rightIr && !leftIr) {
    setMotion(MOVE_ROT_RIGHT, AUTO_SPEED);
    holdUntilMs = now + FOLLOW_TURN_MS;
  } else if (sonarCm >= FOLLOW_MAX_CM) {
    stopMotors();
  }
}

void driveTask(uint32_t now) {
  switch (driveMode) {
    case MODE_IDLE:
      stopMotors();
      break;

    case MODE_REMOTE:
      if (REMOTE_TIMEOUT_MS > 0 && now - lastRemoteMs > REMOTE_TIMEOUT_MS) stopMotors();
      break;

    case MODE_AVOID:
    case MODE_FOLLOW:
      if (!sonarHealthy()) {  // fail safe: never drive blind
        stopMotors();
        break;
      }
      if (driveMode == MODE_AVOID) runAvoid(now);
      else runFollow(now);
      break;

    default:
      break;
  }
}

// =============================================================================
//  Inputs: buttons and remote commands
// =============================================================================

// Debounced press detection; true once per press.
bool buttonPressed(Button& b, uint32_t now) {
  bool raw = digitalRead(b.pin) == LOW;
  if (raw != b.rawPressed) {
    b.rawPressed = raw;
    b.changedMs = now;
  }
  if (now - b.changedMs >= DEBOUNCE_MS && raw != b.stablePressed) {
    b.stablePressed = raw;
    return raw;
  }
  return false;
}

void togglePurifierMode(uint32_t now) {
  purifierMode = (purifierMode == PURIFIER_AUTO) ? PURIFIER_MANUAL : PURIFIER_AUTO;
  manualMistUntil = manualFanUntil = now;
  updatePurifier(now);
}

// Press = run for MANUAL_RUN_MS, press while running = stop. Switches to MANUAL.
void manualToggle(uint32_t& until, uint32_t now) {
  if (purifierMode != PURIFIER_MANUAL) togglePurifierMode(now);
  until = reached(now, until) ? now + MANUAL_RUN_MS : now;
  updatePurifier(now);
}

void handleCommand(char c, uint32_t now) {
  switch (c) {
    case '0': enterDriveMode(MODE_IDLE, now);   return;
    case '1': enterDriveMode(MODE_REMOTE, now); return;
    case '2': enterDriveMode(MODE_AVOID, now);  return;
    case '3': enterDriveMode(MODE_FOLLOW, now); return;
    case 'P': togglePurifierMode(now);          return;
    case 'N': manualToggle(manualMistUntil, now); return;
    case 'O': manualToggle(manualFanUntil, now);  return;
    default: break;
  }

  const Motion* mv = nullptr;
  switch (c) {
    case 'F': mv = &MOVE_FORWARD;   break;
    case 'B': mv = &MOVE_BACK;      break;
    case 'L': mv = &MOVE_LEFT;      break;
    case 'R': mv = &MOVE_RIGHT;     break;
    case 'I': mv = &MOVE_FWD_RIGHT; break;
    case 'J': mv = &MOVE_FWD_LEFT;  break;
    case 'K': mv = &MOVE_BK_RIGHT;  break;
    case 'M': mv = &MOVE_BK_LEFT;   break;
    case 'Q': mv = &MOVE_ROT_LEFT;  break;
    case 'E': mv = &MOVE_ROT_RIGHT; break;
    case 'S':
    case 'T': mv = &MOVE_STOP;      break;
    default:  return;  // ignore line endings and unknown bytes
  }

  // A drive command always takes over manual control, from any mode.
  if (driveMode != MODE_REMOTE) enterDriveMode(MODE_REMOTE, now);
  lastRemoteMs = now;
  setMotion(*mv, (mv == &MOVE_STOP) ? 0 : DRIVE_SPEED);
}

void readCommands(Stream& link, uint32_t now) {
  while (link.available()) handleCommand((char)link.read(), now);
}

void inputTask(uint32_t now) {
  if (buttonPressed(btnMode, now))     enterDriveMode((DriveMode)((driveMode + 1) % MODE_COUNT), now);
  if (buttonPressed(btnPurifier, now)) togglePurifierMode(now);
  if (buttonPressed(btnMist, now))     manualToggle(manualMistUntil, now);
  if (buttonPressed(btnFan, now))      manualToggle(manualFanUntil, now);

  readCommands(BT_SERIAL, now);
  readCommands(WIFI_SERIAL, now);
  readCommands(DEBUG_SERIAL, now);
}

// =============================================================================
//  Outputs: relays, buzzer, status LED
// =============================================================================

void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r);
  digitalWrite(PIN_LED_G, g);
  digitalWrite(PIN_LED_B, b);
}

void outputTask(uint32_t now) {
  // Relays: switch at most one per RELAY_STAGGER_MS so motors/pumps don't all
  // start at once. Mist always switches before its fan (array order).
  if (now - lastRelaySwitchMs >= RELAY_STAGGER_MS) {
    for (uint8_t i = 0; i < 4; i++) {
      if (relayWanted[i] != relayActual[i]) {
        relayActual[i] = relayWanted[i];
        digitalWrite(RELAY_PINS[i], relayActual[i] ? RELAY_ON : !RELAY_ON);
        lastRelaySwitchMs = now;
        break;
      }
    }
  }

  bool blinkFast = (now / 250) % 2;
  bool blinkSlow = (now / 500) % 2;
  digitalWrite(PIN_BUZZER, buzzerWanted && blinkFast);

  if (coAlarm || lpgAlarm)                       setLed(blinkFast, false, false);  // red, flashing
  else if (!gasReady)                            setLed(false, false, blinkSlow);  // blue: warming up
  else if (dustHigh || smokeHigh || aqHigh)      setLed(true, true, false);        // yellow: purifying
  else                                           setLed(false, true, false);       // green: clean air
}

// =============================================================================
//  Reporting
// =============================================================================

void reportTask(uint32_t now) {
  if (printR0Pending && !debugOut.busy()) {
    printR0Pending = false;
    debugOut.start();
    debugOut.add("# Gas sensors calibrated. R0 (kOhm): MQ2=");
    debugOut.addFloat(mq2.r0, 2, "?");
    debugOut.add(" MQ7=");
    debugOut.addFloat(mq7.r0, 2, "?");
    debugOut.add(" MQ135=");
    debugOut.addFloat(mq135.r0, 2, "?");
    debugOut.add("  (copy into config.h to skip calibration)\r\n");
  }
  if (!debugOut.busy() && now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    LineWriter& o = debugOut;
    o.start();
    o.add(DRIVE_MODE_NAMES[driveMode]);
    o.add(purifierMode == PURIFIER_AUTO ? " AUTO" : " MANUAL");
    o.add(" | T="); o.addFloat(temperatureC, 1, "--");
    o.add("C RH="); o.addFloat(humidityPct, 0, "--");
    o.add("% PM2.5="); if (pmsFresh(now)) o.addInt(pm25); else o.add("--");
    o.add(" PM10=");   if (pmsFresh(now)) o.addInt(pm10); else o.add("--");
    if (gasReady) {
      o.add(" | CO="); o.addFloat(coPpm, 1, "--");
      o.add(" LPG=");  o.addFloat(lpgPpm, 0, "--");
      o.add(" smoke="); o.addFloat(smokePpm, 0, "--");
      o.add(" AQ=");   o.addFloat(aqPpm, 0, "--");
    } else {
      o.add(" | gas warming up ");
      o.addInt((long)((MQ_WARMUP_MS - min(now, MQ_WARMUP_MS)) / 1000));
      o.add("s");
    }
    o.add(" | sonar=");
    if (sonarHealthy()) { o.addInt(sonarCm); o.add("cm"); } else o.add("FAULT");
    o.add(" relays=");
    for (uint8_t i = 0; i < 4; i++) o.add(relayActual[i] ? "1" : "0");
    o.add(" | loop max="); o.addInt((long)loopMaxUs);
    o.add("us avg=");      o.addInt(loopCount ? (long)(loopSumUs / loopCount) : 0L);
    o.add("us\r\n");
    loopMaxUs = loopSumUs = loopCount = 0;
  }
  debugOut.pump();

  // Compact JSON line for the ESP / web dashboard
  if (!wifiOut.busy() && now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    LineWriter& o = wifiOut;
    o.start();
    o.add("{\"t\":");     o.addFloat(temperatureC, 1, "null");
    o.add(",\"h\":");     o.addFloat(humidityPct, 1, "null");
    o.add(",\"pm25\":");  if (pmsFresh(now)) o.addInt(pm25); else o.add("null");
    o.add(",\"pm10\":");  if (pmsFresh(now)) o.addInt(pm10); else o.add("null");
    o.add(",\"co\":");    o.addFloat(coPpm, 1, "null");
    o.add(",\"lpg\":");   o.addFloat(lpgPpm, 0, "null");
    o.add(",\"smoke\":"); o.addFloat(smokePpm, 0, "null");
    o.add(",\"aq\":");    o.addFloat(aqPpm, 0, "null");
    o.add(",\"alarm\":"); o.add((coAlarm || lpgAlarm) ? "true" : "false");
    o.add(",\"mode\":\""); o.add(DRIVE_MODE_NAMES[driveMode]);
    o.add("\",\"purifier\":\""); o.add(purifierMode == PURIFIER_AUTO ? "AUTO" : "MANUAL");
    o.add("\"}\n");
  }
  wifiOut.pump();
}

// =============================================================================
//  Setup and main loop
// =============================================================================

void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  BT_SERIAL.begin(BT_BAUD);
  WIFI_SERIAL.begin(WIFI_BAUD);
  PMS_SERIAL.begin(PMS_BAUD);

  Wire.begin();
  Wire.setWireTimeout(3000, true);  // a stuck I2C bus must not hang the robot

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], !RELAY_ON);
  }
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_PURIFIER, INPUT_PULLUP);
  pinMode(PIN_BTN_MIST, INPUT_PULLUP);
  pinMode(PIN_BTN_FAN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), onEchoChange, CHANGE);

  // Boot self-test: sweep the scanner once so a dead servo is obvious.
  // This is the only place delay() is used; the control loop has not started.
  scanner.attach(PIN_SERVO);
  const uint8_t sweep[] = { SERVO_LEFT_DEG, SERVO_RIGHT_DEG, SERVO_CENTER_DEG };
  for (uint8_t i = 0; i < sizeof(sweep); i++) {
    scanner.write(sweep[i]);
    delay(400);
  }

  stopMotors();

  DEBUG_SERIAL.println(F("# Big Smoke ready. Commands: F B L R I J K M Q E S | 0-3 mode | P N O purifier"));
}

void loop() {
  uint32_t startUs = micros();
  uint32_t now = millis();

  sonarTask(now);
  gasSampleTask(now);
  shtTask(now);
  pmsTask(now);
  sensorTask(now);
  inputTask(now);
  driveTask(now);
  outputTask(now);
  reportTask(now);

  uint32_t elapsed = micros() - startUs;
  if (elapsed > loopMaxUs) loopMaxUs = elapsed;
  loopSumUs += elapsed;
  loopCount++;
}
