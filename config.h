// =============================================================================
//  Big Smoke - hardware configuration
//
//  Board  : Arduino Mega 2560
//  Shield : Adafruit Motor Shield v1 (L293D + 74HC595, "AFMotor" library)
//  Drive  : 4 x DC gear motors on mecanum wheels
//
//  Everything you may need to change on the bench lives in this file:
//  pins, polarities, thresholds and timings. The logic is in BigSmoke.ino.
// =============================================================================
#pragma once
#include <Arduino.h>

#if !defined(__AVR_ATmega2560__)
#error "Big Smoke targets the Arduino Mega 2560 (Tools > Board > Arduino Mega or Mega 2560)."
#endif

// -----------------------------------------------------------------------------
//  Pins reserved by the motor shield - do NOT reuse them for anything else
//    D3, D5, D6, D11      L293D PWM (motor speed)
//    D4, D7, D8, D12      74HC595 shift register (motor direction)
//    D9, D10              servo headers (we use D10)
// -----------------------------------------------------------------------------

// ---- Serial links: hardware UARTs only, no SoftwareSerial --------------------
#define DEBUG_SERIAL  Serial    // USB, Serial Monitor (also accepts drive commands)
#define BT_SERIAL     Serial1   // HC-05 Bluetooth     RX1 = D19, TX1 = D18
#define WIFI_SERIAL   Serial2   // ESP (NodeMCU/ESP32) RX2 = D17, TX2 = D16
#define PMS_SERIAL    Serial3   // Plantower PMS       RX3 = D15, TX3 = D14

#define DEBUG_BAUD    115200
#define BT_BAUD       9600      // HC-05 factory default
#define WIFI_BAUD     9600
#define PMS_BAUD      9600      // fixed by the PMS sensor

// ---- I2C: SHT31 temperature / humidity on SDA = D20, SCL = D21 ---------------
#define SHT31_ADDR    0x44      // 0x45 if the ADDR pin is pulled high

// ---- Analog gas sensors -------------------------------------------------------
#define PIN_MQ2       A0        // LPG / smoke
#define PIN_MQ135     A1        // air quality (CO2-equivalent), T/H compensated
#define PIN_MQ7       A2        // carbon monoxide

// ---- Navigation ---------------------------------------------------------------
#define PIN_SERVO     10        // shield header "SER1"
#define PIN_ECHO      2         // HC-SR04 echo: must be an external-interrupt pin
#define PIN_TRIG      22        // HC-SR04 trigger
#define PIN_IR_LEFT   23
#define PIN_IR_RIGHT  25

// ---- Actuators ----------------------------------------------------------------
#define PIN_RELAY_MIST      26  // ultrasonic mist maker
#define PIN_RELAY_MIST_FAN  27  // fan that blows the mist out
#define PIN_RELAY_FILTER    28  // suction fan through HEPA + activated carbon
#define PIN_RELAY_IONIZER   29  // negative ion generator
#define RELAY_ON            HIGH   // change to LOW for active-low relay boards
#define PIN_BUZZER          30     // active buzzer

// ---- Push buttons (wired to GND, internal pull-ups) ---------------------------
#define PIN_BTN_MODE        32  // cycle drive mode: IDLE > REMOTE > AVOID > FOLLOW
#define PIN_BTN_PURIFIER    33  // purifier AUTO <-> MANUAL
#define PIN_BTN_MIST        34  // MANUAL: run mist for MANUAL_RUN_MS (press again = stop)
#define PIN_BTN_FAN         35  // MANUAL: run filter fan + ionizer for MANUAL_RUN_MS

// ---- Status LED (RGB, on/off per channel) ---------------------------------------
#define PIN_LED_R           36
#define PIN_LED_G           37
#define PIN_LED_B           38

// =============================================================================
//  Drive train
// =============================================================================
// Which motor-shield port drives which wheel. Derived from the original drive
// functions: the rotate and strafe patterns only work if M1/M4 share the left
// side and M1/M3 form a diagonal pair.
#define MOTOR_PORT_FL  1   // front left
#define MOTOR_PORT_FR  2   // front right
#define MOTOR_PORT_RR  3   // rear right
#define MOTOR_PORT_RL  4   // rear left

// +1 / -1 per shield port (index 0 = M1). Flip a sign if that wheel spins the
// wrong way when you send 'F'. These match the original forward() function.
const int8_t MOTOR_POLARITY[4] = { +1, -1, +1, -1 };

#define DRIVE_SPEED          255   // PWM 0-255 for remote driving
#define AUTO_SPEED           255   // PWM 0-255 for AVOID / FOLLOW

// Remote safety stop: halt if no command arrives for this long while moving.
// 0 = disabled (use it if your app sends a command only once per button press).
#define REMOTE_TIMEOUT_MS    0

// =============================================================================
//  Ultrasonic + servo scanner
// =============================================================================
#define SONAR_PERIOD_MS      60     // HC-SR04 needs >= 60 ms between pings
#define SONAR_MAX_CM         400    // anything further (or no obstacle) reads as this
#define SONAR_MAX_MISSES     3      // pings without any echo before we call it a fault

#define SERVO_CENTER_DEG     90
#define SERVO_LEFT_DEG       150    // the original code's "rightsee" angle points left
#define SERVO_RIGHT_DEG      20     // and its "leftsee" angle points right
#define SERVO_SETTLE_MS      350    // time for the servo to arrive before we trust a ping

// AVOID mode
#define AVOID_DIST_CM        20     // obstacle closer than this -> back off and scan
#define AVOID_BACKUP_MS      100
#define AVOID_TURN_MS        500
#define AVOID_TURNAROUND_MS  1000   // both sides blocked
#define AVOID_PAUSE_MS       200

// FOLLOW mode
#define FOLLOW_MIN_CM        1
#define FOLLOW_MAX_CM        15     // object between these -> drive toward it
#define FOLLOW_TURN_MS       150    // minimum turn pulse when an IR sensor fires

// =============================================================================
//  Gas sensors (MQ series)
// =============================================================================
// Heaters need time before readings mean anything. During warm-up the purifier
// ignores the MQ sensors, then each sensor's clean-air resistance R0 is
// calibrated from the averaged reading, so power the robot up in clean air.
#define MQ_WARMUP_MS         60000UL
#define MQ_SAMPLE_MS         100    // ADC sampling period
#define MQ_EMA_ALPHA         0.2f   // smoothing factor for the ADC average
#define MQ_RL_KOHM           10.0f  // load resistor (cancels out when R0 is auto-calibrated)

// Set to a value > 0 to skip auto-calibration (the robot prints the measured
// R0 values on the Serial Monitor after warm-up, so you can copy them here).
#define MQ2_R0_KOHM          0.0f
#define MQ7_R0_KOHM          0.0f
#define MQ135_R0_KOHM        0.0f

// Clean-air Rs/R0 ratios from the datasheets
#define MQ2_CLEAN_AIR_RATIO  9.83f
#define MQ7_CLEAN_AIR_RATIO  27.5f
#define MQ135_ATMO_CO2_PPM   400.0f

// Power-law fits of the datasheet curves: ppm = A * (Rs/R0)^B
#define MQ2_LPG_A            574.25f
#define MQ2_LPG_B           -2.222f
#define MQ2_SMOKE_A          3195.0f
#define MQ2_SMOKE_B         -2.273f
#define MQ7_CO_A             99.042f
#define MQ7_CO_B            -1.518f
#define MQ135_A              116.6020682f
#define MQ135_B             -2.769034857f

// =============================================================================
//  Purifier thresholds: ON above the first value, OFF below the second.
//  The gap between the two (hysteresis) stops relays from chattering when a
//  reading hovers around a single limit.
// =============================================================================
#define DUST_ON_UGM3         45     // PM2.5 or PM10, from the project spec
#define DUST_OFF_UGM3        30
#define CO_ON_PPM            30     // from the project spec
#define CO_OFF_PPM           10
#define LPG_ON_PPM           3150   // 15 % of propane's lower explosive limit (2.1 %)
#define LPG_OFF_PPM          2100   // 10 % LEL
#define SMOKE_ON_PPM         300    // tune on your robot
#define SMOKE_OFF_PPM        200
#define AQ_ON_PPM            1000   // MQ-135 CO2-equivalent, tune on your robot
#define AQ_OFF_PPM           800
#define HUMIDITY_LOW_ON_PCT  30     // mist on below this
#define HUMIDITY_LOW_OFF_PCT 35     // mist off above this

#define MANUAL_RUN_MS        60000UL  // how long a manual button press runs an actuator
#define RELAY_STAGGER_MS     100      // min gap between relay switches (limits inrush)

// =============================================================================
//  Scheduling and reporting
// =============================================================================
#define SENSOR_PERIOD_MS     1000   // SHT31 + gas ppm + purifier decisions
#define SHT31_MEASURE_MS     16     // high-repeatability conversion time (datasheet max 15.5)
#define PMS_STALE_MS         5000   // no frame for this long -> dust reading invalid
#define DEBOUNCE_MS          30
#define REPORT_PERIOD_MS     1000   // human-readable status on USB
#define TELEMETRY_PERIOD_MS  2000   // JSON line to the ESP for the web dashboard
