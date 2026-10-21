# Big Smoke: Toxic Smoke Detection and Odor Neutralization Robot

A mobile air purifier on a mecanum-wheel base. It measures smoke, combustible gas, carbon monoxide, particulates, temperature and humidity, then drives a mist maker, a filter fan and an ionizer to clean the air. It can be driven over Bluetooth or Wi-Fi, avoid obstacles on its own, or follow an object.

<p align="center">
  <img src="https://github.com/user-attachments/assets/2c1734b1-d977-4678-82d7-f67c5fcc00d5" height="300" />
  <img src="https://github.com/user-attachments/assets/27ba1fb0-070f-4e3d-8c36-ef3af18a15af" height="300"/>
</p>

## Hardware

| Part | Interface | Purpose |
|---|---|---|
| Arduino Mega 2560 | | Main controller |
| Adafruit Motor Shield v1 (L293D) | shift register + PWM | 4 DC motors on mecanum wheels |
| MQ-2 | ADC | LPG, smoke |
| MQ-7 | ADC | Carbon monoxide |
| MQ-135 | ADC | Air quality (CO2-equivalent), temperature/humidity compensated |
| SHT31 | I2C | Temperature, humidity |
| Plantower PMS | UART | PM2.5, PM10 |
| HC-SR04 on SG90 servo | GPIO + external interrupt | Distance, left/right scanning |
| 2 x IR obstacle sensors | GPIO | Object following |
| 4 x relays | GPIO | Mist maker, mist fan, filter fan, ionizer |
| HC-05 | UART | Bluetooth remote |
| ESP (NodeMCU / ESP32) | UART | Wi-Fi remote and web telemetry |
| Active buzzer, RGB LED, 4 buttons | GPIO | Alarm, status, local control |

## Pin map

| Function | Pin(s) |
|---|---|
| Motor shield (reserved) | D3-D8, D11, D12 |
| Servo | D10 |
| Ultrasonic echo / trigger | D2 (INT) / D22 |
| IR left / right | D23 / D25 |
| SHT31 SDA / SCL | D20 / D21 |
| Bluetooth HC-05 RX / TX | D19 / D18 (Serial1) |
| ESP RX / TX | D17 / D16 (Serial2) |
| PMS RX / TX | D15 / D14 (Serial3) |
| MQ-2 / MQ-135 / MQ-7 | A0 / A1 / A2 |
| Relays: mist, mist fan, filter, ionizer | D26, D27, D28, D29 |
| Buzzer | D30 |
| Buttons: mode, purifier, mist, fan | D32, D33, D34, D35 (to GND) |
| RGB LED | D36, D37, D38 |

The ESP and the PMS use 3.3 V logic. Put a voltage divider (for example 1 kΩ / 2 kΩ) on the Mega TX line going into each of them.

## How the firmware works

`loop()` never blocks. Each subsystem is a task that checks `millis()`/`micros()` and returns immediately when it has nothing to do:

| Task | Rate | What it does |
|---|---|---|
| `sonarTask` | 60 ms | Fires a trigger pulse; the echo is timed by a pin-change interrupt instead of `pulseIn()` |
| `gasSampleTask` | 100 ms | Samples the three MQ sensors and smooths them (exponential moving average) |
| `shtTask` | 1 s | Starts an SHT31 conversion, collects it 16 ms later, checks the CRC |
| `pmsTask` | every pass | Drains the PMS UART and parses frames |
| `sensorTask` | 1 s | Converts gas readings to ppm, handles warm-up/calibration, runs the purifier rules |
| `inputTask` | every pass | Debounced buttons, remote commands from Bluetooth, Wi-Fi and USB |
| `driveTask` | every pass | Drive-mode state machine |
| `outputTask` | every pass | Staggered relay switching, buzzer pattern, status LED |
| `reportTask` | 1 s / 2 s | Status on USB, JSON to the ESP, sent without ever waiting on the UART |

Two state machines run side by side.

**Drive mode:** `IDLE`, `REMOTE`, `AVOID`, `FOLLOW`. `AVOID` has its own sub-states: cruise, back up, look left, look right, turn, pause. AVOID and FOLLOW stop the robot if the ultrasonic sensor stops answering.

**Purifier mode:** `AUTO` or `MANUAL`. In MANUAL a button press runs an actuator for 60 s. The CO/LPG alarm sounds in every mode.

The drive uses mecanum inverse kinematics: a body velocity (forward, strafe, rotate) is mixed into four wheel speeds, so all ten movements come from one function.

## Purifier rules (AUTO)

Each limit has an ON level and a lower OFF level so relays don't chatter.

| Condition | ON | OFF | Actions |
|---|---|---|---|
| Dust (PM2.5 or PM10) | > 45 µg/m³ | < 30 µg/m³ | Filter fan, ionizer |
| Smoke (MQ-2) | > 300 ppm | < 200 ppm | Filter fan, ionizer |
| Air quality (MQ-135) | > 1000 ppm | < 800 ppm | Filter fan, ionizer |
| CO (MQ-7) | > 30 ppm | < 10 ppm | Alarm, mist, filter fan |
| LPG (MQ-2) | > 15 % LEL | < 10 % LEL | Alarm, mist, filter fan |
| Low humidity | < 30 % | > 35 % | Mist |

## Remote protocol

One ASCII character per command. Bluetooth, Wi-Fi and the USB Serial Monitor all use it.

| Key | Action | Key | Action |
|---|---|---|---|
| F / B | Forward / back | I / J | Diagonal forward-right / forward-left |
| L / R | Strafe left / right | K / M | Diagonal back-right / back-left |
| Q / E | Rotate left / right | S or T | Stop |
| 0 1 2 3 | IDLE / REMOTE / AVOID / FOLLOW | P | Purifier AUTO / MANUAL |
| N | Manual mist (toggle) | O | Manual filter fan + ionizer (toggle) |

Any drive key switches the robot to REMOTE mode.

## Building

1. Arduino IDE, board **Arduino Mega or Mega 2560**.
2. Library Manager: **Adafruit Motor Shield library** (v1, `AFMotor.h`), **PMS Library** (Mariusz Kacki), **Servo**.
3. Open `BigSmoke/BigSmoke.ino` and upload.

## First power-up

1. Power up in clean air. The LED blinks blue for 60 s while the MQ heaters warm up, then the sensors calibrate themselves.
2. Open the Serial Monitor at 115200 baud. Copy the printed R0 values into `config.h` so later boots skip calibration.
3. Send `F`, `L`, `Q` and `I` from the Serial Monitor with the wheels off the ground. If a wheel spins the wrong way, flip its sign in `MOTOR_POLARITY`.

## Measuring responsiveness

The 1 s status line ends with `loop max=…us avg=…us`. That is the worst-case and average time of one pass through `loop()`. It is also the upper bound on how long a remote command or an obstacle reading waits before the robot reacts.

## Limitations

- MQ sensors give indicative values, not certified measurements. The MQ-7 is most accurate with its datasheet heater cycle (5 V / 1.4 V), which this hardware does not do.
- The firmware for the ESP board is not in this repository. It needs to forward single-character commands to the Mega and can read the JSON telemetry lines.

