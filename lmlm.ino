/*
Copyright (c) 2012 Jeff Rowberg
Copyright (c) 2023 Tuan Phan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "analogWave.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include <WDT.h>
#include <ArduinoBLE.h>
#include "lunarGateway.h"

#define WDT_TIMEOUT               5000

#define PRESSURE_CUT_LOW          2000
#define PRESSURE_REFILL           6000
#define PRESSURE_CUT_HIGH         9000
#define PRESSURE_VOLTAGE_TOP      40000
#define PRESSURE_VOLTAGE_BOTTOM   65000
#define ANGLE_CUT_LOW             0
#define ANGLE_CUT_HIGH            50
#define ANGLE_NOISE_FILTER        100
#define ANGLE_MIN_STEP            3
#define LUNAR_ACTIVE_ANGLE        5
#define LUNAR_WEIGH_RAMUP         3
#define LUNAR_RAMUP_PRESSURE_STEP 300
#define LUNAR_RAMUP_TIME_INTERVAL 100
#define LUNAR_WEIGH_RAMDOWN       25
#define LUNAR_RAMDOWN_PRESSURE_STEP 100
#define LUNAR_RAMDOWN_TIME_INTERVAL 200

#define LUNAR_WEIGH_STOP          35

#define REFILL_PIN                D2
#define BREW_PIN_IN               D3
#define MOTOR_ENABLE_OUT          D4
#define BREW_PIN_OUT              D5
#define SWITCH_IN                 D6

#define BNO055_SAMPLERATE_DELAY_MS  50 //how often to read data from the board
#define SCALE_SAMPLERATE_DELAY_MS  100 //how often to read data from the scale

bool ble_active;

// paddle sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);
bool sensor_ready;
double last_angle;
unsigned long sensor_read_last_time;

// DAC vars
uint16_t dac_sample[24];
analogWave wave(DAC, dac_sample, 24, 0);

// Linea mini vars
int32_t current_pressure;
int32_t last_pressure;

//  pin status
int refill_state;
int brew_state;
int switch_state;

// Scale
lunarGateway lunar;
bool found_lunar;
bool lunar_active;
bool lunar_session_init;
int lunar_state;
bool lunar_start_timer;
unsigned long scale_read_last_time;
unsigned long auto_ramup_last_time;
unsigned long auto_ramdown_last_time;

void motor_set_pressure(int32_t pressure);
void(* resetFunc) (void) = 0;

void ble_discover_lunar()
{
  BLEDevice peripheral = BLE.available();
  // print the local name, if present
  if (peripheral && peripheral.hasLocalName()) {
    if (peripheral.localName().indexOf("LUNAR-") == 0) {
      if (lunar.connect(&peripheral)) {
        Serial.println("Found LUNAR device");
        found_lunar = true;
        scale_read_last_time = 0;
        BLE.stopScan();
      }
    }
  }
}

void sensor_status(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("Current mode: 0x");
  Serial.println(bno.getMode(), HEX);
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
}

void sensor_cal_status(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void setup_sensor()
{
  sensor_ready = false;
  sensor_read_last_time = 0;

  if (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    Serial.print("No BNO055 detected");
    return;
  }
  Wire1.setClock(400000);
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(100);

  sensor_status();
  delay(100);
  sensor_cal_status();
  sensor_ready = true;
}

void printEvent(sensors_event_t* event)
{
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION 
      || event->type == SENSOR_TYPE_ACCELEROMETER
      || event->type == SENSOR_TYPE_GRAVITY) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  if (x != 0 || y != 0 || z != 0) {
    Serial.println();
    Serial.print(event->type);
    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
  }
}

void read_scale()
{
  if ((millis() - scale_read_last_time) > SCALE_SAMPLERATE_DELAY_MS) {
    lunar.read();
  }

  lunar.sendHeartBeat();
}

void read_sensor()
{
  sensors_event_t orientationData;
  int32_t pressure;
  double angle;

  if ((millis() - sensor_read_last_time) > BNO055_SAMPLERATE_DELAY_MS) {
    sensor_read_last_time = millis();
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    if (angle == 0)
      return;
    if (angle < 0 || angle > ANGLE_NOISE_FILTER)
      angle = ANGLE_CUT_LOW;
    if (angle > ANGLE_CUT_HIGH)
      angle = ANGLE_CUT_HIGH;
    if (abs(angle - last_angle) > ANGLE_MIN_STEP)
      last_angle = angle;
    else
      return;
    pressure = map(last_angle, ANGLE_CUT_LOW, ANGLE_CUT_HIGH, PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH);
    motor_set_pressure(pressure);
  }
}

void motor_enable(bool on)
{
  if (on) {
    digitalWrite(MOTOR_ENABLE_OUT, HIGH);
  } else {
    digitalWrite(MOTOR_ENABLE_OUT, LOW);
  }
}

void motor_set_pressure(int32_t pressure)
{
  if (pressure < PRESSURE_CUT_LOW)
    pressure = PRESSURE_CUT_LOW;
  if (pressure > PRESSURE_CUT_HIGH)
    pressure = PRESSURE_CUT_HIGH;
  current_pressure = pressure;
  last_pressure = map(dac_sample[0],
                              PRESSURE_VOLTAGE_TOP, PRESSURE_VOLTAGE_BOTTOM,
                              PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH);
  last_pressure = PRESSURE_CUT_HIGH - (last_pressure - PRESSURE_CUT_LOW);
  Serial.print(F("Motor set pressure: "));Serial.println(pressure);
  Serial.print(F("Last pressure: "));Serial.println(last_pressure);
  if (pressure == last_pressure)
    return;

  for (int i = 0; i < sizeof(dac_sample) / sizeof(dac_sample[0]); i++) {
    dac_sample[i] = (uint16_t)map(pressure,
                              PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH,
                              PRESSURE_VOLTAGE_TOP, PRESSURE_VOLTAGE_BOTTOM);
    dac_sample[i] = PRESSURE_VOLTAGE_BOTTOM - (dac_sample[i] - PRESSURE_VOLTAGE_TOP);               
  }
  wave.offset(0);
}

void motor_defaut_pressure()
{
  motor_set_pressure(PRESSURE_CUT_HIGH);
}

void lunar_auto_ramup()
{
  if (millis() - auto_ramup_last_time > LUNAR_RAMUP_TIME_INTERVAL) {
    auto_ramup_last_time = millis();
    motor_set_pressure(current_pressure + LUNAR_RAMUP_PRESSURE_STEP);
  }
}

void lunar_auto_ramdown()
{
  if (millis() - auto_ramdown_last_time > LUNAR_RAMDOWN_TIME_INTERVAL) {
    auto_ramdown_last_time = millis();
    motor_set_pressure(current_pressure - LUNAR_RAMDOWN_PRESSURE_STEP);
  }
}

void setup() {
  refill_state = 0;
  brew_state = 0;
  switch_state = 0;

  // refill sensor
  pinMode(REFILL_PIN, INPUT);
  // brew sensor
  pinMode(BREW_PIN_IN, INPUT);
  // Motor enable out
  pinMode(MOTOR_ENABLE_OUT, OUTPUT);
  // Brew enable out
  pinMode(BREW_PIN_OUT, OUTPUT);
  // Switch in
  pinMode(SWITCH_IN, INPUT_PULLUP);
 
  digitalWrite(MOTOR_ENABLE_OUT, LOW);

  // initialize serial communication
  Serial.begin(115200);

  // begin initialization
  found_lunar = false;
  ble_active = false;
  if (BLE.begin()) {
    ble_active = true;
    BLE.scan();
  } else {
    Serial.println("Failed to begin BLE");
  }

  setup_sensor();
  wave.begin(1);
  WDT.begin(WDT_TIMEOUT);
  motor_defaut_pressure();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  WDT.refresh();

  if (sensor_ready && switch_state == 1)
    read_sensor();

  if (ble_active) {
    BLE.poll();

    if (found_lunar) {
      if (lunar.connected()) {
        read_scale();
        if (lunar_active && (!sensor_ready || last_angle < LUNAR_ACTIVE_ANGLE)) {
          if (!lunar_session_init) {
            Serial.println("Lunar session started");
            lunar_session_init = true;
            auto_ramup_last_time = 0;
            auto_ramdown_last_time = 0;
            lunar_start_timer = false;
            lunar_state = 0;
            lunar.stopTimer();
            lunar.doTare();          
          } else {
            switch (lunar_state) {
            case 0:
              if (lunar.weight > LUNAR_WEIGH_RAMUP && lunar.weight <= LUNAR_WEIGH_RAMDOWN) {
                lunar_state++;
                if (!lunar_start_timer) {
                  Serial.println("Lunar timer started");
                  lunar.startTimer();
                }
                Serial.println("Lunar ramup");
              }
              break;
            case 1:
              lunar_auto_ramup();
              if (lunar.weight > LUNAR_WEIGH_RAMDOWN && lunar.weight <= LUNAR_WEIGH_STOP) {
                lunar_state++;
                Serial.println("Lunar ramdown");
              }
              break;
            case 2:
              lunar_auto_ramdown();
              if (lunar.weight > LUNAR_WEIGH_STOP) {
                lunar_state++;
                Serial.println("Lunar stopped");
                motor_enable(false);
                lunar.stopTimer();
                lunar_active = false;
              }
              break;
            }
          }
        }
      } else if (!lunar_active) {
        BLE.scan();
        found_lunar = false;
      }
    } else {
      if (!lunar_active)
        ble_discover_lunar();
    }
  }

  if (digitalRead(BREW_PIN_IN) == LOW) {
    if (brew_state == 0) {
      Serial.println(F("BREW started"));
      brew_state = 1;
      motor_enable(true);
      delay(100);
    }
  } else {
    if (brew_state == 1) {
      Serial.println(F("BREW stopped"));
      brew_state = 0;
      motor_enable(false);
    }
  }
  if (digitalRead(REFILL_PIN) == LOW) {
    if (refill_state == 0) {
      Serial.println(F("REFILL started"));
      refill_state = 1;
      motor_set_pressure(PRESSURE_REFILL);
      motor_enable(true);
    }
  } else {
    if (refill_state == 1) {
      Serial.println(F("REFILL stopped"));
      refill_state = 0;
      motor_set_pressure(last_pressure);
      motor_enable(false);
    }
  }

  if (digitalRead(SWITCH_IN) == LOW) {
    if (switch_state == 0) {
      Serial.println(F("Switch pressed"));
      switch_state = 1;
      last_angle = 0;
      lunar_active = true;
      lunar_session_init = false;
      motor_defaut_pressure();
    }
  } else {
    if (switch_state == 1) {
      Serial.println(F("Switch de-pressed"));
      switch_state = 0;
      lunar_active = false;
      motor_enable(false);
    }
  }

  delay(5);
}