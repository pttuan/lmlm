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
#include <HardwareBLESerial.h>

#define WDT_TIMEOUT               5000

#define PRESSURE_CUT_LOW          2000
#define PRESSURE_REFILL           6000
#define PRESSURE_CUT_HIGH         9000
#define PRESSURE_VOLTAGE_TOP      40000
#define PRESSURE_VOLTAGE_BOTTOM   65000
#define PRESSURE_MIN_STEP         100
#define ANGLE_CUT_LOW             0
#define ANGLE_CUT_HIGH            50
#define ANGLE_NOISE_FILTER        100

#define REFILL_PIN                D2
#define BREW_PIN_IN               D3
#define MOTOR_ENABLE_OUT          D4
#define BREW_PIN_OUT              D5
#define SWITCH_IN                 D6

#define BNO055_SAMPLERATE_DELAY_MS  50 //how often to read data from the board

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();

bool sensor_ready;
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

void motor_analog_out(int32_t pressure);
void(* resetFunc) (void) = 0;

void sensor_status(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  bleSerial.println("");
  bleSerial.print("Current mode: 0x");
  bleSerial.println(bno.getMode(), HEX);
  bleSerial.print("System Status: 0x");
  bleSerial.println(system_status, HEX);
  bleSerial.print("Self Test:     0x");
  bleSerial.println(self_test_results, HEX);
  bleSerial.print("System Error:  0x");
  bleSerial.println(system_error, HEX);
  bleSerial.println("");
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
    bleSerial.print("! ");
  }

  /* Display the individual values */
  bleSerial.print("Sys:");
  bleSerial.print(system, DEC);
  bleSerial.print(" G:");
  bleSerial.print(gyro, DEC);
  bleSerial.print(" A:");
  bleSerial.print(accel, DEC);
  bleSerial.print(" M:");
  bleSerial.println(mag, DEC);
}

void setup_sensor()
{
  sensor_ready = false;
  sensor_read_last_time = 0;

  if (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    bleSerial.print("No BNO055 detected");
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

void read_sensor()
{
  sensors_event_t orientationData;
  double angle;
  int32_t pressure;

  if ((millis() - sensor_read_last_time) > BNO055_SAMPLERATE_DELAY_MS) {
    sensor_read_last_time = millis();
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    if (angle == 0 || !switch_state)
      return;
    bleSerial.print("Handle angle\t");bleSerial.println(angle);
    if (angle < 0 || angle > ANGLE_NOISE_FILTER)
      angle = ANGLE_CUT_LOW;
    if (angle > ANGLE_CUT_HIGH)
      angle = ANGLE_CUT_HIGH;
    pressure = map(angle, ANGLE_CUT_LOW, ANGLE_CUT_HIGH, PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH);
    if (abs(pressure - current_pressure) > PRESSURE_MIN_STEP) {
      current_pressure = pressure;
      motor_analog_out(pressure);
    }
  }
}

void motor_analog_out(int32_t pressure)
{
  bleSerial.print(F("Motor set pressure\t"));
  bleSerial.print(pressure);
  bleSerial.println(F(""));

  if (pressure < PRESSURE_CUT_LOW)
    pressure = PRESSURE_CUT_LOW;
  if (pressure > PRESSURE_CUT_HIGH)
    pressure = PRESSURE_CUT_HIGH;

  last_pressure = map(dac_sample[0],
                              PRESSURE_VOLTAGE_TOP, PRESSURE_VOLTAGE_BOTTOM,
                              PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH);
  last_pressure = PRESSURE_CUT_HIGH - (last_pressure - PRESSURE_CUT_LOW);

  for (int i = 0; i < sizeof(dac_sample) / sizeof(dac_sample[0]); i++) {
    dac_sample[i] = (uint16_t)map(pressure,
                              PRESSURE_CUT_LOW, PRESSURE_CUT_HIGH,
                              PRESSURE_VOLTAGE_TOP, PRESSURE_VOLTAGE_BOTTOM);
    dac_sample[i] = PRESSURE_VOLTAGE_BOTTOM - (dac_sample[i] - PRESSURE_VOLTAGE_TOP);               
  }
  wave.offset(0);
}

void motor_defaut_speed()
{
  current_pressure = PRESSURE_CUT_HIGH;
  motor_analog_out(PRESSURE_CUT_HIGH);
}

void setup() {
  current_pressure = 0;
  last_pressure = 0;
  refill_state = 0;
  brew_state = 0;
  switch_state = 0;
  sensor_ready = false;
  sensor_read_last_time = 0;

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

  if (!bleSerial.beginAndSetupBLE("SerialPassthrough")) {
    bleSerial.println(F("failed to initialize HardwareSerial!"));
  }

  setup_sensor();
  wave.begin(1);
  WDT.begin(WDT_TIMEOUT);
  motor_defaut_speed();
}

void motor_enable(bool on)
{
  if (on) {
    digitalWrite(MOTOR_ENABLE_OUT, HIGH);
  } else {
    digitalWrite(MOTOR_ENABLE_OUT, LOW);
  }
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  WDT.refresh();
  BLE.poll();

  if (sensor_ready && switch_state == 1)
    read_sensor();


  if (digitalRead(BREW_PIN_IN) == LOW) {
    if (brew_state == 0) {
      bleSerial.println(F("BREW started"));
      brew_state = 1;
      motor_enable(true);
      delay(100);
    }
  } else {
    if (brew_state == 1) {
      bleSerial.println(F("BREW stopped"));
      brew_state = 0;
      motor_enable(false);
    }
  }
  if (digitalRead(REFILL_PIN) == LOW) {
    if (refill_state == 0) {
      bleSerial.println(F("REFILL started"));
      refill_state = 1;
      motor_analog_out(PRESSURE_REFILL);
      motor_enable(true);
    }
  } else {
    if (refill_state == 1) {
      bleSerial.println(F("REFILL stopped"));
      refill_state = 0;
      motor_analog_out(last_pressure);
      motor_enable(false);
    }
  }

  if (digitalRead(SWITCH_IN) == LOW) {
    if (switch_state == 0) {
      bleSerial.println(F("Switch pressed"));
      switch_state = 1;
      motor_defaut_speed();
      bleSerial.print("sensor_ready\t");bleSerial.println(sensor_ready);
    }
  } else {
    if (switch_state == 1) {
      bleSerial.println(F("Switch de-pressed"));
      switch_state = 0;
      motor_enable(false);
    }
  }

  delay(5);
}