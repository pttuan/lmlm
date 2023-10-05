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

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "analogWave.h"
#include "Wire.h"
#include <WDT.h>
#include <ArduinoBLE.h>
#include <HardwareBLESerial.h>

#define WDT_TIMEOUT               5000

#define MOTION_DURATION_CHECK     200
#define MOTION_SENSITIVE          20
#define MOTION_AVERAGE            5
#define WEIGHT_MOVING             20

#define ACC_SENSOR_LIFE_DURATION  3600000 // 1 hour

#define PRESSURE_RATIO            (float)3.5
#define PRESSURE_CUT_LOW          2000
#define PRESSURE_REFILL           6000
#define PRESSURE_CUT_HIGH         9000
#define PRESSURE_VOLTAGE_TOP      40000
#define PRESSURE_VOLTAGE_BOTTOM   64000
#define START_MOVE_PRESSURE       (PRESSURE_CUT_LOW - 1500) //minues due to error move reading from begin
#define MOTOR_DELAY_UPDATE        100

#define VCC_SENSOR_PIN            D0
#define REFILL_PIN                D2
#define BREW_PIN_IN               D3
#define MOTOR_ENABLE_OUT          D4
#define BREW_PIN_OUT              D5
#define SWITCH_IN                 D6

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();

// MPU control/status vars
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
bool acc_sensor_ready;
unsigned long acc_sensor_enable_timer;

// DAC vars
uint16_t dac_sample[24];
analogWave wave(DAC, dac_sample, 24, 0);

// Linea mini vars
uint32_t motion_detect_timer;
int32_t last_delta;
int last_move; // -1->left, 0-neutral, 1 -> right
int32_t acc_total;
int32_t current_pressure;
int32_t last_pressure;
bool motion_monitor;

//  pin status
int refill_state;
int brew_state;
int switch_state;

void motor_analog_out(int32_t pressure);
void(* resetFunc) (void) = 0;

int get_temp()
{
  if (!acc_sensor_ready)
    return 0;

  return (mpu.getTemperature() / 340 + 36.5);
}

void enable_acc_sensor(bool enable)
{
  if (enable) {
    digitalWrite(VCC_SENSOR_PIN, LOW);
    delay(500);
    digitalWrite(VCC_SENSOR_PIN, HIGH);
    delay(1000);
    acc_sensor_enable_timer = millis();
  } else {
    acc_sensor_ready = false;
    Wire1.end();
    delay(10);
    digitalWrite(VCC_SENSOR_PIN, LOW);
  }
}

void setup_acc_sensor()
{
  bool test_conn;

  Wire1.begin();
  Wire1.setClock(400000);
  delay(10);

  bleSerial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  mpu.setTempSensorEnabled(true);

  // verify connection
  bleSerial.println(F("Testing device connections..."));
  test_conn = mpu.testConnection();
  bleSerial.println(test_conn ?
                  F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  if (!test_conn) {
    goto failed;
  }

  // load and configure the DMP
  bleSerial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(126);
  mpu.setYGyroOffset(-26);
  mpu.setZGyroOffset(-36);
  mpu.setZAccelOffset(1060); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    bleSerial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    bleSerial.println(F("OK"));
    acc_sensor_ready = true;
    return;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    bleSerial.print(F("DMP Initialization failed (code "));
    bleSerial.print(devStatus);
    bleSerial.println(F(")"));
  }

failed:
  enable_acc_sensor(false);
}

void read_accelerator()
{
  int32_t delta;
  uint8_t has_data;

  has_data = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

  if (!switch_state)
    return;

  if (!motion_monitor) {
    motion_monitor = true;
    motion_detect_timer = 0;
  }

  // read a packet from FIFO
  if (has_data) { // Get the Latest packet 
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    //Serial.print("aaReal.x\t");Serial.print(aaReal.x);Serial.println(F(""));
    //Serial.print("aaWorld.x\t");Serial.print(aaWorld.x);Serial.println(F(""));

    if (motion_detect_timer == 0) {
      if (abs(aaWorld.x) > MOTION_SENSITIVE) {
        motion_detect_timer = millis();
        acc_total = aaWorld.x;
      }
    } else {
      if ((millis() - motion_detect_timer) > MOTION_DURATION_CHECK) {
        motion_detect_timer = 0;
        delta = ((float)acc_total / ((float)MOTION_DURATION_CHECK / MOTION_AVERAGE)) * WEIGHT_MOVING;
        //Serial.print("acc_total\t");Serial.print(acc_total);Serial.println(F(""));
        bleSerial.print(F("Delta\t"));bleSerial.print(delta);bleSerial.println(F(""));
        delta = (float)delta * PRESSURE_RATIO;
        if (acc_total > 0) {
          bleSerial.println(F("Moving right: "));
          if (last_move < 0) {
            // Moved left last time, sudden move?
            last_delta = delta;
            delta = 0;
          } else {
            delta += last_delta;
            last_delta = 0;
          }
          last_move = 1;
        } else {
          bleSerial.println(F("Moving left: "));
          if (last_move > 0) {
            // Moved right last time, sudden move?
            last_delta = delta;
            delta = 0;
          }
          else {
            delta += last_delta;
            last_delta = 0;
          }
          last_move = -1;
        }
        if (delta != 0) {
          current_pressure -= delta;
          if (current_pressure > PRESSURE_CUT_HIGH)
            current_pressure = PRESSURE_CUT_HIGH;
          if (delta > 0 && current_pressure < PRESSURE_CUT_LOW)
            current_pressure = PRESSURE_CUT_LOW;
          motor_analog_out(current_pressure);
        }
      } else {
        acc_total += aaWorld.x;
      }
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
  motor_analog_out(PRESSURE_CUT_LOW);
}

void setup() {
  acc_total = 0;
  last_delta = 0;
  last_move = 0;
  current_pressure = 0;
  last_pressure = 0;
  refill_state = 0;
  brew_state = 0;
  switch_state = 0;
  motion_monitor = false;
  motion_detect_timer = 0;

  pinMode(VCC_SENSOR_PIN, OUTPUT);
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

  enable_acc_sensor(true);
  setup_acc_sensor();
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

  if (digitalRead(BREW_PIN_IN) == LOW) {
    if (brew_state == 0) {
      bleSerial.println(F("BREW started"));
      brew_state = 1;
      delay(100);
      motor_enable(true);
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
      current_pressure = START_MOVE_PRESSURE;
      motor_defaut_speed();
      bleSerial.print(F("acc_sensor_ready\t")); bleSerial.print(acc_sensor_ready); bleSerial.println(F(""));
      bleSerial.print(F("temp\t")); bleSerial.print(get_temp()); bleSerial.println(F(""));
    }
  } else {
    if (switch_state == 1) {
      bleSerial.println(F("Switch de-pressed"));
      motion_monitor = false;
      switch_state = 0;
      motor_enable(false);
      bleSerial.print(F("acc_sensor_enable_timer\t")); bleSerial.print(acc_sensor_enable_timer); bleSerial.println(F(""));
      bleSerial.print(F("millis\t")); bleSerial.print(millis()); bleSerial.println(F(""));
      if (!acc_sensor_ready || (get_temp() <= 37)) {
        resetFunc();
      }
    }
  }

  if (acc_sensor_ready) {
    read_accelerator();

    if (abs(millis() - acc_sensor_enable_timer) > ACC_SENSOR_LIFE_DURATION) {
      bleSerial.println(F("Turn off sensor"));
      enable_acc_sensor(false);
    }
  }

  delay(5);
}