/*
 This code uses J4 as a UART and provide Commander interface.
 
 Also performs calibration of the sensors and write to EEPROM.
*/

// this setting seems not working :(
#define SIMPLEFOC_RP2040_ADC_VDDA 3.0f

#include <SimpleFOC.h>
#include <EEPROM.h>

#include "SimpleFOCDrivers.h"

#include "encoders/mt6835/MagneticSensorMT6835.h"


SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(1, myMT6835SPISettings);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, -50.0f, A0, A2, A1);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, 10.0f, 145.0f * 30.0f / _PI);
BLDCDriver6PWM driver = BLDCDriver6PWM(24, 21, 22, 20, 23, 19, 18);

float target_voltage = 5;
struct CalibData {
  char mark[5];
  float zero_electric_angle;
  Direction sensor_direction;
};

void writeToEEPROM(int startAddress, const CalibData& data) {
  const byte* dataPointer = (const byte*)(const void*)&data;
  for (size_t i = 0; i < sizeof(CalibData); ++i) {
    EEPROM.write(startAddress + i, dataPointer[i]);
  }
}

void readFromEEPROM(int startAddress, CalibData& data) {
  byte* dataPointer = (byte*)(void*)&data;
  for (size_t i = 0; i < sizeof(CalibData); ++i) {
    dataPointer[i] = EEPROM.read(startAddress + i);
  }
}

// commander interface
Commander command = Commander(Serial2);
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doTarget(char* cmd) {
  command.scalar(&target_voltage, cmd);
}
void calcKV(char* cmd) {
  Serial2.println(motor.shaft_velocity / motor.target * 30.0f / _PI);
}

void setup() {
  EEPROM.begin(512);
  // initialise magnetic sensor hardware
  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setTX(3);
  SPI.setSCK(2);
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor.sensor_direction = Direction::CW;
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 5.0;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 10.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  motor.PID_velocity.limit = 0.5;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.05;
  // angle loop PID (not tuned)
  motor.P_angle.P = 30.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 100.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  motor.PID_current_q.P = 5.0;
  motor.PID_current_q.I = 5000.0;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 0.0;
  motor.PID_current_q.limit = 5.0;
  // Low pass filtering time constant
  motor.LPF_current_q.Tf = 0.005;
  // current d loop PID
  motor.PID_current_d.P = 5.0;
  motor.PID_current_d.I = 5000.0;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.output_ramp = 0.0;
  motor.PID_current_d.limit = 5.0;
  // Low pass filtering time constant
  motor.LPF_current_d.Tf = 0.005;
  // Limits
  motor.velocity_limit = 100.0;
  motor.voltage_limit = 5.0;
  motor.current_limit = 0.5;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // use monitoring with serial for motor init
  // monitoring port
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(250000);
  // comment out if not needed
  motor.useMonitoring(Serial2);

  // initialise motor
  motor.init();


  motor.linkCurrentSense(&current_sense);
  current_sense.init();

  Serial2.print("current_sense.gain:\t");
  Serial2.print(current_sense.gain_a);
  Serial2.print(", ");
  Serial2.print(current_sense.gain_b);
  Serial2.print(", ");
  Serial2.println(current_sense.gain_c);
  Serial2.print("current_sense.offset_i:\t");
  Serial2.print(current_sense.offset_ia);
  Serial2.print(", ");
  Serial2.print(current_sense.offset_ib);
  Serial2.print(", ");
  Serial2.println(current_sense.offset_ic);

  motor.voltage_sensor_align = motor.voltage_limit;

  sensor.setZeroPosition(0);

  uint8_t st = sensor.getStatus();
  Serial2.print("MT6835: status = ");
  Serial2.println(st);


  uint16_t pos = sensor.getZeroPosition();
  Serial2.print("MT6835: zero position = ");
  Serial2.println(pos);



  // align encoder and start FOC
  motor.initFOC();

  Serial2.print("current_sense.gain:\t");
  Serial2.print(current_sense.gain_a);
  Serial2.print(", ");
  Serial2.print(current_sense.gain_b);
  Serial2.print(", ");
  Serial2.println(current_sense.gain_c);
  Serial2.print("current_sense.offset_i:\t");
  Serial2.print(current_sense.offset_ia);
  Serial2.print(", ");
  Serial2.print(current_sense.offset_ib);
  Serial2.print(", ");
  Serial2.println(current_sense.offset_ic);

  CalibData caldat = { "done", motor.zero_electric_angle, motor.sensor_direction };
  writeToEEPROM(0, caldat);
  EEPROM.commit();

  CalibData dataToRead;
  readFromEEPROM(0, dataToRead);
  EEPROM.end();
  Serial2.println(dataToRead.mark);
  Serial2.println(dataToRead.zero_electric_angle, 4);
  Serial2.println(dataToRead.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");
  // set the inital target value
  motor.target = 0;

  // define the motor id
  command.add('A', onMotor, "motor");
  command.add('T', doTarget, "target voltage");
  command.add('K', calcKV, "calculate KV rating");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  motor.monitor_variables = 0;
  _delay(1000);
}


void loop() {
  motor.loopFOC();

  motor.move();
  motor.monitor();
  command.run();
}
