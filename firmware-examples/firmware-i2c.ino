// Require SimpleFOC library and SimpleFOCDrivers library

// this setting seems not working :(
#define SIMPLEFOC_RP2040_ADC_VDDA 3.0f

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"

#include "encoders/mt6835/MagneticSensorMT6835.h"
#include <EEPROM.h>

const float voltage_power_supply = 5.0;
const float current_limit = 0.5;

// Intended to be used for robots with motors on both sides
const uint8_t I2C_ADDR = 0x24; // LEFT
//const uint8_t I2C_ADDR = 0x25;  // RIGHT
const int I2C_CLOCK = 1000000;

SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(1, myMT6835SPISettings);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, -50.0f, A0, A2, A1);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, 10.0, 145.0 * 30 / 3.1415);
BLDCDriver6PWM driver = BLDCDriver6PWM(24, 21, 22, 20, 23, 19, 18);


struct MotorState {
  float angle;
  float velocity;
};

struct CalibData {
  char mark[5];
  float zero_electric_angle;
  Direction sensor_direction;
};

void readFromEEPROM(int startAddress, CalibData& data) {
  byte* dataPointer = (byte*)(void*)&data;
  for (size_t i = 0; i < sizeof(CalibData); ++i) {
    dataPointer[i] = EEPROM.read(startAddress + i);
  }
}


void setup() {
  // load calibration data from EEPROM if available
  EEPROM.begin(512);
  CalibData caldat;
  readFromEEPROM(0, caldat);
  if (caldat.mark[0] == 'd' && caldat.mark[1] == 'o' && caldat.mark[2] == 'n' && caldat.mark[3] == 'e') {
    motor.zero_electric_angle = caldat.zero_electric_angle;
    motor.sensor_direction = caldat.sensor_direction;
  }

  // initialise magnetic sensor hardware
  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setTX(3);
  SPI.setSCK(2);
  sensor.init();
  sensor.setZeroPosition(0);

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor.sensor_direction = Direction::CW;

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = voltage_power_supply;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);
  current_sense.skip_align = true;
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
  motor.voltage_limit = voltage_power_supply;
  motor.current_limit = current_limit;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // initialise motor
  motor.init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.monitor_variables = 0;

  // If both motors perform calibration operations at the same time, 
  // excessive current will flow, so a time difference is needed.
  if (I2C_ADDR == 0x25) {
    _delay(500);
  }

  motor.initFOC();


  motor.disable();

  _delay(10);

  Wire.setSDA(4);
  Wire.setSCL(5);

  Wire.setClock(I2C_CLOCK);
  Wire.begin(I2C_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void requestEvent() {
  MotorState dataToSend;
  dataToSend.angle = sensor.getAngle();
  dataToSend.velocity = sensor.getVelocity();
  byte* bytePtr = (byte*)(&dataToSend);
  for (int i = 0; i < sizeof(MotorState); i++) {
    Wire.write(*bytePtr++);
  }
}

inline void parseTorqueCommand() {
  motor.controller = MotionControlType::torque;
  setTargetFromI2C();
}

inline void parseVelocityCommand() {
  motor.controller = MotionControlType::velocity;
  setTargetFromI2C();
}

inline void parseAngleCommand() {
  motor.controller = MotionControlType::angle;
  setTargetFromI2C();
}

void receiveEvent(int numBytes) {
  if (numBytes <= 0) return;

  uint8_t registerAddress = Wire.read();

  switch (registerAddress) {
    case 0x00:
      parseEnableCommand();
      break;

    case 0x01:
      parseTorqueCommand();
      break;

    case 0x02:
      parseVelocityCommand();
      break;

    case 0x04:
      parseAngleCommand();

    default:
      break;
  }
}

inline void setTargetFromI2C() {
  if (Wire.available() == sizeof(float)) {
    union {
      float asFloat;
      uint8_t asBytes[sizeof(float)];
    } dataUnion;

    for (unsigned int i = 0; i < sizeof(float); i++) {
      dataUnion.asBytes[i] = Wire.read();
    }
    motor.target = dataUnion.asFloat;
  }
}

inline void parseEnableCommand() {
  if (Wire.available() == sizeof(uint8_t)) {
    uint8_t cmd = Wire.read();
    if (cmd == 1) {
      motor.target = 0;
      motor.enable();
    } else {
      motor.disable();
    }
  }
}

void loop() {
  motor.loopFOC();
  motor.move();
}
