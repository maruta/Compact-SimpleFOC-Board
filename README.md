# Compact SimpleFOC Controller Board

This repository contains design data for a BLDC motor controller board composed of:

* RP2040 (Controller)
* TMC6300 (Driver)
* MT6535 (Magnetic Angle Sensor)
* INA240 (Current Sense Amplifier)

and mainly designed for a tiny inverted pendulm project.

[![Wheeled inverted pendulum powered by SimpleFOC-1080p60H mov](https://github.com/maruta/Compact-SimpleFOC-Board/assets/486675/f13f1e1a-335b-40cc-a1a0-964be4d5bf98)](https://youtu.be/5sXEKwi0VvQ?si=sFahMU4BhdU_3RiM)

## Disclaimer

The information and design files in this repository are provided "as is" without warranty of any kind, express or implied. We make no guarantees regarding the functionality or reliability of these designs. Users assume full responsibility for the application and use of these files. Any commercial application of these designs is at the user's own risk.

## Description

The board is designed primarily for the SimpleFOC library, with an emphasis on easy customization of the software to suit your application.

The RP2040 component in this design is compatible with the Raspberry Pi Pico, minus the USB connector. It can be programmed using the Arduino IDE with the [Raspberry Pi Pico Arduino core](https://github.com/earlephilhower/arduino-pico), using the [Raspberry Pi Debug Probe](https://www.raspberrypi.com/products/debug-probe/).


### Board Specifications
- Voltage Range: 2-5.5 V
- Maximum Current: 2 A (1.4 Arms)

### Motor Compatibility
- The board has been tested with the RCTIMER GBM2804 motor.

### Connector Details

- J1: RP2040 SWD Connector (3-pin JST SH connector)
  - Pin 1: SWCLK
  - Pin 2: GND
  - Pin 3: SWDIO
- J2: Motor Connector (not populated; 2.54 mm)
  - Pin 1: Motor Phase U
  - Pin 2: Motor Phase V
  - Pin 3: Motor Phase W
- J3: Power and Interface Connector
  - 4-pin JST SH Connector
  - compatible with Qwiic
  - Logic voltage is 3.3 V
  - Pin assign
    - Pin 1: Ground (GND)
    - Pin 2: Positive Voltage (V+)
    - Pin 3: GPIO4 (I2C0 SDA / UART1_TX)
    - Pin 4: GPIO5 (I2C0 SCL / UART1_RX)

## Manufacturing

Use the `production` folder for manufacturing this board with [JLCPCB](https://jlcpcb.com/).

### JLCPCB Ordering Guide

PCB Manufacturing Specifications:
- File to Upload: `gerber.zip`
- Key Settings:
  - Via Covering: Epoxy Filled & Capped
  - Min. via hole size/diameter: 0.2mm(0.3/0.35mm)

PCB Assembly Details:
- Key Settings:
  - Assembly Side: Both Sides
  - Confirm Parts Placement: Required
  - Board Depaneling & Edge Rail: As per requirement
- Required Files: `BOM: bom.csv`, `CPL: positions.csv`
- Components not automatically selected are specified by LCSC part numbers in the circuit files.
- RT6150 and TMC6300 need to be ordered through Global Sourcing Parts Service due to their unavailability at JLCPCB. *Note that the quantity required may be more than the actual quantity to be implemented.*

## Circuit Editing

* Created using KiCAD v7.

* Component models from [Ultra Librarian](https://www.ultralibrarian.com/) are not included due to licensing. Download these models to edit the circuit. See README.md in the following folders for more information:

  - ul_RT6150B-33GQW
  - ul_TMC6300-LA
  - ul_W25Q16JVUXIQ

* Use [Fabrication Toolkit](https://github.com/bennymeg/JLC-Plugin-for-KiCad) to generate . Generated files are stored in the `production` folder.

## License

Files in this repository are under the MIT License.
