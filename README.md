
The MCC400 Marine Cabin Controller monitors vessel cabin/house systems. It collects data and transmits this data on the vessel's NMEA2000 (N2K) bus. The MCC400 monitors:
* Fridge, freezer, battery and inverter temperatures using [DS18B20 1-wire Thermometer sensors](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) connected to the 1-wire bus.
* Barometric pressure, cabin temperature and relative humidity using a [Bosch BME820 Environmental Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) on the controller board.
* A bilge water level switch via an opto-isolated input.
* Fresh water consumption using a hall effect flow meter at the input to the fresh water pump, connected to an opto-isolated input.
* Up to four tank level senders connected to 14v tolerant ADC inputs.
* Up to four MQ gas sensors connected to 5v ADC inputs.
* Three voltage / current / power channels using external shunt resistors. These are usually conntected to the house battery bank, the cranking battery and the solar charge controller.

The MCC400 also has two serial ports: one RS232 (RX/TX) and one RS485. These can be connected to other serial devices, e.g. a GPS or a solar charge controller.

**Contents**
- [Quick-start Guide](#quick-start-guide)
  - [Unboxing](#unboxing)
  - [Connecting to the NMEA 2000 Bus](#connecting-to-the-nmea-2000-bus)
  - [Turning the device on and off](#turning-the-device-on-and-off)
- [Installation](#installation)
  - [Connecting to the NMEA 2000 Bus](#connecting-to-the-nmea-2000-bus-1)
  - [Connecting the Sensors](#connecting-the-sensors)
    - [1-Wire Temperature Sensors](#1-wire-temperature-sensors)
    - [Flow meter](#flow-meter)
    - [Battery Shunts](#battery-shunts)
    - [Analog Tank Level Senders](#analog-tank-level-senders)
    - [Bilge Switch](#bilge-switch)
    - [Gas Sensors](#gas-sensors)
  - [Connecting the Relays](#connecting-the-relays)
- [Operating Instructions](#operating-instructions)
  - [Start-up, Sleep and Shutdown](#start-up-sleep-and-shutdown)
  - [Button Functions](#button-functions)
  - [Setup](#setup)
- [Hardware](#hardware)
  - [Front Panel Buttons](#front-panel-buttons)
  - [Front Panel OLED Display](#front-panel-oled-display)
  - [External Connectors](#external-connectors)
    - [Micro-USB Programming Port](#micro-usb-programming-port)
    - [1-Wire Bus](#1-wire-bus)
    - [Opto-isolated Inputs](#opto-isolated-inputs)
    - [MQ Gas Sensors ADC Inputs](#mq-gas-sensors-adc-inputs)
    - [Tank ADC Inputs](#tank-adc-inputs)
    - [Opto-isolated Relay Outputs](#opto-isolated-relay-outputs)
    - [Voltage, Current and Power Measurement Inputs](#voltage-current-and-power-measurement-inputs)
    - [NMEA2000 I/O Connector](#nmea2000-io-connector)
    - [NMEA0183 / RS232 I/O Connector](#nmea0183--rs232-io-connector)
    - [RS485 I/O Connector](#rs485-io-connector)
  - [Firmware](#firmware)
- [Resources](#resources)
  - [Assets](#assets)
  - [Datasheets](#datasheets)

# Quick-start Guide

*TODO: Document the `Quick-start Guide` section*

## Unboxing
## Connecting to the NMEA 2000 Bus
## Turning the device on and off

# Installation

*TODO: Document the `Installation` section*

## Connecting to the NMEA 2000 Bus

## Connecting the Sensors
### 1-Wire Temperature Sensors
### Flow meter
### Battery Shunts
### Analog Tank Level Senders
### Bilge Switch
### Gas Sensors

## Connecting the Relays

# Operating Instructions

*TODO: Document the `Operating Instructions` section*

## Start-up, Sleep and Shutdown
## Button Functions
## Setup

# Hardware

## Front Panel Buttons

*TODO: Document the `Front Panel OLED Display` section*

## Front Panel OLED Display

*TODO: Document the `Front Panel OLED Display` section*

## External Connectors

### Micro-USB Programming Port

*TODO: Document the `USB Programming Port` section*

###  1-Wire Bus

A 3-pin terminal block is provided with the pins labled GND, DQ and 5V. 

* `Black`: GND
* `Red`: Regulated 5V for connected sensors
* `Blue`: Data / Signal

The 1-wire input is connected to five (5) [DS18B20 1-wire Thermometer sensors](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ds18b20_1_wire_thermometer.pdf) for monitoring the: 
*   fridge temperature (`PGN 130316: Temperature, Extended Range`);
*   freezer temperature (`PGN 130316: Temperature, Extended Range`);
*   house battery bank 1 temperature (`PGN127508: Battery status`);
*   house battery bank 2 temperature (`PGN127508: Battery status`); and 
*   inverter temperature (`PGN 130316: Temperature, Extended Range`).
   
###  Opto-isolated Inputs

*TODO: Document the `Opto-isolated Inputs` section*


###  MQ Gas Sensors ADC Inputs

*TODO: Document the `MQ Gas Sensors` section*


###  Tank ADC Inputs

*TODO: Document the `Tank ADC Inputs` section*


###  Opto-isolated Relay Outputs

*TODO: Document the `Opto-isolated  Relay Outputs` section*


###  Voltage, Current and Power Measurement Inputs

*TODO: Document the `Voltage, Current and Power Measurement Inputs` section*

###  NMEA2000 I/O Connector

*TODO: Document the `NMEA2000 I/O Connector` section*

###  NMEA0183 / RS232 I/O Connector

*TODO: Document the `NMEA0183 / RS232 I/O Connector` section*

###  RS485 I/O Connector

*TODO: Document the `RS485 I/O Connector` section*

## Firmware

*TODO: Document the `FIRMWARE` section*

# Resources

## Assets

The [_ASSETS](https://github.com/GM-Consult-IOT/MCC400_Firmware/tree/main/_ASSETS) folder contains assets for the MLI400 Marine Legacy Interface:
*   [Circuit schematics for the v2.1 PCB as a PDF](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MC400%20schematic%20v2.1.pdf)
*   [PCB v2.1 Layout and dimensions as a PDF](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MC400%20PCB%20v2.1.pdf)
*   [Workbook with  v2.1 design calculations](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_V2.1%20DESIGN_CALCULATIONS.xlsx)
*   [3D render of the populated v2.1 PCB](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20(1).jpg)
*   [3D render of the populated v2.1 Housing](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20HOUSING%203D%20(1).png)   
*   [3D render of the populated v2.1 Housing](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20HOUSING%203D%20(2).png)   
*   [3D render of the populated v2.1 PCB](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20PCB%203D%20(1).png)   
*   [3D render of the populated v2.1 PCB](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20PCB%203D%20(2).png)   
*   [3D render of the populated v2.1 PCB](https://github.com/GM-Consult-IOT/MCC400_Firmware/blob/main/_ASSETS/MCC400_v2.1%20PCB%203D%20(3).png)   

## Datasheets

Datasheets for the principal hardware components are listed below:
* [4N25 Phototransistor Optocoupler (QT Optoelectronics)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/4n25_optocoupler_qt.pdf)
* [ADS111x I2C compatible ADC (Texas Instruments)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ads1115_adc_ti.pdf)
* [BC807 PNP Transistor (onsemi)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/BC807_pnp_onsemi.pdf)
* [BC817 NPN Transistor (onsemi)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/BC817_npn_onsemi.pdf)
* [BME280 Barometric, Humidity and Temperature Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
* [DS18B20 1-wire Thermometer (Maxim)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ds18b20_1_wire_thermometer.pdf)
* [ESP32-DevKitC V4 (Espressif)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/esp32_devkits_v4_wroom32D_complete.pdf)
* [ESP32-WROVER-B Microcontroller module with Wi-Fi, Bluetooth and BLE (Espressif)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/esp32-wrover-b_espressif.pdf)
* [GY-BMP280-3.3 Pressure Sensor Module](https://startingelectronics.org/pinout/GY-BMP280-pressure-sensor-module/)
* [INA3221 Triple-Channel, High-Side Measurement, Shunt and Bus Voltage Monitor with I2C- and SMBUS-Compatible Interface (Texas Instruments)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ina3221_ti.pdf)
* [INA3221 Triple-Channel Current/Voltage Monitor with I2C Module (Texas Instruments)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ina3221_mod.pdf)
* [MQ2 Gas Sensor Module (Flying Fish)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/mq2-gas-sensor-module.pdf)
* [PS2501 Optically coupled isolator (Renesas)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ps2501_opto_coupler_renesas.pdf)
* [PS2801 Photocoupler (Renesas)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/ps2801_photocoupler_renesas.pdf)
* [RS-232 Transceiver (MaxLinear)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/sp3232_rs232_ttl_maxlinear.pdf)
* [SP3481 RS-485 Transceiver (Sipex)](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/SP3481_SP3485_rs485_transceiver_sipex.pdf)




