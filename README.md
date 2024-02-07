**MCC400 MARINE CABIN CONTROLLER**

A marine cabin systems and environment controller/supervisor.

**Contents**
- [Product Description](#product-description)
- [External Connectors](#external-connectors)
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
- [Assets](#assets)


# Product Description

The MCC400 Marine Cabin Controller monitors vessel cabin/house systems. It collects data and transmits this data on the vessel's NMEA2000 (N2K) bus. The MCC400 monitors:
* Fridge, freezer, battery and inverter temperatures using [DS18B20 1-wire Thermometer sensors](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) connected to the 1-wire bus.
* Barometric pressure, cabin temperature and relative humidity using a [Bosch BME820 Environmental Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) on the controller board.
* A bilge water level switch via an opto-isolated input.
* Fresh water consumption using a hall effect flow meter at the input to the fresh water pump, connected to an opto-isolated input.
* Up to four tank level senders connected to 14v tolerant ADC inputs.
* Up to four MQ gas sensors connected to 5v ADC inputs.
* Three voltage / current / power channels using external shunt resistors. These are usually conntected to the house battery bank, the cranking battery and the PV regulator.

The MCC400 also has two serial ports: one RS232 (RX/TX) and one RS485. These can be connected to other serial devices, e.g. a GPS or a solar charge controller.

# External Connectors

## 1-Wire Bus

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
   
## Opto-isolated Inputs

*TODO: Document the `Opto-isolated Inputs` section*


## MQ Gas Sensors ADC Inputs

*TODO: Document the `MQ Gas Sensors` section*


## Tank ADC Inputs

*TODO: Document the `Tank ADC Inputs` section*


## Opto-isolated Relay Outputs

*TODO: Document the `Opto-isolated  Relay Outputs` section*


## Voltage, Current and Power Measurement Inputs

*TODO: Document the `Voltage, Current and Power Measurement Inputs` section*

## NMEA2000 I/O Connector

*TODO: Document the `NMEA2000 I/O Connector` section*

## NMEA0183 / RS232 I/O Connector

*TODO: Document the `NMEA0183 / RS232 I/O Connector` section*

## RS485 I/O Connector

*TODO: Document the `RS485 I/O Connector` section*

# Firmware

*TODO: Document the `FIRMWARE` section*


# Assets

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
