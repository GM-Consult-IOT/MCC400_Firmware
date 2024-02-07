# MCC400 MARINE CABIN CONTROLLER

A marine cabin systems and environment controller/supervisor.


## Contents

## Product Description

The MCC400 Marine Cabin Controller monitors vessel cabin/house systems. It collects data and transmits this data on the vessel's NMEA2000 (N2K) bus. The MCC400 monitors:
* Fridge, freezer, battery and inverter temperatures using [DS18B20 1-wire Thermometer sensors](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) connected to the 1-wire bus.
* Barometric pressure, cabin temperature and relative humidity using a [Bosch BME820 Environmental Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) on the controller board.
* A bilge water level switch via an opto-isolated input.
* Fresh water consumption using a hall effect flow meter at the input to the fresh water pump, connected to an opto-isolated input.
* Up to four tank level senders connected to 14v tolerant ADC inputs.
* Up to four MQ gas sensors connected to 5v ADC inputs.
* Three voltage / current / power channels using external shunt resistors. These are usually conntected to the house battery bank, the cranking battery and the PV regulator.

The MCC400 also has two serial ports: one RS232 (RX/TX) and one RS485. These can be connected to other serial devices, e.g. a GPS or a solar charge controller.

## External Connectors

### 1-Wire Bus

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
