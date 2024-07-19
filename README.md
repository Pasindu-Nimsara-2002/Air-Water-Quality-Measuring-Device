# Air and Water Quality Measuring Device

## Overview

The Air and Water Quality Measuring Device is designed to measure various environmental parameters including air quality and water quality. It uses a combination of sensors to measure temperature, humidity, pH, electrical conductivity, and concentrations of gases like LPG, CO, CH4, NH3, and NOx.

## Features

- **Air Quality Measurement**:
  - MQ-9 Sensor for detecting LPG, CO, and CH4.
  - MQ-135 Sensor for detecting NH3 and NOx.
  - DHT11 Sensor for measuring temperature and humidity.

- **Water Quality Measurement**:
  - pH Sensor for measuring the pH level of the water.
  - Conductivity Sensor for measuring the electrical conductivity (EC) and Total Dissolved Solids (TDS).
  - DS18B20 Sensor for measuring water temperature.

## Hardware Components

- **Microcontroller**: AVR ATmega microcontroller
- **Sensors**:
  - MQ-9 Gas Sensor
  - MQ-135 Gas Sensor
  - DHT11 Temperature and Humidity Sensor
  - pH Sensor
  - Conductivity Sensor
  - DS18B20 Temperature Sensor

## Software

The device firmware is written in C and runs on the AVR microcontroller. It handles sensor data acquisition, data processing, and communication via UART.

### Libraries

- **DHT Library**: For reading from the DHT11 sensor.
- **DS18B20 Library**: For reading from the DS18B20 temperature sensor.

## Installation

1. **Hardware Setup**:
   - Connect all sensors to the microcontroller according to the pin definitions in the code.
   - Ensure proper power supply and connections for all sensors and the microcontroller.

2. **Software Setup**:
   - Install the necessary AVR toolchain on your computer.
   - Compile the provided source code using the toolchain.

3. **Flashing the Firmware**:
   - Upload the compiled firmware to the microcontroller using an AVR programmer.

## Code Structure

- **`main.c`**: Main program file containing setup and loop functions.
- **`DHT.c`**: Library for DHT11 sensor interaction.
- **`ds18b20`**: Library for DS18B20 sensor interaction.

## Usage

1. **Power on the Device**:
   - Ensure the device is properly powered.

2. **Navigate the Menu**:
   - Use the push buttons to navigate through the menu:
     - **UP**: Move to the next menu item.
     - **DOWN**: Move to the previous menu item.
     - **OK**: Select the current menu item.
     - **CANCEL**: Exit the current menu.

3. **Measure Air Quality**:
   - Navigate to the Air Quality menu to select between MQ-9, MQ-135, or Temperature readings.

4. **Measure Water Quality**:
   - Navigate to the Water Quality menu to select between pH, Conductivity, or Water Temperature readings.

## Troubleshooting

- **No Data Displayed**: Check sensor connections and ensure sensors are powered correctly.
- **Inaccurate Readings**: Ensure proper calibration of sensors and check for environmental factors affecting measurements.

## Acknowledgements

- The DHT11 and DS18B20 libraries used in this project are developed by their respective authors.
- Special thanks to the community for open-source libraries and tools used.

---

