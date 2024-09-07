# MCP2515 OBD2 Data Reader for AVR

A C library for reading data from a car's engine using the OBD2 protocol through the MCP2515 CAN controller. Designed for AVR microcontrollers, specifically the ATmega328P, this library allows you to access vital engine parameters such as RPM, speed, oil temperature, and more.

## Features

- Communicate with a car's OBD2 system via the MCP2515 CAN controller.
- Read various engine parameters like:
  - RPM (Revolutions Per Minute)
  - Vehicle Speed
  - Engine Oil Temperature
  - Coolant Temperature
  - Throttle Position
  - Fuel Level
  - All codes can be found [here](https://en.wikipedia.org/wiki/OBD-II_PIDs).

## Problems

- CAN bus speed values are hardcoded
