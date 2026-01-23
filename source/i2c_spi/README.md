# Talking to the World - Working with I2C and SPI

## Course Overview

A comprehensive course on I2C and SPI communication protocols for Raspberry Pi Pico with MicroPython. Part of the "MicroPython Projects: From Code to Creation" video series.

## Course Structure

### Introduction
- `00_intro.md` - Course introduction and overview

### Understanding I2C (Lessons 1-4)
- `01_what-is-i2c.md` - I2C protocol fundamentals
- `02_i2c-hardware-setup.md` - Wiring and pull-up resistors
- `03_scanning-i2c-devices.md` - Device discovery and addressing
- `04_reading-i2c-sensor.md` - Reading BME280 sensor data

### Working with Datasheets (Lessons 5-6)
- `05_demystifying-datasheets.md` - How to read technical documentation
- `06_bme280-deep-dive.md` - Advanced BME280 features and optimization

### Understanding SPI (Lessons 7-9)
- `07_what-is-spi.md` - SPI protocol fundamentals
- `08_spi-hardware-setup.md` - Wiring SPI devices
- `09_spi-sensor-example.md` - Reading MAX31855 thermocouple

### Choosing and Troubleshooting (Lessons 10-12)
- `10_i2c-vs-spi.md` - Protocol comparison and decision-making
- `11_common-issues.md` - Troubleshooting guide
- `12_final-project.md` - Environmental monitoring station capstone

## Hardware Requirements

### Essential
- Raspberry Pi Pico (or Pico W)
- BME280 sensor module (I2C)
- 128x64 OLED display (I2C, SSD1306)
- Breadboard and jumper wires

### Optional
- MAX31855 thermocouple amplifier (SPI)
- K-type thermocouple
- SD card module (SPI)
- DS3231 real-time clock (I2C)

## Learning Outcomes

After completing this course, students will be able to:
- Wire and communicate with I2C devices
- Wire and communicate with SPI devices
- Read and interpret datasheets
- Scan for I2C devices and understand addressing
- Choose the appropriate protocol for projects
- Build robust sensor systems with error handling
- Create complete IoT monitoring projects

## Target Audience

- Beginners with basic MicroPython knowledge
- Makers wanting to connect sensors and displays
- Students learning embedded communication protocols
- Anyone building Raspberry Pi Pico projects

## Course Format

- **Lesson length**: 400-1500 words per lesson
- **Coding style**: Working examples with detailed explanations
- **Approach**: Hands-on, practical, project-based
- **Tone**: Friendly and encouraging, maker-focused

## Related Courses

- Introduction to MicroPython
- MicroPython GPIO
- Raspberry Pi Pico Projects
- IoT Weather Station (capstone project)

## Author

Kevin McAleer
kevinmcaleer@gmail.com
https://www.kevsrobots.com

## Date Created

January 15, 2026

## Course Assets

Images should be placed in `assets/` directory:
- `cover.jpg` - Course cover image
- `i2c-diagram.jpg` - I2C communication illustration
- `i2c-wiring.jpg` - BME280 wiring example
- `i2c-scanner.jpg` - I2C scanning output
- `bme280-reading.jpg` - Sensor reading example
- `datasheet.jpg` - Datasheet example
- `bme280-advanced.jpg` - Advanced features
- `spi-diagram.jpg` - SPI communication illustration
- `spi-wiring.jpg` - SPI device wiring
- `max31855-monitor.jpg` - Temperature monitoring
- `i2c-vs-spi.jpg` - Protocol comparison
- `troubleshooting.jpg` - Debugging illustration
- `final-project.jpg` - Complete monitoring station

## Notes for Course Maintenance

- Test all code examples with latest MicroPython firmware
- Update library installation instructions if packages change
- Verify all datasheet links remain valid
- Check that device addresses match current hardware
- Update project enhancements with new sensor options
