# Air monitor

This is an ESP32 based project to measure air quality in terms of CO2 concentration, temperature and humidity. It is especially useful in well-isolated houses without ventilation to indicate when you need to open the window to get some fresh air.

## Features
* Regularly measures CO2, temperature and humidity
* Visualizes air quality with the on-board 3 colour LED in a green yellow red scale
* Optionally publishes measurements to MQTT via WiFI
* Optionally publishes measurements via Bluetooth low energy (BLE) GATT

## Hardware
* [ESP32 C3 dev module](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html)
* [Sensirion SCD30 breakout board](https://wiki.seeedstudio.com/Grove-CO2_Temperature_Humidity_Sensor-SCD30/)

## Wiring
* SCD30 yellow cable (SCL) to ESP32 GPIO3
* SCD30 white cable (SDA) to ESP32 GPIO2
* SCD30 red cable (VCC) to ESP32 3V3
* SCD30 black cable (GND) to ESP32 GND

# Software installation
* Install ESP32 board support in Arduino [as described in Espressif's documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
* Install the necessary libraries in Arduino:
    * [Sensirion SCD30 library](https://github.com/Sensirion/arduino-i2c-scd30.git) and its dependency [Sensirion Arduino Core Library](https://github.com/Sensirion/arduino-core.git)
    * [Adafruit Neopixel library](https://github.com/adafruit/Adafruit_NeoPixel) to drive the (built-in) 3 colour LED
    * [Arduino MQTT client library](https://github.com/knolleary/PubSubClient) and [NTP library](https://github.com/sstaub/NTP) if you want to send data to MQTT
* Select the features you want to use at the top of the main source file under feature selection.
* If you use MQTT, fill in your WiFi and MQTT data in `mqtt-settings.h`.
* Select the board ESP32 C3 Dev Module in Arduino
* Compile and flash.
