# Air monitor

This is an ESP32 based project to measure air quality in terms of CO2 concentration, temperature and humidity. It is especially useful in well-isolated houses without ventilation to indicate when you need to open the window to get some fresh air.

## Features

* Regularly measures CO2, temperature and humidity
* Visualizes air quality with the on-board 3 colour LED in a green yellow red scale
* Optionally publishes measurements to MQTT via WiFi
* Optionally publishes measurements via Bluetooth low energy (BLE) GATT


## Hardware

* [ESP32 C3 dev module](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html)
* A CO2 sensor, one of
    * [Sensirion SCD30 breakout board](https://wiki.seeedstudio.com/Grove-CO2_Temperature_Humidity_Sensor-SCD30/)
    * [Sensirion SCD41 breakout board](https://wiki.seeedstudio.com/Grove-CO2_&_Temperature_&_Humidity_Sensor-SCD41/)
    * [Winsen MH-Z19](https://www.winsen-sensor.com/product/mh-z19c.html)


## Wiring

If you use a Sensirion I2C sensor:

* SCDxx yellow cable (SCL) to ESP32 GPIO3
* SCDxx white cable (SDA) to ESP32 GPIO2
* SCDxx red cable (VCC) to ESP32 3V3
* SCDxx black cable (GND) to ESP32 GND

If you use the serial MH-Z19 sensor:

* MH-Z19 Vin to ESP32 5V
* MH-Z19 GND to ESP32 GND
* MH-Z19 Rx to ESP32 TX
* MH-Z19 Tx to ESP32 RX


# Software installation

* Install ESP32 board support in Arduino [as described in Espressif's documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
* Install the necessary libraries in Arduino:
    * [Sensirion SCD30 library](https://github.com/Sensirion/arduino-i2c-scd30) and its dependency [Sensirion Arduino Core Library](https://github.com/Sensirion/arduino-core), if you use this sensor
    * [Sensirion SCD4x library](https://github.com/Sensirion/arduino-i2c-scd4x) and its dependency [Sensirion Arduino Core Library](https://github.com/Sensirion/arduino-core), if you use this sensor
    * Jonathan Dempsey's [MH-Z19 library](https://github.com/WifWaf/MH-Z19), if you use this sensor
    * [Adafruit Neopixel library](https://github.com/adafruit/Adafruit_NeoPixel) to drive the (built-in) 3 colour LED
    * [Arduino MQTT client library](https://github.com/knolleary/PubSubClient) and [NTP library](https://github.com/sstaub/NTP) if you want to send data to MQTT
* Select the features you want to use at the top of the main source file under feature selection.
* If you use MQTT, fill in your WiFi and MQTT data in `mqtt-settings.h`.
* Select the board ESP32 C3 Dev Module in Arduino
* Compile and flash.
