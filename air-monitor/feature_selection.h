#pragma once

/* Feature selection */
#define SERIAL_OUTPUT
//#define HAS_SCD30
#define HAS_SCD4X
//#define HAS_MHZ19
#define HAS_NEOPIXEL
#define HAS_4DIGIT
#define HAS_LDR
#define USE_MQTT
//#define USE_BLE

/* Pins (GPIOs) for ESP32-C3-DevKitM-1 */
#define PIN_SCL 3 // yellow cable of SCDxx
#define PIN_SDA 2 // white cable of SCDxx
#define PIN_RGB_LED 8 // built-in rgb LED
#define MHZ19_SERIAL_PORT 0 // serial port to use for MH-Z19
#define TM1637_CLK 7
#define TM1637_DIO 6
#define ADC_PIN 1

