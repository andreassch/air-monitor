/**
 * Air quality monitor with ESP32
 *
 * Use one of these supported CO2 sensors: 
 * - Sensirion SCD30: https://github.com/Sensirion/arduino-i2c-scd30
 * - Sensirion SCD4x: https://github.com/Sensirion/arduino-i2c-scd4x
 * - MH-Z19: https://github.com/WifWaf/MH-Z19
 *
 * Further libraries:
 * For Neopixel:
 * - https://github.com/adafruit/Adafruit_NeoPixel
 * For 4 digit 7-segment display:
 * - https://github.com/avishorp/TM1637
 * For 16x2 LCD:
 * - https://github.com/sstaub/LCD-I2C-HD44780
 * For MQTT:
 * - https://github.com/knolleary/PubSubClient
 * - https://github.com/bblanchon/ArduinoJson if reception from MQTT is desired
 *
 * If sketch does not fit, select partition scheme "Minimal SPIFFS" or similar.
 */

/* Feature selection for Arduino IDE */
#ifndef PLATFORMIO
#include "feature_selection.h"
#endif

/* Includes */
#include <Arduino.h>
#include <Wire.h>
#ifdef HAS_SCD30
#include <SensirionI2cScd30.h>
#endif
#ifdef HAS_SCD4X
#include <SensirionI2CScd4x.h>
#endif
#ifdef HAS_MHZ19
#include <MHZ19.h>
#endif
#ifdef HAS_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif
#if defined(HAS_NEOPIXEL) || defined(RGB_BUILTIN)
#include "colormap.h"
#endif
#ifdef HAS_4DIGIT
#include <TM1637Display.h>
#endif
#ifdef HAS_LCD
#include <LCDi2c.h>
#endif
#ifdef USE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include <ctime>
#include "mqtt_settings.h"
#ifdef MQTT_TOPIC_RECEIVE
#include <ArduinoJson.h>
#endif
#endif
#ifdef USE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#endif
#include <limits.h>

/* Typedefs */
struct measurement_t
{
  time_t time = 0;
  uint16_t co2 = 0;
  float temperature = NAN;
  float humidity = NAN;
};

/* Macros */
#ifdef SERIAL_OUTPUT
#define SERIAL_PRINT(x) Serial.print(x)
#define SERIAL_PRINTLN(x) Serial.println(x)
#define SERIAL_PRINT_HEX(x) Serial.print(x, HEX)
#else
#define SERIAL_PRINT(x)
#define SERIAL_PRINTLN(x)
#define SERIAL_PRINT_HEX(x)
#endif

/* Constants */
#ifdef SERIAL_OUTPUT
#define SERIAL_OUTPUT_SPEED 9600
#endif
#ifdef HAS_MHZ19
#define MHZ19_BAUDRATE 9600
#endif
#ifdef HAS_NEOPIXEL
#define NEOPIXEL_PIN PIN_RGB_LED
#define NEOPIXEL_NUM 1
#endif
#if defined(HAS_NEOPIXEL) || defined(RGB_BUILTIN)
#define NEOPIXEL_MAX_BRIGHTNESS_FACTOR 128
#endif
#ifdef HAS_4DIGIT
const uint8_t SEG_HI[] = {
        0,                                       // _
        SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,   // H
        SEG_B | SEG_C,                           // I
        0                                        // _
    };
#endif
#ifdef HAS_LCD
#define LCD_I2C_ADDRESS 0x27
#define LCD_WIDTH 16
#define LCD_HEIGHT 2
#define DISPLAY_BUF_LEN 17
#define LCD_DEGREE_SIGN 0xDF
#define LCD_USER_PERCENT_SIGN 0
uint8_t lcd_user_percent_sign[8] = {
  0b11000,
  0b11001,
  0b00010,
  0b00100,
  0b01000,
  0b10011,
  0b00011,
};
#define LCD_PERCENT_SIGN (0x08 + LCD_USER_PERCENT_SIGN)
#define LCD_USER_SUMMERTIME_SIGN 1
uint8_t lcd_user_summertime_sign[8] = {
  0b11111,
  0b11001,
  0b10111,
  0b10001,
  0b11101,
  0b10011,
  0b11111,
};
#define LCD_SUMMERTIME_SIGN (0x08 + LCD_USER_SUMMERTIME_SIGN)
#endif
#ifdef USE_MQTT
#define TIME_BUF_LEN 30
#endif
#ifdef USE_BLE
#define BLE_NAME "ESP32 Air Monitor"
#define BLE_SERVICE_UUID BLEUUID((uint16_t)0x181A)
#define BLE_UUID_CO2 BLEUUID((uint16_t)0x2B8C)
#define BLE_UUID_TEMPERATURE BLEUUID((uint16_t)0x2A6E)
#define BLE_UUID_HUMIDITY BLEUUID((uint16_t)0x2A6F)
#define BLE_DESCRIPTOR BLEUUID((uint16_t)0x2901)
#endif

/* Global variables */
#if !defined(HAS_SCD30) && !defined(HAS_SCD4X) && !defined(HAS_MHZ19)
static uint16_t loop_counter = 0;
#endif
#ifdef HAS_SCD30
SensirionI2cScd30 scd30;
#endif
#ifdef HAS_SCD4X
SensirionI2CScd4x scd4x;
#endif
#if defined(HAS_SCD30) || defined(HAS_SCD4X)
#define ERRMSGLEN 256
static char error_message[ERRMSGLEN];
static int16_t error;
#endif
#ifdef HAS_MHZ19
HardwareSerial serial_port(MHZ19_SERIAL_PORT);
MHZ19 mhz19;
#endif
#ifdef HAS_NEOPIXEL
Adafruit_NeoPixel neopixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif
#if defined(HAS_NEOPIXEL) || defined(RGB_BUILTIN)
Colormap colormap(600.0, 1200.0);
#endif
#ifdef HAS_4DIGIT
TM1637Display digits(TM1637_CLK, TM1637_DIO);
#endif
#ifdef HAS_LCD
LCDi2c lcd(LCD_I2C_ADDRESS, Wire);
time_t last_display_time = 0;
#endif
#ifdef USE_MQTT
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
WiFiUDP wifiudp;
static unsigned long time_last_pub = 0;
#define MSG_BUFFER_SIZE	(200)
char mqtt_topic[MSG_BUFFER_SIZE];
char mqtt_msg[MSG_BUFFER_SIZE];
#ifdef MQTT_TOPIC_RECEIVE
measurement_t mqtt_data;
#endif
#endif
#ifdef USE_BLE
BLECharacteristic* co2_characteristic = nullptr;
BLECharacteristic* temperature_characteristic = nullptr;
BLECharacteristic* humidity_characteristic = nullptr;
#endif

/* Function definitions */
#ifdef HAS_SCD4X
void printUint16Hex(uint16_t value)
{
  SERIAL_PRINT(value < 4096 ? "0" : "");
  SERIAL_PRINT(value < 256 ? "0" : "");
  SERIAL_PRINT(value < 16 ? "0" : "");
  SERIAL_PRINT_HEX(value);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2)
{
  SERIAL_PRINT("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  SERIAL_PRINTLN();
}
#endif

#ifdef USE_MQTT
void setupWifi()
{
  delay(10);
  // Start by connecting to a WiFi network.
  SERIAL_PRINTLN();
  SERIAL_PRINT("Connecting to ");
  SERIAL_PRINTLN(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    SERIAL_PRINT(".");
  }

  randomSeed(micros());

  SERIAL_PRINTLN();
  SERIAL_PRINTLN("WiFi connected");
  SERIAL_PRINTLN("IP address: ");
  SERIAL_PRINTLN(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length)
{
  SERIAL_PRINT("Message arrived [");
  SERIAL_PRINT(topic);
  SERIAL_PRINT("] ");
  for (int i = 0; i < length; i++)
  {
    SERIAL_PRINT((char)payload[i]);
  }
  SERIAL_PRINTLN();

#ifdef MQTT_TOPIC_RECEIVE
  if (!strcmp(topic, MQTT_TOPIC_PREFIX "/" MQTT_TOPIC_RECEIVE))
  {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else
    {
      mqtt_data.time = time(nullptr);
      mqtt_data.co2 = doc["co2"].as<uint16_t>();
      mqtt_data.temperature = doc["temperature"].as<float>();
      mqtt_data.humidity = doc["humidity"].as<float>();
    }
  }
#endif
}

void reconnect() {
  // Loop until reconnected
  for (int tries = 0; !mqtt_client.connected() && tries < 5; tries++)
  {
    SERIAL_PRINT("Attempting MQTT connection ...");
    // Create a random client ID.
    String client_id = "ESP32-Air-Monitor-";
    client_id += String(random(0xffff), HEX);
    // Attempt to connect.
    if (mqtt_client.connect(client_id.c_str()))
    {
      SERIAL_PRINTLN("connected");
      // Subscribe.
#ifdef MQTT_TOPIC_RECEIVE
      mqtt_client.subscribe(MQTT_TOPIC_PREFIX "/" MQTT_TOPIC_RECEIVE);
#endif
    }
    else
    {
      SERIAL_PRINT("failed, rc=");
      SERIAL_PRINT(mqtt_client.state());
      SERIAL_PRINTLN(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishData(const char* name, const measurement_t data)
{
  struct tm timeinfo;
  localtime_r(&data.time, &timeinfo);
  char time_buf[TIME_BUF_LEN];
  strftime(time_buf, TIME_BUF_LEN, "%d.%m.%Y %H:%M:%S", &timeinfo);
  snprintf(mqtt_topic, MSG_BUFFER_SIZE, "%s/%s/%s", MQTT_TOPIC_PREFIX, MQTT_TOPIC_MEASUREMENT, name);
  if (isnan(data.humidity))
    snprintf(mqtt_msg, MSG_BUFFER_SIZE, "{\"time\": \"%s\", \"co2\": %u, \"temperature\": %.0f}", time_buf, data.co2, data.temperature);
  else
    snprintf(mqtt_msg, MSG_BUFFER_SIZE, "{\"time\": \"%s\", \"co2\": %u, \"temperature\": %.1f, \"humidity\": %.1f}", time_buf, data.co2, data.temperature, data.humidity);
  mqtt_client.publish(mqtt_topic, mqtt_msg);
}
#endif

#ifdef HAS_LCD
void displayData(measurement_t data)
{
  char display_buf[DISPLAY_BUF_LEN];
  if (isnan(data.humidity))
    snprintf(display_buf, DISPLAY_BUF_LEN, "%2.1f%cC  %5dppm", data.temperature, LCD_DEGREE_SIGN, data.co2);
  else
    snprintf(display_buf, DISPLAY_BUF_LEN, "%2.0f%cC%3.0f%c%5dppm", data.temperature, LCD_DEGREE_SIGN, data.humidity, LCD_PERCENT_SIGN, data.co2);
  lcd.locate(2, 1);
  lcd.printf(display_buf);
}

#if defined(USE_MQTT) // time is only available if WiFi is connected
void displayTime(time_t cur_time)
{
  struct tm timeinfo;
  localtime_r(&cur_time, &timeinfo);
  char display_buf[DISPLAY_BUF_LEN];
  strftime(display_buf, DISPLAY_BUF_LEN, "%H:%M:%S %d.%m.", &timeinfo);
  lcd.locate(1, 1);
  lcd.printf(display_buf);
  if (timeinfo.tm_isdst)
  {
    lcd.character(1, 16, LCD_USER_SUMMERTIME_SIGN);
  }
}

void updateDisplayTime()
{
  time_t cur_time = time(nullptr);
  if (cur_time != last_display_time)
  {
    displayTime(cur_time);
    last_display_time = cur_time;
  }
}
#endif
#endif


/** Setup function */
void setup()
{
#ifdef SERIAL_OUTPUT
  Serial.begin(SERIAL_OUTPUT_SPEED);
  for (int i = 0; !Serial && i < 50; i++)
  {
    delay(100);
  }
#endif

#if defined(HAS_SCD30) || defined(HAS_SCD4X) || defined(HAS_LCD)
  Wire.begin(PIN_SDA, PIN_SCL);
#endif
#ifdef HAS_SCD30
  scd30.begin(Wire, SCD30_I2C_ADDR_61);
#endif
#ifdef HAS_SCD4X
  scd4x.begin(Wire);

  error = scd4x.stopPeriodicMeasurement();
  if (error)
  {
    SERIAL_PRINT("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, error_message, sizeof error_message);
    SERIAL_PRINTLN(error_message);
  }
  uint16_t serial0, serial1, serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error)
  {
    SERIAL_PRINT("Error trying to execute getSerialNumber(): ");
    errorToString(error, error_message, sizeof error_message);
    SERIAL_PRINTLN(error_message);
  }
  else
  {
    printSerialNumber(serial0, serial1, serial2);
  }
  error = scd4x.startPeriodicMeasurement();
  if (error)
  {
    SERIAL_PRINT("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, error_message, sizeof error_message);
    SERIAL_PRINTLN(error_message);
  }
#endif

#ifdef HAS_MHZ19
  serial_port.begin(MHZ19_BAUDRATE);
  mhz19.begin(serial_port);
  mhz19.autoCalibration();
#endif

#ifdef HAS_NEOPIXEL
  neopixels.begin();
#endif

#ifdef HAS_4DIGIT
  digits.setBrightness(0x0f);
#endif

#ifdef HAS_LCD
  lcd.begin(LCD_HEIGHT, LCD_WIDTH);
  lcd.create(LCD_USER_PERCENT_SIGN, lcd_user_percent_sign);
  lcd.create(LCD_USER_SUMMERTIME_SIGN, lcd_user_summertime_sign);
  lcd.cls();
  lcd.display(BACKLIGHT_ON);
  lcd.locate(1, 3);
  lcd.printf("Air monitor");
  lcd.locate(2, 1);
  lcd.printf("Initializing ...");
#endif

#ifdef USE_MQTT
  setupWifi();
  configTzTime(TIME_ZONE, NTP_SERVER);
  mqtt_client.setServer(MQTT_SERVER, 1883);
  mqtt_client.setCallback(callback);
#endif

#ifdef USE_BLE
  SERIAL_PRINTLN("Starting BLE ...");
  BLEDevice::init(BLE_NAME);
  BLEServer* server = BLEDevice::createServer();
  BLEService* service = server->createService(BLE_SERVICE_UUID);
  co2_characteristic = service->createCharacteristic(
      BLE_UUID_CO2,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  temperature_characteristic = service->createCharacteristic(
      BLE_UUID_TEMPERATURE,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  humidity_characteristic = service->createCharacteristic(
      BLE_UUID_HUMIDITY,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  co2_characteristic->addDescriptor(new BLE2902());
  temperature_characteristic->addDescriptor(new BLE2902());
  humidity_characteristic->addDescriptor(new BLE2902());
  BLEDescriptor co2_descriptor(BLE_DESCRIPTOR);
  BLEDescriptor temperature_descriptor(BLE_DESCRIPTOR);
  BLEDescriptor humidity_descriptor(BLE_DESCRIPTOR);
  co2_descriptor.setValue("CO2 concentration in ppm");
  temperature_descriptor.setValue("Temperature in °C");
  humidity_descriptor.setValue("Humidity in %");
  co2_characteristic->addDescriptor(&co2_descriptor);
  temperature_characteristic->addDescriptor(&temperature_descriptor);
  humidity_characteristic->addDescriptor(&humidity_descriptor);
  service->addCharacteristic(co2_characteristic);
  service->addCharacteristic(temperature_characteristic);
  service->addCharacteristic(humidity_characteristic);
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  SERIAL_PRINTLN("BLE characteristic defined! Now you can read it in your phone!");
#endif

#ifdef HAS_LCD
  lcd.cls();
#endif
}

/** Loop function */
void loop()
{
#if !defined(HAS_SCD30) && !defined(HAS_SCD4X) && !defined(HAS_MHZ19)
  loop_counter++;
  SERIAL_PRINT("Loop ");
  SERIAL_PRINTLN(loop_counter);
#endif

#if defined(HAS_LCD) && defined(USE_MQTT) && !defined(HAS_SCD4X) // time is only available if WiFi is connected, update during SCD4X measurement is handled below
  updateDisplayTime();
#endif

#ifdef HAS_SCD30
  measurement_t scd30_data;
  scd30_data.time = time(nullptr);
  float scd30_co2 = NAN;
  error = scd30.blockingReadMeasurementData(scd30_co2, scd30_data.temperature, scd30_data.humidity);
  if (error != NO_ERROR) {
    SERIAL_PRINT("Error trying to execute blockingReadMeasurementData(): ");
    errorToString(error, error_message, sizeof error_message);
    SERIAL_PRINTLN(error_message);
  }
  else
  {
    scd30_data.co2 = static_cast<uint16_t>(scd30_co2);
    SERIAL_PRINT("CO2 concentration: ");
    SERIAL_PRINT(scd30_co2);
    SERIAL_PRINT("ppm\ttemperature: ");
    SERIAL_PRINT(scd30_data.temperature);
    SERIAL_PRINT("°C\thumidity: ");
    SERIAL_PRINT(scd30_data.humidity);
    SERIAL_PRINTLN("%");
  }
#endif
#ifdef HAS_SCD4X
  measurement_t scd4x_data;
  bool is_data_ready = false;
  while (!is_data_ready)
  {
#if defined(HAS_LCD) && defined(USE_MQTT) // time is only available if WiFi is connected
    updateDisplayTime();
#endif
    delay(50);
    error = scd4x.getDataReadyFlag(is_data_ready);
    if (error)
    {
      SERIAL_PRINT("Error trying to execute getDataReadyFlag(): ");
      errorToString(error, error_message, sizeof error_message);
      SERIAL_PRINTLN(error_message);
      return;
    }
  }
  if (is_data_ready)
  {
    scd4x_data.time = time(nullptr);
    error = scd4x.readMeasurement(scd4x_data.co2, scd4x_data.temperature, scd4x_data.humidity);
    if (error)
    {
      SERIAL_PRINT("Error trying to execute readMeasurement(): ");
      errorToString(error, error_message, 256);
      SERIAL_PRINTLN(error_message);
    }
    else if (scd4x_data.co2 == 0)
    {
      SERIAL_PRINTLN("Invalid sample detected, skipping.");
    }
    else
    {
      SERIAL_PRINT("CO2: ");
      SERIAL_PRINT(scd4x_data.co2);
      SERIAL_PRINT("ppm\tTemperature: ");
      SERIAL_PRINT(scd4x_data.temperature);
      SERIAL_PRINT("°C\tHumidity: ");
      SERIAL_PRINT(scd4x_data.humidity);
      SERIAL_PRINTLN("%");
    }
  }
#endif
#ifdef HAS_MHZ19
  measurement_t mhz19_data;
  mhz19_data.time = time(nullptr);
  int mhz19_co2 = mhz19.getCO2();
  int8_t temp = mhz19.getTemperature();
  SERIAL_PRINT("CO2: ");
  SERIAL_PRINT(mhz19_co2);
  SERIAL_PRINT("ppm\t");
  SERIAL_PRINT("Temperature: ");
  SERIAL_PRINT(temp);
  SERIAL_PRINTLN("°C");
  mhz19_data.co2 = static_cast<uint16_t>(mhz19_co2);
  mhz19_data.temperature = static_cast<float>(temp);
  mhz19_data.humidity = NAN;
#endif
  uint16_t co2_concentration = 0;
  float temperature = NAN;
  float humidity = NAN;
#if defined(HAS_SCD30)
  co2_concentration = scd30_data.co2;
  temperature = scd30_data.temperature;
  humidity = scd30_data.humidity;
#elif defined(HAS_SCD4X)
  co2_concentration = scd4x_data.co2;
  temperature = scd4x_data.temperature;
  humidity = scd4x_data.humidity;
#elif defined(HAS_MHZ19)
  co2_concentration = mhz19_data.co2;
  temperature = mhz19_data.temperature;
#elif defined(MQTT_TOPIC_RECEIVE)
  co2_concentration = mqtt_data.co2;
  temperature = mqtt_data.temperature;
  humidity = mqtt_data.humidity;
#else
  co2_concentration = 400 + static_cast<uint16_t>(loop_counter); // test value
#endif

#ifdef USE_MQTT
  if (!mqtt_client.connected())
    reconnect();
  if (mqtt_client.connected())
  {
    mqtt_client.loop();

    long cur_time = millis();
    if ( ((cur_time - time_last_pub >= MQTT_PUB_INTERVAL) || (cur_time < time_last_pub)) && (!(co2_concentration == 0)) )
    {
      time_last_pub = cur_time;
#ifdef HAS_SCD30
      publishData("scd30", scd30_data);
#endif
#ifdef HAS_SCD4X
      publishData("scd4x", scd4x_data);
#endif
#ifdef HAS_MHZ19
      publishData("mh-z19", mhz19_data);
#endif
    }
  }
  else
  {
    SERIAL_PRINTLN("No MQTT connection!");
  }
#endif

#ifdef HAS_LDR
  uint16_t brightness = analogRead(ADC_PIN);
  SERIAL_PRINT(F("Brightness "));
  SERIAL_PRINTLN(brightness);
#endif

#if defined(HAS_NEOPIXEL) || defined(RGB_BUILTIN)
  rgb_t color = colormap.color(co2_concentration);
  SERIAL_PRINT("Value ");
  SERIAL_PRINT(co2_concentration);
  SERIAL_PRINT(" -> ");
  SERIAL_PRINT(color.red);
  SERIAL_PRINT(", ");
  SERIAL_PRINT(color.green);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(color.blue);
#ifdef HAS_LDR
  uint16_t brightness_factor = (brightness == 0) ? NEOPIXEL_MAX_BRIGHTNESS_FACTOR : 4096 / brightness;
  if (brightness_factor == 0)
    brightness_factor = 1;
  else if (brightness_factor > NEOPIXEL_MAX_BRIGHTNESS_FACTOR)
    brightness_factor = NEOPIXEL_MAX_BRIGHTNESS_FACTOR;
  SERIAL_PRINT(F("Brightness factor "));
  SERIAL_PRINT(brightness_factor);
  color.red /= static_cast<uint8_t>(brightness_factor);
  color.green /= static_cast<uint8_t>(brightness_factor);
  color.blue /= static_cast<uint8_t>(brightness_factor);
  SERIAL_PRINT(F(" -> "));
  SERIAL_PRINT(color.red);
  SERIAL_PRINT(F(", "));
  SERIAL_PRINT(color.green);
  SERIAL_PRINT(F(", "));
  SERIAL_PRINTLN(color.blue);
#endif
#ifdef HAS_NEOPIXEL
  neopixels.setPixelColor(0, neopixels.Color(color.red, color.green, color.blue));
  neopixels.show();
#elif defined(RGB_BUILTIN)
  rgbLedWrite(RGB_BUILTIN, color.red, color.green, color.blue);
#endif
#endif

#ifdef HAS_4DIGIT
  if (co2_concentration >= 10000)
    digits.setSegments(SEG_HI);
  else
    digits.showNumberDec(co2_concentration, false);
#ifdef HAS_LDR
  uint8_t digit_brightness = static_cast<uint8_t>(brightness / 256);
  SERIAL_PRINT(F("4-digit brightness "));
  SERIAL_PRINTLN(digit_brightness);
  digits.setBrightness(digit_brightness);
#endif
#endif

#ifdef HAS_LCD
#ifdef HAS_SCD4X
  displayData(scd4x_data);
#elif defined(HAS_SCD30)
  displayData(scd30_data);
#elif defined(HAS_MHZ19)
  displayData(mhz19_data);
#else
  snprintf(display_buf, DISPLAY_BUF_LEN, "%4dppm", co2_concentration);
  lcd.locate(2, 1);
  lcd.printf(display_buf);
#endif
#endif

#ifdef USE_BLE
  co2_characteristic->setValue(co2_concentration);
  co2_characteristic->notify();
  uint16_t i_temperature = static_cast<int16_t>(temperature*100.0);
  if (isnan(temperature))
    i_temperature = 0x8000; // value not known
  temperature_characteristic->setValue(i_temperature);
  temperature_characteristic->notify();
  uint16_t i_humidity = static_cast<uint16_t>(humidity*100.0);
  if (isnan(humidity))
    i_humidity = 0xFFFF;
  humidity_characteristic->setValue(i_humidity);
  humidity_characteristic->notify();
#endif

  // wait
  delay(100);
}
