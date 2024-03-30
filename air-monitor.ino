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
 * For MQTT:
 * - https://github.com/knolleary/PubSubClient
 * - https://github.com/sstaub/NTP
 */

/* Feature selection */
#define SERIAL_OUTPUT
//#define HAS_SCD30
//#define HAS_SCD4X
#define HAS_MHZ19
#define HAS_NEOPIXEL
//#define USE_MQTT
#define USE_BLE

/* Pins (GPIOs) for ESP32-C3-DevKitM-1 */
#define PIN_SCL 3 // yellow cable of SCDxx
#define PIN_SDA 2 // white cable of SCDxx
#define PIN_RGB_LED 8 // built-in rgb LED
#define MHZ19_SERIAL_PORT 0 // serial port to use for MH-Z19

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
#include "colormap.h"
#endif
#ifdef USE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTP.h>
#include "mqtt-settings.h"
#endif
#ifdef USE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#endif
#include <limits.h>

/* Constants */
#ifdef SERIAL_OUTPUT
#define SERIAL_PRINT(x) Serial.print(x)
#define SERIAL_PRINTLN(x) Serial.println(x)
#define SERIAL_PRINT_HEX(x) Serial.print(x, HEX)
#else
#define SERIAL_PRINT(x)
#define SERIAL_PRINTLN(x)
#define SERIAL_PRINT_HEX(x)
#endif
#ifdef HAS_MHZ19
#define MHZ19_BAUDRATE 9600
#endif
#ifdef HAS_NEOPIXEL
#define NEOPIXEL_PIN PIN_RGB_LED
#define NEOPIXEL_NUM 1
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
static uint16_t loop_counter;
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
MHZ19 mhz19;
HardwareSerial serial_port(MHZ19_SERIAL_PORT);
#endif
#ifdef HAS_NEOPIXEL
Adafruit_NeoPixel neopixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Colormap colormap(600.0, 1200.0);
#endif
#ifdef USE_MQTT
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
WiFiUDP wifiudp;
NTP ntp(wifiudp);
static unsigned long time_last_pub = 0;
#define MSG_BUFFER_SIZE	(200)
char mqtt_topic[MSG_BUFFER_SIZE];
char mqtt_msg[MSG_BUFFER_SIZE];
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
void setupWifi() {
  delay(10);
  // Start by connecting to a WiFi network.
  SERIAL_PRINTLN();
  SERIAL_PRINT("Connecting to ");
  SERIAL_PRINTLN(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL_PRINT(".");
  }

  randomSeed(micros());

  SERIAL_PRINTLN();
  SERIAL_PRINTLN("WiFi connected");
  SERIAL_PRINTLN("IP address: ");
  SERIAL_PRINTLN(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  SERIAL_PRINT("Message arrived [");
  SERIAL_PRINT(topic);
  SERIAL_PRINT("] ");
  for (int i = 0; i < length; i++) {
    SERIAL_PRINT((char)payload[i]);
  }
  SERIAL_PRINTLN();
}

void reconnect() {
  // Loop until reconnected
  for (int tries = 0; !mqtt_client.connected() && tries < 5; tries++) {
    SERIAL_PRINT("Attempting MQTT connection ...");
    // Create a random client ID.
    String client_id = "ESP32-Air-Monitor-";
    client_id += String(random(0xffff), HEX);
    // Attempt to connect.
    if (mqtt_client.connect(client_id.c_str())) {
      SERIAL_PRINTLN("connected");
      // Subscribe.
      mqtt_client.subscribe("inTopic");
    } else {
      SERIAL_PRINT("failed, rc=");
      SERIAL_PRINT(mqtt_client.state());
      SERIAL_PRINTLN(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif

/** Setup function */
void setup()
{
#ifdef SERIAL_OUTPUT
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
#endif

#if defined(HAS_SCD30) || defined(HAS_SCD4X)
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

#ifdef USE_MQTT
  setupWifi();
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  ntp.begin();
  SERIAL_PRINTLN("Start NTP");
  mqtt_client.setServer(mqtt_server, 1883);
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
}

/** Loop function */
void loop()
{
  loop_counter++;
  SERIAL_PRINT("Loop ");
  SERIAL_PRINTLN(loop_counter);

  float co2_concentration = NAN;
  float temperature = NAN;
  float humidity = NAN;
#ifdef HAS_SCD30
  error = scd30.blockingReadMeasurementData(co2_concentration, temperature, humidity);
  if (error != NO_ERROR) {
    SERIAL_PRINT("Error trying to execute blockingReadMeasurementData(): ");
    errorToString(error, error_message, sizeof error_message);
    SERIAL_PRINTLN(error_message);
  }
  else
  {
    SERIAL_PRINT("co2Concentration: ");
    SERIAL_PRINT(co2_concentration);
    SERIAL_PRINT("ppm\ttemperature: ");
    SERIAL_PRINT(temperature);
    SERIAL_PRINT("°C\thumidity: ");
    SERIAL_PRINT(humidity);
    SERIAL_PRINTLN("%");
  }
#endif
#ifdef HAS_SCD4X
  bool is_data_ready = false;
  while (!is_data_ready)
  {
    delay(100);
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
    uint16_t co2 = 0;
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error)
    {
      SERIAL_PRINT("Error trying to execute readMeasurement(): ");
      errorToString(error, error_message, 256);
      SERIAL_PRINTLN(error_message);
    }
    else if (co2 == 0)
    {
      SERIAL_PRINTLN("Invalid sample detected, skipping.");
    }
    else
    {
      co2_concentration = static_cast<float>(co2);
      SERIAL_PRINT("CO2: ");
      SERIAL_PRINT(co2);
      SERIAL_PRINT("ppm\tTemperature: ");
      SERIAL_PRINT(temperature);
      SERIAL_PRINT("°C\tHumidity: ");
      SERIAL_PRINT(humidity);
      SERIAL_PRINTLN("%");
    }
  }
#endif
#ifdef HAS_MHZ19
  int co2 = mhz19.getCO2();
  co2_concentration = static_cast<float>(co2);
  int8_t temp = mhz19.getTemperature();
  temperature = static_cast<float>(temp);
  SERIAL_PRINT("CO2: ");
  SERIAL_PRINT(co2);
  SERIAL_PRINT("ppm\t");
  SERIAL_PRINT("Temperature: ");
  SERIAL_PRINT(temp);
  SERIAL_PRINTLN("°C");
#endif
#if !defined(HAS_SCD30) && !defined(HAS_SCD4X) && !defined(HAS_MHZ19)
  co2_concentration = 400.0 + static_cast<float>(loop_counter); // test value
#endif

#ifdef USE_MQTT
  if (!mqtt_client.connected())
    reconnect();
  if (mqtt_client.connected()) {
    mqtt_client.loop();
  }
  long cur_time = millis();
  if ( ((cur_time - time_last_pub >= MQTT_PUB_INTERVAL) || (cur_time < time_last_pub)) && (!isnan(co2_concentration)) ) {
    time_last_pub = cur_time;
    ntp.update();
    snprintf(mqtt_topic, MSG_BUFFER_SIZE, "%s/%s", mqtt_topic_prefix, mqtt_topic_measurement);
#ifdef HAS_MHZ19
    snprintf(mqtt_msg, MSG_BUFFER_SIZE, "{\"time\": \"%s\", \"co2\": %.0f, \"temperature\": %.0f}", ntp.formattedTime("%d.%m.%Y %H:%M:%S"), co2_concentration, temperature);
#else
    snprintf(mqtt_msg, MSG_BUFFER_SIZE, "{\"time\": \"%s\", \"co2\": %.0f, \"temperature\": %.1f, \"humidity\": %.1f}", ntp.formattedTime("%d.%m.%Y %H:%M:%S"), co2_concentration, temperature, humidity);
#endif
    mqtt_client.publish(mqtt_topic, mqtt_msg);
  }
#endif

#ifdef HAS_NEOPIXEL
  rgb_t color = colormap.color(co2_concentration);
  SERIAL_PRINT("Value ");
  SERIAL_PRINT(co2_concentration);
  SERIAL_PRINT(" -> ");
  SERIAL_PRINT(color.red);
  SERIAL_PRINT(", ");
  SERIAL_PRINT(color.green);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(color.blue);
  neopixels.setPixelColor(0, neopixels.Color(color.red, color.green, color.blue));
  neopixels.show();
#endif

#ifdef USE_BLE
  uint16_t i_co2 = static_cast<uint16_t>(co2_concentration);
  co2_characteristic->setValue(i_co2);
  co2_characteristic->notify();
  uint16_t i_temperature = static_cast<int16_t>(static_cast<int16_t>(temperature*100.0));
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
  delay(200);
}
