/**
 * Air quality monitor with ESP32
 *
 * Uses Sensirion SCD30 sensor with the manufactorer's library
 * for CO2 measurement.
 *
 * Libraries:
 * - https://github.com/Sensirion/arduino-i2c-scd30.git
 * - https://github.com/Sensirion/arduino-core.git
 * For Neopixel:
 * - https://github.com/adafruit/Adafruit_NeoPixel
 * For MQTT:
 * - https://github.com/knolleary/PubSubClient
 * - https://github.com/sstaub/NTP
 */

/* Feature selection */
#define HAS_SCD30
#define HAS_NEOPIXEL
//#define USE_MQTT
#define USE_BLE

/* Pins (GPIOs) for ESP32-C3-DevKitM-1 */
#define PIN_SCL 3 // yellow cable of SCD30
#define PIN_SDA 2 // white cable of SCD30
#define PIN_RGB_LED 8 // built-in rgb LED

/* Includes */
#include <Arduino.h>
#ifdef HAS_SCD30
#include <SensirionI2cScd30.h>
#include <Wire.h>
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
SensirionI2cScd30 co2_sensor;
static char error_message[128];
static int16_t error;
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
#ifdef USE_MQTT
void setupWifi() {
  delay(10);
  // Start by connecting to a WiFi network.
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until reconnected
  for (int tries = 0; !mqtt_client.connected() && tries < 5; tries++) {
    Serial.print("Attempting MQTT connection ...");
    // Create a random client ID.
    String client_id = "ESP32Client-";
    client_id += String(random(0xffff), HEX);
    // Attempt to connect.
    if (mqtt_client.connect(client_id.c_str())) {
      Serial.println("connected");
      // Subscribe.
      mqtt_client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif

/** Setup function */
void setup()
{
    // Set up serial connection.
    Serial.begin(115200);

#ifdef HAS_SCD30
    // Set up Sensirion SCD30 CO2 sensor.
    Wire.begin(PIN_SDA, PIN_SCL);
    co2_sensor.begin(Wire, SCD30_I2C_ADDR_61);
#endif

#ifdef HAS_NEOPIXEL
    // Setup Neopixel.
    neopixels.begin();
#endif

#ifdef USE_MQTT
    setupWifi();
    ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
    ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
    ntp.begin();
    Serial.println("Start NTP");
    mqtt_client.setServer(mqtt_server, 1883);
    mqtt_client.setCallback(callback);
#endif

#ifdef USE_BLE
  Serial.println("Starting BLE ...");
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
  Serial.println("BLE characteristic defined! Now you can read it in your phone!");
#endif
}

/** Loop function */
void loop()
{
    loop_counter++;
    Serial.print("Loop ");
    Serial.println(loop_counter);

    float co2_concentration = NAN;
    float temperature = NAN;
    float humidity = NAN;
#ifdef HAS_SCD30
    /* Take a CO2 measurement */
    error = co2_sensor.blockingReadMeasurementData(co2_concentration, temperature, humidity);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute blockingReadMeasurementData(): ");
        errorToString(error, error_message, sizeof error_message);
        Serial.println(error_message);
    }
    else
    {
        Serial.print("co2Concentration: ");
        Serial.print(co2_concentration);
        Serial.print("\t");
        Serial.print("temperature: ");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("humidity: ");
        Serial.print(humidity);
        Serial.println();
    }
#else
    co2_concentration = 400.0 + static_cast<double>(loop_counter); // test value
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
        snprintf(mqtt_msg, MSG_BUFFER_SIZE, "{\"time\": \"%s\", \"co2\": %.0f, \"temperature\": %.1f, \"humidity\": %.1f}", ntp.formattedTime("%d.%m.%Y %H:%M:%S"), co2_concentration, temperature, humidity);
        mqtt_client.publish(mqtt_topic, mqtt_msg);
    }
#endif

#ifdef HAS_NEOPIXEL
    rgb_t color = colormap.color(co2_concentration);
    Serial.print("Value ");
    Serial.print(co2_concentration);
    Serial.print(" -> ");
    Serial.print(color.red);
    Serial.print(", ");
    Serial.print(color.green);
    Serial.print(", ");
    Serial.println(color.blue);
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
    delay(100);
}
