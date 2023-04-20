#ifdef USE_MQTT
#define MQTT_PUB_INTERVAL 60000 // publish time interval in milliseconds

const char* wifi_ssid = "mywifissid";
const char* wifi_password = "mywifipassword";
const char* mqtt_server = "myserver";
const char* mqtt_topic_prefix = "prefix";
const char* mqtt_topic_measurement = "air-monitor";
#endif
