#ifdef USE_MQTT
#define MQTT_PUB_INTERVAL 60000 // publish time interval in milliseconds

const char* wifi_ssid = "mywifissid";
const char* wifi_password = "mywifipassword";
const char* ntp_server = "de.pool.ntp.org";
const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";
const char* mqtt_server = "myserver";
const char* mqtt_topic_prefix = "prefix";
const char* mqtt_topic_measurement = "air-monitor";
#endif
