#pragma once

#ifdef USE_MQTT
#define MQTT_PUB_INTERVAL 60000 // publish time interval in milliseconds

#define WIFI_SSID "mywifissid"
#define WIFI_PASSWORD "mywifipassword"
#define NTP_SERVER "de.pool.ntp.org"
#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"
#define MQTT_SERVER "myserver"
#define MQTT_TOPIC_PREFIX "prefix"
#define MQTT_TOPIC_MEASUREMENT "air-monitor"
//#define MQTT_TOPIC_RECEIVE "scd4x"
#endif
