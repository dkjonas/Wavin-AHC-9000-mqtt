#include <ESP8266WiFi.h>

const String   WIFI_SSID = "Enter wireless SSID here";         // wifi ssid
const String   WIFI_PASS = "Enter wireless password here";     // wifi password

const String   MQTT_SERVER = "Enter mqtt server address here"; // mqtt server address without port number
const String   MQTT_USER   = "Enter mqtt username here";       // mqtt user. Use "" for no username
const String   MQTT_PASS   = "Enter mqtt password here";       // mqtt password. Use "" for no password
const uint16_t MQTT_PORT   = 1883;                             // mqtt port

const float    MIN_TEMP    = 6.0;                              // minimum temperature to set
const float    MAX_TEMP    = 40.0;                             // maximum temperature to set
const float    TEMP_STEP   = 0.5;                              // the temperature step size, either 0.5 or 1.0
