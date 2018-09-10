#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "WavinController.h"

// Number of thermostats connected to the controller.
// It is assumed each thermostat controls one output, that all thermostats learned into the controller are used,
// and that no thermostats has been removed from the controller
const uint8_t  MAX_ELEMENTS = 5;

const String   WIFI_SSID = "Enter wireless SSID here";     // wifi ssid
const String   WIFI_PASS = "Enter wireless password here"; // wifi password

const String   MQTT_CLIENT = "WAVIN_AC-116";                 // mqtt client_id must be unique for each device connecting to a server
const String   MQTT_SERVER = "Enter IP of mqtt server here"; // mqtt server
const uint16_t MQTT_PORT   = 1883;                           // mqtt port
const String   MQTT_USER   = "Enter mqtt username here";     // mqtt user
const String   MQTT_PASS   = "Enter mqtt password here";     // mqtt password


// Mqtt defines. Default is topics like 'heat/floor/3/target', where 3 is the thermostat id
const String   MQTT_PREFIX              = "heat/floor/"; // include tailing '/' in prefix
const String   MQTT_ONLINE              = "online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

const String   MQTT_WILL = String(MQTT_PREFIX + MQTT_ONLINE);
const String   MQTT_SETPOINT_SET = String(MQTT_PREFIX + "+" + MQTT_SUFFIX_SETPOINT_SET);

const uint8_t TX_ENABLE_PIN = 5;
const bool SWAP_SERIAL_PINS = true;
const uint16_t RECIEVE_TIMEOUT_MS = 1000;
WavinController wavinController(TX_ENABLE_PIN, SWAP_SERIAL_PINS, RECIEVE_TIMEOUT_MS);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastUpdateTime = 0;

const uint16_t POLL_TIME_MS = 5000;

struct lastKnownValue_t {
  unsigned short temperature;
  unsigned short setpoint;
  unsigned short battery;
  unsigned short status;
} lastSentValues[MAX_ELEMENTS];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;


// Read a float value from a non zero terminated array of bytes and
// return 10 times the value as an integer
uint16_t temperatureFromString(byte* payload, unsigned int length)
{
  char terminatedPayload[length+1];
  for(unsigned int i=0; i<length; i++)
  {
    terminatedPayload[i] = payload[i];
  }
  terminatedPayload[length] = 0;

  float targetf = atof(terminatedPayload);
  return (unsigned short)(targetf * 10);
}


// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}


uint8_t getIdFromTopic(char* topic)
{
  unsigned int startIndex = MQTT_PREFIX.length();
  int i = 0;
  uint8_t result = 0;

  while(topic[startIndex+i] != '/' && i<5)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  uint8_t id = getIdFromTopic(topic);
  uint16_t target = temperatureFromString(payload, length);
  wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, id, 0, target);

  // Force re-read of registers from controller now
  lastUpdateTime = 0;
}


void resetLastSentValues()
{
  for(int8_t i=0; i<MAX_ELEMENTS; i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
  }
}


void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}


void setup()
{
  mqttClient.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}


void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!mqttClient.connected())
    {
      if (mqttClient.connect(MQTT_CLIENT.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), MQTT_WILL.c_str(), 1, true, "False") )
      {
          mqttClient.subscribe(MQTT_SETPOINT_SET.c_str(), 1);
          mqttClient.publish(MQTT_WILL.c_str(), (const uint8_t *)"True", 4, true);

          // Forces resending of all parameters to server
          resetLastSentValues();
      }
      else
      {
          return;
      }
    }

    if (lastUpdateTime + POLL_TIME_MS < millis())
    {
      lastUpdateTime = millis();

      uint16_t registers[12];
      for (uint8_t page = 0; page < MAX_ELEMENTS; page++)
      {
        // ELEMENTS are the thermostats. It is assumed that the thermostats are learned into the controller in the
        // first n pages of the elements, which may not be true if thermostats have been removed from the controller.
        // ELEMENTS_SYNC_GROUP is the id of the output the thermostat controls (lowest output if more than one channel is controlled)
        if (wavinController.readRegisters(WavinController::CATEGORY_ELEMENTS, page, 0, 12, registers))
        {
          uint16_t output = registers[WavinController::ELEMENTS_SYNC_GROUP];
          uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
          uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

          String topic = String(MQTT_PREFIX + output + MQTT_SUFFIX_CURRENT);
          String payload = temperatureAsFloatString(temperature);

          publishIfNewValue(topic, payload, temperature, &(lastSentValues[output].temperature));

          topic = String(MQTT_PREFIX + output + MQTT_SUFFIX_BATTERY);
          payload = String(battery*10);

          publishIfNewValue(topic, payload, battery, &(lastSentValues[output].battery));
        }

        // Read the current setpoint programmed for output
        if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, page, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
        {
          uint16_t setpoint = registers[0];

          String topic = String(MQTT_PREFIX + page + MQTT_SUFFIX_SETPOINT_GET);
          String payload = temperatureAsFloatString(setpoint);

          publishIfNewValue(topic, payload, setpoint, &(lastSentValues[page].setpoint));
        }

        // Read the current status of the output
        if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, page, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
        {
          uint16_t status = registers[0];

          String topic = String(MQTT_PREFIX + page + MQTT_SUFFIX_OUTPUT);
          String payload;
          if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
            payload = "on";
          else
            payload = "off";

          publishIfNewValue(topic, payload, status, &(lastSentValues[page].status));
        }
      }
    }

    if (mqttClient.connected())
    {
      mqttClient.loop();
    }
  }
}
