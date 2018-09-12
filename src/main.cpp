#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "WavinController.h"

const String   WIFI_SSID = "Enter wireless SSID here";     // wifi ssid
const String   WIFI_PASS = "Enter wireless password here"; // wifi password

const String   MQTT_CLIENT = "Wavin-AHC-9000-mqtt";          // mqtt client_id prefix. Will be suffixed with Esp8266 mac to make it unique
const String   MQTT_SERVER = "Enter IP of mqtt server here"; // mqtt server
const uint16_t MQTT_PORT   = 1883;                           // mqtt port
const String   MQTT_USER   = "Enter mqtt username here";     // mqtt user
const String   MQTT_PASS   = "Enter mqtt password here";     // mqtt password

// MQTT defines
// Esp8266 MAC will be added to the prefix, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "heat/floor";  // do not include tailing '/' in prefix
const String   MQTT_ONLINE              = "online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

String mqttPrefixWithMac;
String mqttClientWithMac;

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
} lastSentValues[WavinController::NUMBER_OF_CHANNELS];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[WavinController::NUMBER_OF_CHANNELS];

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
  unsigned int startIndex = mqttPrefixWithMac.length();
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
  for(int8_t i=0; i<WavinController::NUMBER_OF_CHANNELS; i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
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
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttPrefixWithMac = String(MQTT_PREFIX + macStr + "/");
  mqttClientWithMac = String(MQTT_CLIENT + macStr);

  mqttClient.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}


void publishConfiguration(uint8_t channel)
{
  // TODO: publish discovery message for HomeAssistant
  // https://www.home-assistant.io/docs/mqtt/discovery/
  configurationPublished[channel] = true;
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
      String will = String(mqttPrefixWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      {
          String setpointSetTopic = String(mqttPrefixWithMac + "+" + MQTT_SUFFIX_SETPOINT_SET);
          mqttClient.subscribe(setpointSetTopic.c_str(), 1);
          mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, true);

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

      uint16_t registers[11];

      for(uint8_t channel = 0; channel < WavinController::NUMBER_OF_CHANNELS; channel++)
      {
        if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
        {
          uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
          bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;

          if(primaryElement==0)
          {
              // Channel not used
              continue;
          }

          if(!configurationPublished[channel])
          {
            publishConfiguration(channel);
          }

          // Read the current setpoint programmed for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
          {
            uint16_t setpoint = registers[0];

            String topic = String(mqttPrefixWithMac + channel + MQTT_SUFFIX_SETPOINT_GET);
            String payload = temperatureAsFloatString(setpoint);

            publishIfNewValue(topic, payload, setpoint, &(lastSentValues[channel].setpoint));
          }

          // Read the current status of the output for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
          {
            uint16_t status = registers[0];

            String topic = String(mqttPrefixWithMac + channel + MQTT_SUFFIX_OUTPUT);
            String payload;
            if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
              payload = "on";
            else
              payload = "off";

            publishIfNewValue(topic, payload, status, &(lastSentValues[channel].status));
          }

          // If a thermostat for the channel is connected to the controller
          if(!allThermostatsLost)
          {
            // Read values from the primary thermostat connected to this channel 
            // Primary element from controller is returned as index+1, so 1 i subtracted here to read the correct element
            if (wavinController.readRegisters(WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
            {
              uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
              uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

              String topic = String(mqttPrefixWithMac + channel + MQTT_SUFFIX_CURRENT);
              String payload = temperatureAsFloatString(temperature);

              publishIfNewValue(topic, payload, temperature, &(lastSentValues[channel].temperature));

              topic = String(mqttPrefixWithMac + channel + MQTT_SUFFIX_BATTERY);
              payload = String(battery*10);

              publishIfNewValue(topic, payload, battery, &(lastSentValues[channel].battery));
            }
          }
        }
      }
    }

    if (mqttClient.connected())
    {
      mqttClient.loop();
    }
  }
}
