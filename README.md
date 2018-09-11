# Wavin-AHC-9000-mqtt
This is a simple Esp8266 mqtt interface for Wavin AHC-9000/Jablotron AC-116, with the goal of being able to control this heating controller from a home automation system.

## Hardware
The AHC-9000 uses modbus to communicate over half duplex RS422 connection. It has two RJ45 connectors for this purpose, which can both be used. 
The following schematic shows how to connect an Esp8266 to the AHC-9000:
![Schematic](/electronics/schematic.png)

Components with links to devices on eBay
* Esp8266. I use a [NodeMcu 0.9](https://www.ebay.com/itm/NEW-Version-NodeMcu-Lua-ESP8266-CH340-WIFI-Internet-Development-Board-Module/311413475392?epid=502141093&hash=item4881b08840:g:-IEAAOSw-YVXldDM), mostly because it is very convenient to have the onboard USB interface for programming. Almost anything with an Esp8266 on it will work.
* [24V to 3v3 switchmode converter](https://www.ebay.com/itm/DC-Buck-24V-12V-9V-to-3-3V-3A-Step-Down-Converter-Voltage-Regulator-Power-Module/173494900654?hash=item28651a17ae:g:688AAOSwL1hbgY62). This is only needed if you want to power the Esp8266 from the AHC-9000. A 24V to 5V converter can also be used, if it is connected to the +5V input of the NodeMcu. Please note that not all 3V3 step down converters on eBay supports 24V input
* [MAX3072E](https://www.maximintegrated.com/en/products/interface/transceivers/MAX3072E.html) for converting the 3V3 serial output from the Esp8266 to RS422. There are many similar IC's from other suppliers, which can also be used. Speed is limited, and cables can be kept short, so this is rather uncritical. Note though, that it should be a 3V3 version. [MAX3485](https://www.ebay.com/itm/5pcs-MAX3485CPA-DIP-DIP-8-MAX3485-3-3V-Powered-Transceiver-new/400985402735?hash=item5d5c97ad6f:g:WS4AAOSwGvhT43se) should be compatible, and can be found on eBay.
* RJ45 connector. This can be omitted by soldering a patch cable directly to the circuit.

## Software

### Configuration
src/main.cpp containts 6 constants, that should be changed to fit your own setup.

`WIFI_SSID`, `WIFI_PASS`, `MQTT_SERVER`, `MQTT_USER`, `MQTT_PASS`, and `MAX_ELEMENTS`.
`MAX_ELEMENTS` is the number of thermostats connected to the controller. See comment in the code.

### Compiling
I use [PlatformIO](https://platformio.org/) for compiling, uploading, and and maintaining dependencies for my code. If you install PlatformIO in a supported editor, building this project is quite simple. Just open the directory containing `platformio.ini` from this project, and click build/upload. If you prefer you may be able to use the Arduino tools with the esp8266 additions for compiling, but a few changes may be needed, including downloading dependencies manually.

### Testing
Assuming you have a working mqtt server setup, you should now be able to control your AHC-9000 using mqtt. If you have the [Mosquitto](https://mosquitto.org/) mqtt tools installed on your mqtt server, you can execude:
```
mosquitto_sub -u username -P password -t heat/floor/# -v
```
to see all live updated parameters from the controller.

To change the target temperature for a thermostat, use:
```
mosquitto_pub -u username -P password -t heat/floor/1/target_set -m 20.5
```
where the number 1 in the above command is the thermostat you want to control and 20.5 is the target temperature in degree celcius.

### Integration with HomeAssistant
If you have a working mqtt setup in [HomeAssistant](https://home-assistant.io/), all you need to do in order to control your heating from HomeAssistant, is to include the following in your `configuration.yaml`. Create an entry for each thermostat you want to control. Replace the number 0 in the topics with the id of the thermostat.
```
climate wavinAhc9000:
  - platform: mqtt
    name: floor_kitchen
    current_temperature_topic: "heat/floor/0/current"
    temperature_command_topic: "heat/floor/0/target_set"
    temperature_state_topic: "heat/floor/0/target"
    qos: 0

sensor wavinBattery:
  - platform: mqtt
    state_topic: "heat/floor/0/battery"
    name: floor_kitchen_battery
    unit_of_measurement: "%"
    qos: 0
```
