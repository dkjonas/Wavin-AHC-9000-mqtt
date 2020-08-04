# Wavin-AHC-9000-mqtt
This is a simple Esp8266 mqtt interface for Wavin AHC-9000/Jablotron AC-116, with the goal of being able to control this heating controller from a home automation system.

## Hardware
The AHC-9000 uses modbus to communicate over a half duplex RS422 connection. It has two RJ45 connectors for this purpose, which can both be used. 
The following schematic shows how to connect an Esp8266 to the AHC-9000:
![Schematic](/electronics/schematic.png)

Components with links to devices on eBay
* Esp8266. I use a [NodeMcu 0.9](https://www.ebay.com/itm/NEW-Version-NodeMcu-Lua-ESP8266-CH340-WIFI-Internet-Development-Board-Module/311413475392?epid=502141093&hash=item4881b08840:g:-IEAAOSw-YVXldDM), mostly because it is very convenient to have the onboard USB interface for programming. Almost anything with an Esp8266 on it will work.
* [24V to 3v3 switchmode converter](https://www.ebay.com/itm/DC-Buck-24V-12V-9V-to-3-3V-3A-Step-Down-Converter-Voltage-Regulator-Power-Module/173494900654?hash=item28651a17ae:g:688AAOSwL1hbgY62). This is only needed if you want to power the Esp8266 from the AHC-9000. A 24V to 5V converter can also be used, if it is connected to the +5V input of the NodeMcu. Please note that not all 3V3 step down converters on eBay supports 24V input
* [MAX3072E](https://www.maximintegrated.com/en/products/interface/transceivers/MAX3072E.html) for converting the 3V3 serial output from the Esp8266 to RS422. There are many similar IC's from other suppliers, which can also be used. Speed is limited, and cables can be kept short, so this is rather uncritical. Note though, that it should be a 3V3 version. [MAX3485](https://www.ebay.com/itm/5pcs-MAX3485CPA-DIP-DIP-8-MAX3485-3-3V-Powered-Transceiver-new/400985402735?hash=item5d5c97ad6f:g:WS4AAOSwGvhT43se) should be compatible, and can be found on eBay.
* RJ45 connector. This can be omitted by soldering a patch cable directly to the circuit.

Depending on the used Esp8266 board and/or the switchmode converter used, it may be benificial to add a larger capacitor in the range of 100uF between 3V3 and gnd.  This will most likely help if you experience WiFi connection/stability issues.

## Software

### Configuration
src/PrivateConfig.h contains 5 constants, that should be changed to fit your own setup.

`WIFI_SSID`, `WIFI_PASS`, `MQTT_SERVER`, `MQTT_USER`, and `MQTT_PASS`.

### Compiling
I use [PlatformIO](https://platformio.org/) for compiling, uploading, and and maintaining dependencies for my code. If you install PlatformIO in a supported editor, building this project is quite simple. Just open the directory containing `platformio.ini` from this project, and click build/upload. If you use a different board than nodemcu, remember to change the `board` variable in `platformio.ini`.
You may be able to use the Arduino tools with the esp8266 additions for compiling, but a few changes may be needed, including downloading dependencies manually.

### Testing
Assuming you have a working mqtt server setup, you should now be able to control your AHC-9000 using mqtt. If you have the [Mosquitto](https://mosquitto.org/) mqtt tools installed on your mqtt server, you can execude:
```
mosquitto_sub -u username -P password -t heat/# -v
```
to see all live updated parameters from the controller.

To change the target temperature for a output, use:
```
mosquitto_pub -u username -P password -t heat/floorXXXXXXXXXXXX/1/target_set -m 20.5
```
where the number 1 in the above command is the output you want to control and 20.5 is the target temperature in degree celcius. XXXXXXXXXXXX is the MAC address of the Esp8266, so it will be unique for your setup.

### Integration with HomeAssistant
If you have a working mqtt setup in [HomeAssistant](https://home-assistant.io/), all you need to do in order to control your heating from HomeAssistant is to enable auto discovery for mqtt in your `configuration.yaml`.
```
mqtt:
  discovery: true
  discovery_prefix: homeassistant
```
You will then get a climate and a battery sensor device for each configured output on the controller.

If you don't like auto discovery, you can add the entries manually. Create an entry for each output you want to control. Replace the number 0 in the topics with the id of the output and XXXXXXXXXXXX with the MAC of the Esp8266 (can be determined with the mosquitto_sub command shown above)
```
climate wavinAhc9000:
  - platform: mqtt
    name: floor_kitchen
    current_temperature_topic: "heat/floorXXXXXXXXXXXX/0/current"
    temperature_command_topic: "heat/floorXXXXXXXXXXXX/0/target_set"
    temperature_state_topic: "heat/floorXXXXXXXXXXXX/0/target"
    mode_command_topic: "heat/floorXXXXXXXXXXXX/0/mode_set"
    mode_state_topic: "heat/floorXXXXXXXXXXXX/0/mode"
    modes:
      - "heat"
      - "off"
    availability_topic: "heat/floorXXXXXXXXXXXX/online"
    payload_available: "True"
    payload_not_available: "False"
    qos: 0

sensor wavinBattery:
  - platform: mqtt
    state_topic: "heat/floorXXXXXXXXXXXX/0/battery"
    availability_topic: "heat/floorXXXXXXXXXXXX/online"
    payload_available: "True"
    payload_not_available: "False"
    name: floor_kitchen_battery
    unit_of_measurement: "%"
    device_class: battery
    qos: 0
```
