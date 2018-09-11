# Wavin-AHC-9000-mqtt
This is a simple Esp8266 mqtt interface for Wavin AHC-9000/Jablotron AC-116, with the goal of being able to control this heating controller from a home automation system.

## Hardware
The AHC-9000 uses modbus to communicate over half duplex RS422 connection. It has two RJ45 connectors for this purpose, which can both be used. 
The following schematic shows how to connect an Esp8266 to the AHC-9000:
![Schematic](/electronics/schematic.png)

Components:
* Esp8266. I use a [NodeMcu 0.9](https://www.ebay.com/itm/NEW-Version-NodeMcu-Lua-ESP8266-CH340-WIFI-Internet-Development-Board-Module/311413475392?epid=502141093&hash=item4881b08840:g:-IEAAOSw-YVXldDM), mostly because it is very convenient to have the onboard USB interface for programming. Almost anything with an Esp8266 on it will work.
* [24V to 3v3 switchmode converter](https://www.ebay.com/itm/DC-Buck-24V-12V-9V-to-3-3V-3A-Step-Down-Converter-Voltage-Regulator-Power-Module/173494900654?hash=item28651a17ae:g:688AAOSwL1hbgY62). This is only needed if you want to power the Esp8266 from the AHC-9000. A 24V to 5V converter can also be used, if it is connected to the +5V input of the NodeMcu. Please note that not all 3V3 step down converters on eBay supports 24V input
* [MAX3072E](https://www.maximintegrated.com/en/products/interface/transceivers/MAX3072E.html) for converting the 3V3 serial output from the Esp8266 to RS422. There are many similar IC's from other suppliers, which can also be used. Speed is limited, and cables can be kept short, so this is rather uncritical. Note though, that it should be a 3V3 version. [MAX3485](https://www.ebay.com/itm/5pcs-MAX3485CPA-DIP-DIP-8-MAX3485-3-3V-Powered-Transceiver-new/400985402735?hash=item5d5c97ad6f:g:WS4AAOSwGvhT43se) should be compatible, and can be found on eBay.
* RJ45 connector. This can be omitted by soldering a patch cable directly to the circuit.
