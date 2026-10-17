## KD Sensor Modul

![ESP32](https://img.shields.io/badge/ESP32-grey?logo=Espressif)
![KiCad](https://img.shields.io/badge/KiCad-darkblue?logo=KiCad)
![Relaise](https://img.shields.io/github/release-date/gerryvel/Kieldaten?)
![lastcommit](https://img.shields.io/github/last-commit/gerryvel/Kieldaten)
[![OBP](https://img.shields.io/badge/Sailing_with-OpenBoatsProjects-blue)](https://open-boat-projects.org/de/)


## Description

The ESP32 in this project is an Adafruit Huzzah! ESP32. This is a small module without USB connector.

With the ESP32 and Sensors for  distance (Digilant PmodToF) it's possible
monitoring the varaible keel (Kiel). 
All data is available in the web interface. 

The 12 Volt is reduced to 5 Volt with a DC Step-Down_Converter. 12V DC comes from the N2k Bus Connector with the M12 Connector for future Data to NMEA2000).

The Website use LittleFS Filesystem. You must use Partition Schemes "Minimal SPIFFS with APPS and OTA".
The HTML Data upload separately with 
- Arduino IDE > "ESP 32 Sketch Data upload" or 
- VS-Code > Build Filesystem and Upload Filesystem Image (PlatformIO) 
from /data directory.

# Physik:


# Partlist:

- Adafruit Huzzah! ESP32 (for programming need USB-Adapter)[Link](https://www.exp-tech.de/plattformen/internet-of-things-iot/9350/adafruit-huzzah32-esp32-breakout-board)
- SN65HVD230 [Link](https://www.reichelt.de/high-speed-can-transceiver-1-mbit-s-3-3-v-so-8-sn-65hvd230d-p58427.html?&trstct=pos_0&nbc=1)
- Traco-Power TSR 1-2450 for 12V / 5V [Link](https://www.reichelt.de/dc-dc-wandler-tsr-1-1-w-5-v-1000-ma-sil-to-220-tsr-1-2450-p116850.html?search=tsr+1-24)
- Case Wago 789 [Link](https://www.conrad.de/de/p/wago-789-905-hutschienen-gehaeuse-90-x-17-5-x-55-polyamid-6-6-grau-1-set-530120.html)
- Resistor 200Ohm , 10kOhm


# Wiring diagram



# PCB and Housing assembly


# Webinterface


# Versions

- 1.0 working Version
