/*
    Name:       bootsdaten.ino
    Created:	10.10.2026
    Update: 	V1.0 vom 19.10.2026
    Author:     Gerry Sebb
*/

#include <Arduino.h>
#include <ArduinoOTA.h>
#include "BoardInfo.h"
#include "configuration.h"
#include "helper.h"
#include "web.h"
#include "LED.h"
#include <esp.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ESP_WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <NMEA2000_CAN.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "NMEA0183Telegram.h"
#include "tof.h"


// NMEA2000
Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress

//NMEA 0183 Stream
WiFiUDP udp;  // Create UDP instance

void SendNMEA0183Message(String var) {
  udp.beginPacket(udpAddress, udpPort);   // Send to UDP
  udp.println(var);
  udp.endPacket();
}

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127257L, // Yaw,Pitch,Roll                                              
                                                  0
                                                 };


bool IsTimeToUpdate(unsigned long NextUpdate) {
  return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
  return millis() + Period + Offset;
}
void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
  while ( NextUpdate < millis() ) NextUpdate += Period;
}



// The setup() function runs once each time the micro-controller starts
void setup()
{
  Serial.begin(115200);

  Serial.printf("KielSensor setup %s start\n", VersionSoftware);

  //Filesystem prepare for Webfiles
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  Serial.println("Speicher LittleFS benutzt:");
  Serial.println(LittleFS.usedBytes());

  File root = LittleFS.open("/");
  listDir(LittleFS, "/", 3);
  
  // file exists, reading and loading config file
  readConfig("/config.json");
  AP_SSID = tWeb_Config.wAP_SSID;
  AP_PASSWORD = tWeb_Config.wAP_Password;
  fKielOffset = atof(tWeb_Config.wKiel_Offset);

  Serial.println("\nConfigdata : AP SSID: " + AP_SSID + " , Passwort: " + AP_PASSWORD + " , Kieloffset: " + fKielOffset);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  I2C_scan();

  // Start TOF Sensor: 
  ToF_begin(); //initialize the pmod
  debug();     //display all registers
  delay(1000); //wait one second (make the message readable)

  // Calibrate TOF sensor
  ToF_calibrate();

  //LED
  LEDInit();
    
  // Boardinfo	
  sBoardInfo = boardInfo.ShowChipIDtoString();


  //WiFiServer AP starten
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(1000);
  if (WiFi.softAPConfig(IP, Gateway, NMask)){
    Serial.println("Network " + String(AP_SSID) + " running");	
    Serial.println("\nNetwork IP " + IP.toString() + " ,GW: " + Gateway.toString() + " ,Mask: " + NMask.toString() + " set");
  } else {
    Serial.println("IP config not success");	
  }

  if (WiFi.setHostname(HostName))
    Serial.println("\nSet Hostname success");
  else
    Serial.println("\nSet Hostname not success");

  delay(1000);

  WiFiDiag();

  if (!MDNS.begin(HostName)) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  // Start TCP (HTTP) server
  server.begin();
  Serial.println("TCP server started\n");

  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);

  //Website
  website();
  // firmware_configure();

  // NMEA2000
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Set product information
  NMEA2000.SetProductInformation("KD01.2610", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "KD Sensor Module",  // Manufacturer's Model ID
                                 VersionSoftware,  // Manufacturer's Software version code
                                 VersionHardware // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(01, // Unique number. Use e.g. Serial number.
                                180, // 180 Device function=Attitude (Pitch, Roll, Yaw) Control. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                40, // 40 Device class=Steering and Control Surfaces. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 34);  // Read stored last NodeAddress, default 33
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  Serial.println(F("Opening NMEA2000"));
  if (NMEA2000.Open())
    Serial.println(F("NMEA2000 Initialized\n"));
  else
    Serial.println(F("NMEA2000 Initialized failed\n"));

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else
        type = "filesystem";
      Serial.println("start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname(HostName);
  ArduinoOTA.begin();
}


void loop()
{
  //Wifi variables
  bConnect_CL = WiFi.status() == WL_CONNECTED ? 1 : 0;
  //bAP_on = WiFi.enableAP(bAP_on);
    
  // LED visu Wifi
  LEDflash(LED(Green)); // Betrieb ok
  if (bI2C_Status == 0){
    //LEDflash(LED(Red)); // Sensorfehler
    Serial.print("Sensorfehler\n");
  } 

  // OTA	
  ArduinoOTA.handle();

  // Status AP 
  Serial.printf("Stationen mit AP verbunden = %d\n", WiFi.softAPgetStationNum());
  Serial.printf("Soft-AP IP address = %s\n", WiFi.softAPIP().toString());
  sCL_Status = sWifiStatus(WiFi.status());

  // Distance-Sensor
  constexpr uint32_t INTERVAL_MS = 30000;
  uint32_t lastRead = 0;

  if (millis() - lastRead >= INTERVAL_MS) {
    lastRead = millis();

    float fdistanceMeters = ToF_getMeasurement();

      Serial.print("Distanz: ");
      Serial.print(fdistanceMeters, 3);
      Serial.println(" m");
    } else {
      Serial.println("I2C-Lesefehler");
    }
  
  Serial.printf("Schwert: %f cm\n", fSStellung);
  fAbsTief = fdistanceMeters + fKielOffset;
  Serial.printf("Tiefgang: %f cm\n", fAbsTief);

  // Send Messages NMEA 0183
  SendNMEA0183Message(sendXDR()); // Send NMEA0183 Message

  delay(200);

  // Send Messages NMEA 2000

  delay(200);

  //Serial.print("NMEA2000.ParseMessages............\n");

  NMEA2000.ParseMessages();       // Parse NMEA2000 Messages
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) {
    Serial.read();
  }

  // OLED
  //	oledloop();

  // Website Data	
  freeHeapSpace();
  
  if (IsRebootRequired) {
    Serial.println("Rebooting ESP32: "); 
    delay(1000); // give time for reboot page to load
    ESP.restart();
  }
}

