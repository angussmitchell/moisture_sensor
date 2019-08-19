/*
  Example sketch using the VIA Chameleon Shield library
  File: VIAChameleonShieldExample.ino
  Author: Matthew Driver
  Edited: 1 June 2019

Required Arduino libraries (via 'Include Library'\'Manage Libraries...':
Adafruit NeoPixel (by Adafruit)
U8glib by oliver
DallasTemperature by Miles Burton et al

Hardware assumed to be Arduino Uno compatible with an ATMEGA329p microcontroller

Set serial monitor to 57600 BAUD.

Arduino pins used:
#define DS18B20_Pin              8
#define MUXpinAddressA           5
#define MUXpinAddressB           6
#define MUXpinEnable             A2
#define sensorYAnalogPin         A0
#define sensorXAnalogPin         A1
#define LEDSdataPin              7
*/


#include "VIAChameleonShield.h"   //will look in the sketch folder first and next in the library directories
VIAChameleonShield chameleon;

void setup() {
  chameleon.initialise_shield();
}

void loop() {
  char temp1[16];
  chameleon.ReadAllSensors();  // Read sensors 1 to 3 and the temperature ID chip
  chameleon.SetChameleonLED();
  if (chameleon.TempIDSensorExists) {

    Serial.print(F("Temperature ID Sensor exists: True"));
    Serial.print(F("\r\nTemperature: "));
    Serial.print(chameleon.TempID_Celsius);
    Serial.print(F("\r\nShort ID: "));
    Serial.print(chameleon.TempID_short_id);
    Serial.print(F("\r\nLong ID: "));
    Serial.print(chameleon.TempID_long_id);
    Serial.print(F("\r\n"));

    //Print out sensor resistances
    for (int PortNo = 1; PortNo < 4; PortNo++) {  //loop through sensors 1 to 3.
      Serial.print(F("Port: "));
      Serial.print(PortNo);
      Serial.print(F(", Resistance(kOhms) - Raw: "));
      Serial.print(chameleon.Sensor[PortNo].resistance_raw);
      Serial.print(F(", Temp. calibrated: "));
      Serial.print(chameleon.Sensor[PortNo].resistance_temp_calibrated);
      Serial.print(F(", Formatted: "));
      Serial.print(chameleon.Sensor[PortNo].resistance_formatted);
      Serial.print(F("k\r\n"));
    }

    chameleon.screen(5, "", "", "", 2000); //display resistance values of sensors
    chameleon.screen(3, "Sensor ID", chameleon.TempID_short_id, "", 2000);
    dtostrf(chameleon.TempID_Celsius, 2, 0, temp1);   // convert int to char
    strcat(temp1, "C");  // append 'C' to the end of the temperature
    chameleon.screen(3, "Temperature", temp1, "", 2000);
  }
  else {
    Serial.print(F("Temperature ID Sensor exists: False\r\n"));
    chameleon.screen(2, "Plug in", "sensor cable", "", 500);
  }
}
