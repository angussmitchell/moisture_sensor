/*
  VIA Chameleon_Shield Arduino library
  File: VIAChameleonShieldExample.ino
  Author: Matthew Driver
  Edited: 1 June 2019


*/

#include "Arduino.h"


#include "VIAChameleonShield.h"


//WS2812B
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDSdataPin, NEO_GRB + NEO_KHZ800);

//OLED
#include <Wire.h>
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);  // Fast I2C / TWI

//DS18B20
OneWire oneWire(DS18B20_Pin);
Statistic myStats;
DallasTemperature sensors_DS18B20(&oneWire);

VIAChameleonShield::VIAChameleonShield()
{
  setup_mux();
}

void VIAChameleonShield::Read_DS18B20() {
  TempID_Celsius = -127;
  TempID_short_id[0] = '\0';
  TempID_long_id[0] = '\0';
  sensors_DS18B20.begin();
  if (sensors_DS18B20.getDeviceCount() > 0)  {
    TempIDSensorExists = true;
    sensors_DS18B20.requestTemperatures();
    sensors_DS18B20.getAddress(TempID_address, 0);
    TempID_Celsius = sensors_DS18B20.getTempC(TempID_address);
    ConvertAddressToHEX(TempID_address, TempID_short_id, TempID_long_id);
  } else {
    TempIDSensorExists = false;
  }
}


void VIAChameleonShield::Read_DS18B20_ID_only() {
  TempID_Celsius = -127;
  TempID_short_id[0] = '\0';
  TempID_long_id[0] = '\0';
  sensors_DS18B20.begin();
  if (sensors_DS18B20.getDeviceCount() > 0)  {
    TempIDSensorExists = true;

    sensors_DS18B20.getAddress(TempID_address, 0);

    ConvertAddressToHEX(TempID_address, TempID_short_id, TempID_long_id);
  } else {
    TempIDSensorExists = false;
  }
}




void VIAChameleonShield::ConvertAddressToHEX(byte *oneWireAdd, char *address_short, char *address_long) {
  address_short[0] = ASCIIHex(oneWireAdd[1] >> 4 & 15);
  address_short[1] = ASCIIHex(oneWireAdd[1] & 15);

  address_short[2] = ASCIIHex(oneWireAdd[7] >> 4 & 15);
  address_short[3] = ASCIIHex(oneWireAdd[7] & 15);

  address_long[0] = ASCIIHex(oneWireAdd[0] >> 4 & 15);
  address_long[1] = ASCIIHex(oneWireAdd[0] & 15);

  address_long[2] = ASCIIHex(oneWireAdd[1] >> 4 & 15);
  address_long[3] = ASCIIHex(oneWireAdd[1] & 15);

  address_long[4] = ASCIIHex(oneWireAdd[2] >> 4 & 15);
  address_long[5] = ASCIIHex(oneWireAdd[2] & 15);

  address_long[6] = ASCIIHex(oneWireAdd[3] >> 4 & 15);
  address_long[7] = ASCIIHex(oneWireAdd[3] & 15);

  address_long[8] = ASCIIHex(oneWireAdd[4] >> 4 & 15);
  address_long[9] = ASCIIHex(oneWireAdd[4] & 15);

  address_long[10] = ASCIIHex(oneWireAdd[5] >> 4 & 15);
  address_long[11] = ASCIIHex(oneWireAdd[5] & 15);

  address_long[12] = ASCIIHex(oneWireAdd[6] >> 4 & 15);
  address_long[13] = ASCIIHex(oneWireAdd[6] & 15);

  address_long[14] = ASCIIHex(oneWireAdd[7] >> 4 & 15);
  address_long[15] = ASCIIHex(oneWireAdd[7] & 15);
}


void VIAChameleonShield::setup_mux() {
  pinMode(MUXpinAddressA, OUTPUT);
  pinMode(MUXpinAddressB, OUTPUT);
  pinMode(MUXpinEnable, OUTPUT);
  digitalWrite(MUXpinEnable, HIGH);    //disable MUX
}

void VIAChameleonShield::SetMUXport(int PortNo) {
  // PortNo 1, 2 or 3 and -1 to disable
  if (PortNo == -1) {
    digitalWrite(MUXpinAddressA, LOW);
    digitalWrite(MUXpinAddressB, LOW);
    digitalWrite(MUXpinEnable, HIGH);   //disable MUX
  }
  else
  {
    // select MUX channel and enable
    digitalWrite(MUXpinAddressA, (PortNo - 1) & 01);
    digitalWrite(MUXpinAddressB, (PortNo - 1) & 10);
    digitalWrite(MUXpinEnable, LOW);
  }
  delay(10);
}

void VIAChameleonShield::ReadAllSensors() {
  Read_DS18B20();
  for (int PortNo = 1; PortNo < 4; PortNo++) {
    ReadSensor(PortNo);
  }
}

void VIAChameleonShield::ReadSensor(int PortNo) {
  //PortNo: 1, 2 or 3
  int i;
  float sensorVoltage;                  // Measured sensor voltage
  int sensorVoltageX, sensorVoltageY;
  float resistance;
  SetMUXport(PortNo);             //Select MUX sensor port



  for (i = 0; i < NUM_READS; i++) {  // Loop NUM_READS times

    //Forward current through sensor
    pinMode(sensorYAnalogPin, INPUT);
    pinMode(sensorXAnalogPin, OUTPUT);
    digitalWrite(sensorXAnalogPin, HIGH);                 // set the voltage supply on
    delayMicroseconds(stepwavedelayMicro);
    sensorVoltageY = analogRead(sensorYAnalogPin);   // read the sensor voltage takes 125us
    digitalWrite(sensorXAnalogPin, LOW);
    delayMicroseconds(stepwavedelayMicro);

    //Reverse current through sensor
    pinMode(sensorXAnalogPin, INPUT);
    pinMode(sensorYAnalogPin, OUTPUT);
    digitalWrite(sensorYAnalogPin, HIGH);
    delayMicroseconds(stepwavedelayMicro);
    sensorVoltageX = analogRead(sensorXAnalogPin);   // read the sensor voltage takes 125us
    digitalWrite(sensorYAnalogPin, LOW);
    delayMicroseconds(stepwavedelayMicro);
    _resistance_set[i]._sensorVoltageDouble = sensorVoltageY + sensorVoltageX;
  }
  SetMUXport(-1);   //disable MUX, this discontects all the sensors from the circuit to avoid galvantic loops

  myStats.clear();
  for (i = 0; i < NUM_READS; i++)  // Loop NUM_READS times
    myStats.add(_resistance_set[i]._sensorVoltageDouble);  // takes 100us to execute so take it out of the sensor loop
  sensorVoltage = myStats.average() / 2;
  if (sensorVoltage < 2)  //avoids divide by zero when open circuit and sensorVoltage=0, sensorVoltage=1 is Resistance=10220 which is also too high.
    resistance = 9999;
  else
    resistance = float( float(_knownResistor) * ( 1023 - sensorVoltage ) / sensorVoltage - _MUXresistance );
  resistance = float(round(resistance * 100)) / 100;  //Round to 2 decimal place

  Sensor[PortNo].resistance_raw = resistance;

  // Formula: Ra = R0 * (1 + (Temp - 22) * 0 .018 )

  if (TempID_Celsius > 5 && TempID_Celsius < 35)
    Sensor[PortNo].resistance_temp_calibrated = (float)Sensor[PortNo].resistance_raw * ((float)1 + ((float)TempID_Celsius - (float)22) * (float)0.018 );
  else
    Sensor[PortNo].resistance_temp_calibrated = Sensor[PortNo].resistance_raw;
  formatResistance(Sensor[PortNo].resistance_temp_calibrated, Sensor[PortNo].resistance_formatted);

  // DisplayReadingStats(PortNo, sensorVoltage, resistance);

} //void ReadSensor

/*
  void VIAChameleonShield::DisplayReadingStats(int PortNo, float sensorVoltage, float resistance)
  {
  Serial.print(F("Port:"));
  Serial.print(PortNo + 1);
  Serial.print(F(", Res: "));
  Serial.print(resistance, 4);
  Serial.print(F(",  CV:"));
  Serial.print((float)myStats.pop_stdev() / sensorVoltage, 4);
  Serial.print(F("\r\n"));
  }
*/

void VIAChameleonShield::formatResistance(float value, char *str) {
  int decimals;
  float rnd;
  if (value < 10)
    decimals = 1;
  else
    decimals = 0;

  if (value > 1000)
    rnd = float(int(value / 100)) * 100;
  else if (value > 200)
    rnd = float(int(value / 10)) * 10;
  else
    rnd = value;

  if (value != 9999)
    dtostrf(rnd, 6, decimals, str);
  else
    str[0] = (char)0;
  deblank(str);
}
byte VIAChameleonShield::ASCIIHex(byte fourbits) {
  if (fourbits < 10)
    return fourbits + 48;
  else
    return fourbits + 55;
}

char * VIAChameleonShield::deblank(char *str)
{
  char *out = str, *put = str;
  for (; *str != '\0'; ++str)
  {
    if (*str != ' ')
      *put++ = *str;
  }
  *put = '\0';
  return out;
}



void VIAChameleonShield::screendelay() {
  while (millis() < time_last_display + previousDelayAfter)
    delay(1);
}


void VIAChameleonShield::screen(int format, char line1[15], char line2[15], char line3[15], int delayAfter) {
  /*
    format 1 - 1 line - 20r, centered
    format 2 - 2 line even - 14r
    format 3 - 2 line uneven - 14r, 20r
    format 4 - 3 lines 20r
    format 5 - custom: sensor values

  */



  screendelay();
  u8g.firstPage();

  do {
    // u8g.setFont(u8g_font_fur14r);
    switch (format) {
      //       case 1: //format 1 - 1 line - 20r, centered
      //         u8g.setFont(u8g_font_fur20r);
      //         drawStr_center(40, line1);
      //         break;
      case 2: //format 2 - 2 line even - 14r
        u8g.setFont(u8g_font_fur14r);
        drawStr_center(28, line1);
        drawStr_center(55, line2);
        break;
      case 3: //format 3 - 2 line uneven - 14r, 20r
        u8g.setFont(u8g_font_fur14r);
        drawStr_center(26, line1);
        // u8g.setFont(u8g_font_fur20r);
        u8g.setFont(u8g_font_helvB18r);
        drawStr_center(59, line2);
        break;
      case 4: //format 4 - 3 lines 20r
        u8g.setFont(u8g_font_fur14r);
        drawStr_center(21, line1);
        drawStr_center(42, line2);
        drawStr_center(63, line3);
        break;

      case 5: //format 5 - custom: sensor values
        u8g.setFont(u8g_font_fur14r);
        for (uint8_t z = 1; z < 4; z++) {
          if (Sensor[z].LED != 0) {
            u8g.drawStr(1, 21 * (z), Sensor[z].LEDtext);
            if (Sensor[z].LED != 8) {
              u8g.drawStr(int(128 - u8g.getStrWidth(Sensor[z].resistance_formatted)), 21 * (z), Sensor[z].resistance_formatted);
            }
          }
        }
        break;

    }
  }
  while ( u8g.nextPage() );
  time_last_display = millis();
  previousDelayAfter = delayAfter;

}


void VIAChameleonShield::drawStr_center(int vertical, char line[15])
{
  u8g.drawStr(int(64 - u8g.getStrWidth(line) / 2), vertical, line);
}


void VIAChameleonShield::SetChameleonLED()
{
  // LED values: Undefined=0, Red=1, Green=2, Blue=4, White=7, Off=8
  for (uint8_t PortNo = 1; PortNo < 4; PortNo++) {
    if (Sensor[PortNo].resistance_temp_calibrated == 0)
      changeLED(PortNo, 0);     // "Undefined"
    else if (Sensor[PortNo].resistance_temp_calibrated < closedCicuitResistance || Sensor[PortNo].resistance_temp_calibrated > openCicuitResistance)
      changeLED(PortNo, 8);     // "Off"
    else if (Sensor[PortNo].resistance_temp_calibrated < switch1)
      changeLED(PortNo, 4);     //"Blue"
    else if (Sensor[PortNo].resistance_temp_calibrated < switch2)
      changeLED(PortNo, 2);     //"Green"
    else
      changeLED(PortNo, 1);     //"Red"


  }
}


void VIAChameleonShield::changeLED(int PortNo, uint8_t LED)
// LED values: Undefined=0, Red=1, Green=2, Blue=4, White=7, Off=8
{

  int i;
  int Red, Green, Blue;
  if (PortNo == -1) {          // All LED's
    for (i = 1; i < 4; i++) {
      Sensor[i].LED = LED;
    }
  }
  else {
    Sensor[PortNo].LED = LED;
  }
  for (i = 1; i < 4; i++) {
    Red = 0; Green = 0; Blue = 0;
    if (Sensor[i].LED == 8)
      strcpy(Sensor[i].LEDtext, "Off");
    if (int(Sensor[i].LED & 1) == 1) {
      Red = neopixel_brightness;
      strcpy(Sensor[i].LEDtext, "Red");
    }
    if (int(Sensor[i].LED & 2) == 2) {
      Green = neopixel_brightness;
      strcpy(Sensor[i].LEDtext, "Green");
    }
    if (int(Sensor[i].LED & 4) == 4) {
      Blue = neopixel_brightness;
      strcpy(Sensor[i].LEDtext, "Blue");
    }
    pixels.setPixelColor(i - 1, Red, Green, Blue);
  }
  delay(1);
  pixels.show(); // This sends the updated pixel color to the hardware.
}


void VIAChameleonShield::initialise_OLED() {
  Wire.beginTransmission(0x3C); // might be different for your display
  Wire.write(0x8d);
  Wire.write(0x14);
  Wire.endTransmission(); // stop transmitting
}

void VIAChameleonShield::initialise_shield() {
  Serial.begin(57600);
  pixels.begin(); // This initializes the NeoPixel library.
  initialise_OLED();  // turn on the OLED display
  screen(4, name_line1, name_line2, softwareVersion, 1000);
}
