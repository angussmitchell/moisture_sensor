/*
  VIA Chameleon_Shield Arduino library
  File: VIAChameleonShieldExample.ino
  Author: Matthew Driver
  Edited: 1 June 2019
*/

#ifndef VIAChameleonShield_h
#define VIAChameleonShield_h

#include "Arduino.h"

#define DS18B20_Pin              8
#define MUXpinAddressA           5
#define MUXpinAddressB           6
#define MUXpinEnable             A2
#define sensorYAnalogPin         A0
#define sensorXAnalogPin         A1
#define NUM_READS                20    // Number of sensor reads for sorting before obtaining mean value
#define stepwavedelayMicro       250   // microsecond: pulse width of Vcc uC applied to sensor, reading voltage at end of pulse
#define LEDSdataPin              7
#define NUMPIXELS                3
#define neopixel_brightness 10  //127 // 255 for full brightness, 10 low for development
#include <Statistic.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define name_line1 "Chameleon"
#define name_line2 "Shield"
#define softwareVersion "v0.1"



class VIAChameleonShield
{
  public:
#define switch1 4
#define switch2 40
#define openCicuitResistance 4000
    //#define quickCheckOpenCicuitResistance 3000
#define closedCicuitResistance 0.1

    float TempID_Celsius;
    boolean TempIDSensorExists = false;
    byte TempID_address[8];
    char TempID_short_id[5];
    char TempID_long_id[17];

    uint32_t time_last_display = 0;
    uint16_t previousDelayAfter = 0;

    typedef struct {
      float resistance_raw;   // in k Ohms             // Sensor[PortNo].resistance_raw
      uint8_t LED;             // LED colour: Red=1, Green=2, Blue=4
      char LEDtext[6];        // Store text: Red, Blue, Green, Off
      float resistance_temp_calibrated;    // in k Ohms                                 // Sensor[PortNo].resistance_temp_calibrated
      char resistance_formatted[10];                                            // Sensor[PortNo].resistance_formatted
    } Sensor_struct;

    Sensor_struct Sensor[4];     //Array of three sensor values and LED colours

    VIAChameleonShield();
    void Read_DS18B20();
    void Read_DS18B20_ID_only();
    void ReadAllSensors();
    void ReadSensor(int PortNo);
    void screen(int format, char line1[15], char line2[15], char line3[15], int delayAfter);
    void screendelay();
    void drawStr_center(int vertical, char line[15]);
    void SetChameleonLED();
    void changeLED(int PortNo, uint8_t LED);
    void initialise_OLED();
    void initialise_shield();
  private:

    const float _MUXresistance = 0.10;
    const float _knownResistor = 10.0;       // Constant value of known resistor in k Ohms
    typedef struct {
      int _sensorVoltageDouble;
    } _resistance_set_struct;
    _resistance_set_struct _resistance_set[NUM_READS];        // Array of resistances to be collected before selecting the median value

    void setup_mux();
    void SetMUXport(int PortNo);
    void formatResistance(float value, char *str);
    void ConvertAddressToHEX(byte *oneWireAdd, char *address_short, char *address_long);
    byte ASCIIHex(byte fourbits);
    char * deblank(char *str);

};

#endif
