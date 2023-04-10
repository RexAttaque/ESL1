#pragma once

#include <Arduino.h>
#include <faultCodes.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

GPS_UBX9 GPS1_Avio = GPS_UBX9();
const uint8_t qty_GPS_Avio = 1;
GPS_UBX9* GPS_Avio_array;

const uint16_t default_GPS_UBX9_Hz = 25; //default GPS refresh rate (Hz)
const uint8_t GPS_varAmount = 5; //time,x,y,z,CEP (ms ; cm)

class GPS_UBX9 : public Base_Sensor<long> {
  private :
    ublox_gen9 GPS;
  
  public :
  
    GPS_UBX9(HardwareSerial HWSerial = Serial1, long Baud = 115200, uint8_t Pltfrm_Model = 8, uint16_t refreshRate = default_GPS_UBX9_Hz, uint8_t Nav_Rate = 1, uint8_t config_Level = 7, long default_Baud = 38400, bool buffered_POS = false, bool NMEA_USB = false, long USB_Baud = 115200, bool NMEA_UART1 = false, uint8_t Stop_Bits = 1);

    bool init();

    uint8_t getStatus();

    long* getMeas();
};