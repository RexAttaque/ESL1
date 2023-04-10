#pragma once

#include <Sensors/Base_Sensor.h>
#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

GPS_UBX9 GPS1_Avio = GPS_UBX9(Serial1);
const uint8_t qty_GPS_Avio = 1;
GPS_UBX9* GPS_Avio_array;

const uint8_t GPS_varAmount = 5; //time,x,y,z,CEP (ms ; cm)

class GPS_UBX9 : public Base_Sensor<long> {
  private :
    ublox_gen9 GPS;
  
  public :
  
    GPS_UBX9(HardwareSerial HWSerial);

    bool init();

    uint8_t getStatus();

    long* getMeas();
};