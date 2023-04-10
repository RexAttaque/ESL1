#pragma once

#include <Sensors/Base_Sensor.h>
#include <Sensors/GPS_Avio/InterfaceLibs/ubloxGen9.h>

GPS_Avio GPS1_Avio = GPS_Avio(Serial1);
const uint8_t qty_GPS_Avio = 1;
GPS_Avio* GPS_Avio_array;

const uint8_t GPS_varAmount = 5; //time,x,y,z,CEP (ms ; cm)

class GPS_Avio : public Base_Sensor<long> {
  private :
    ublox_gen9 GPS;
  
  public :
  
    GPS_Avio(HardwareSerial HWSerial);

    uint8_t getStatus();

    long* getMeas();
};