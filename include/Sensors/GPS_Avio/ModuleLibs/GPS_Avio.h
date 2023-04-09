#pragma once

#include <Sensors/GPS_Avio/InterfaceLibs/ubloxGen9.h>

const uint8_t GPS_varAmount = 3; //x,y,z (cm)

class GPS_Avio {
  private :
    ublox_gen9 GPS;
    uint8_t varAmount;
  
  public :
  
    GPS_Avio(HardwareSerial HWSerial);

    uint8_t get_var_amount();

    float* getMeas();
};