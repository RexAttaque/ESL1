#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>

GPS_Avio::GPS_Avio(HardwareSerial HWSerial):GPS(ublox_gen9(HWSerial))
{
  varAmount = GPS_varAmount; 
};

uint8_t GPS_Avio::get_var_amount()
{
  return varAmount;
}

float* GPS_Avio::getMeas()
{
  
}