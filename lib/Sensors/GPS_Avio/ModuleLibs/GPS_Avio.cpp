#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>

GPS_Avio::GPS_Avio(HardwareSerial HWSerial):Base_Sensor(GPS_varAmount),GPS(ublox_gen9(HWSerial))
{}

uint8_t GPS_Avio::getStatus()
{
    return GPS.getNavFixStatus();
}

long* GPS_Avio::getMeas()
{
  return GPS.getPOSECEF();
}