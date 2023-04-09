#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>

GPS_Avio::GPS_Avio(HardwareSerial HWSerial):Base_Sensor(GPS_varAmount),GPS(ublox_gen9(HWSerial))
{}

long* GPS_Avio::getMeas()
{
  return GPS.getPOSECEF();
}