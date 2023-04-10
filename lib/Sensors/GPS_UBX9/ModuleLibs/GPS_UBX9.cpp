#include <Sensors/GPS_UBX9/ModuleLibs/GPS_UBX9.h>

GPS_UBX9::GPS_UBX9(HardwareSerial HWSerial):Base_Sensor(GPS_varAmount),GPS(ublox_gen9(HWSerial))
{}

bool GPS_UBX9::init()
{
  return GPS.initGPS();
}

uint8_t GPS_UBX9::getStatus()
{
    return GPS.getNavFixStatus();
}

long* GPS_UBX9::getMeas()
{
  freeMeasMemory();
  return GPS.getPOSECEF();
}