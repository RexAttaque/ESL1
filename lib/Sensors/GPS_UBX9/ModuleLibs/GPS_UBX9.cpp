#include <Sensors/GPS_UBX9/ModuleLibs/GPS_UBX9.h>

GPS_UBX9::GPS_UBX9(HardwareSerial HWSerial, long Baud, uint8_t Pltfrm_Model, uint16_t refreshRate, uint8_t Nav_Rate, uint8_t config_Level, long default_Baud, bool buffered_POS, bool NMEA_USB, long USB_Baud, bool NMEA_UART1, uint8_t Stop_Bits)
:Base_Sensor(GPS_varAmount, refreshRate),GPS(ublox_gen9(HWSerial, Baud, Pltfrm_Model, (uint16_t) 1000/refreshRate, Nav_Rate, config_Level, default_Baud, buffered_POS, NMEA_USB, USB_Baud, NMEA_UART1, Stop_Bits))
{}

bool GPS_UBX9::init()
{
  return GPS.initGPS();
}

bool GPS_UBX9::goLive()
{
  return GPS.highPower();
}

bool GPS_UBX9::goIdle()
{
  return GPS.lowPower();
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