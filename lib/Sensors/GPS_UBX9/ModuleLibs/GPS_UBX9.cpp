#include <Sensors/GPS_UBX9/ModuleLibs/GPS_UBX9.h>

GPS_UBX9::GPS_UBX9(HardwareSerial HWSerial, long Baud, uint8_t Pltfrm_Model, uint16_t refreshRate, uint8_t Nav_Rate, uint8_t config_Level, long default_Baud, bool buffered_POS, bool NMEA_USB, long USB_Baud, bool NMEA_UART1, uint8_t Stop_Bits)
:Base_Sensor(UBX9_const::varAmount, refreshRate),GPS(ublox_gen9(HWSerial, Baud, Pltfrm_Model, (uint16_t) 1000/refreshRate, Nav_Rate, config_Level, default_Baud, buffered_POS, NMEA_USB, USB_Baud, NMEA_UART1, Stop_Bits))
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

bool GPS_UBX9::calibrate()
{
  uint8_t attempts = 0;
  uint8_t fixData = GPS.getNavFixStatus(); 
  
  if(debug::info()) Serial.println("      --->Instructions : Place the GPS antenna near a window, preferably outside and away from cover");
    

  while(attempts<UBX9_const::maxCalibrationAttemps && fixData<UBX9_const::minFixStatus && fixData>UBX9_const::maxFixStatus)
  {
    if(debug::info()) Serial.println("        ---->UBX9 GPS does not yet have the required fix (result is " + String(fixData) + "), waiting " + String(UBX9_const::timeBetweenCalibrations/1000) + "s...");
    fixData = GPS.getNavFixStatus();
    attempts++;
    delay(UBX9_const::timeBetweenCalibrations);
  }

  if(attempts<UBX9_const::maxCalibrationAttemps)
  {
    if(debug::info()) Serial.println("      --->Got required UBX9 GPS fix, waiting " + String(UBX9_const::calibrationSettleTime/60000) + "s for it to settle ...");
    delay(UBX9_const::calibrationSettleTime);
    return true;
  }
  else
  {
    if(debug::info()) Serial.println("      --->Attempts at getting required UBX9 GPS fix failed");
    return false;
  }
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