#include <Sensors/GPS_UBX9/ModuleLibs/GPS_UBX9.h>

GPS_UBX9::GPS_UBX9(HardwareSerial HWSerial, long Baud, uint8_t Pltfrm_Model, uint16_t refreshRate, uint8_t Nav_Rate, uint8_t config_Level, long default_Baud, bool buffered_POS, bool NMEA_USB, long USB_Baud, bool NMEA_UART1, uint8_t Stop_Bits)
:Base_Sensor(UBX9_const::varAmount, refreshRate),GPS(ublox_gen9(HWSerial, Baud, Pltfrm_Model, (uint16_t) 1000/refreshRate, Nav_Rate, config_Level, default_Baud, buffered_POS, NMEA_USB, USB_Baud, NMEA_UART1, Stop_Bits))
{}

ublox_gen9* GPS_UBX9::getSensor()
{
  return &GPS;
}

bool GPS_UBX9::init()
{
  return GPS.initGPS();
}

bool GPS_UBX9::calibrate()
{
  uint8_t attempts = 0;

  debug_GPSUBX9.println(debugLevel::INFO, "      --->Instructions : Place the GPS antenna near a window, preferably outside and away from cover", "calibrate()");
    
  while(attempts<UBX9_const::maxCalibrationAttempts && getStatus())
  {
    debug_GPSUBX9.println(debugLevel::INFO, "        ---->UBX9 GPS does not yet have the required fix (result is " + String(fixType) + "), waiting " + String(UBX9_const::timeBetweenCalibrations/1000) + "s...");
    attempts++;
    delay(UBX9_const::timeBetweenCalibrations);
  }

  if(attempts<UBX9_const::maxCalibrationAttempts)
  {
    debug_GPSUBX9.println(debugLevel::INFO, "      --->Got required UBX9 GPS fix, waiting " + String(UBX9_const::calibrationSettleTime/60000) + "s for it to settle ...");
    delay(UBX9_const::calibrationSettleTime);
    return true;
  }
  else
  {
    debug_GPSUBX9.println(debugLevel::INFO, "      --->Attempts at getting required UBX9 GPS fix failed");
    return false;
  }
}

bool GPS_UBX9::goLive()
{
  return GPS.highPower();
}

bool GPS_UBX9::goIdle()
{
  return GPS.lowPower();
}

bool GPS_UBX9::getStatus()
{
  fixType = GPS.getNavFixStatus();
  return fixType<UBX9_const::minFixStatus && fixType>UBX9_const::maxFixStatus;
}

double* GPS_UBX9::getMeas()
{
  reallocateMemory();

  long* data = GPS.getPOSECEF();
  
  for(uint8_t i=0; i<_varAmount; i++)
  {
    meas[i] = (double) data[i]*UBX9_const::measFactors[i]; //conversion to the proper unit using factors
  }

  delete[] data;

  return meas;
}