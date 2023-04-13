#include <Sensors/SensingSystem.h>

SensingSystem::SensingSystem()
:IMUs_Avio(Sensors<IMU_BNO055,double>(IMU_Avio_array,qty_IMU_Avio)),GPSs_Avio(Sensors<GPS_UBX9,double>(GPS_Avio_array,qty_GPS_Avio)),BAROs_Avio(Sensors<BARO_BMP280,float>(BARO_Avio_array,qty_BARO_Avio))
{}

bool SensingSystem::initAll()
{
    if(debug::info()) debug::Serial.println("!! Sensors Check/Init !!");

    bool result = false;

    if(debug::info()) debug::Serial.println("\n\n  ->IMUs Avio :");

    if(IMUs_Avio.initAll())
    { 
      result = true;
      if(debug::info()) debug::Serial.println(" ->IMUs Avio PASS");
    }
    
    if(debug::info()) debug::Serial.println("\n ->GPSs Avio :");

    if(GPSs_Avio.initAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->GPSs Avio PASS");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n  ->BAROs Avio :");

    if(BAROs_Avio.initAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->BAROs Avio PASS");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n\n");

    return result;
}

bool SensingSystem::wakeAll()
{
    if(debug::info()) debug::Serial.println("!! Sensors Wake !!");

    bool result = false;

    if(debug::info()) debug::Serial.println("\n\n  ->IMUs Avio :");

    if(IMUs_Avio.wakeAll())
    { 
      result = true;
      if(debug::info()) debug::Serial.println(" ->IMUs Avio awake");
    }
    
    if(debug::info()) debug::Serial.println("\n ->GPSs Avio :");

    if(GPSs_Avio.wakeAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->GPSs Avio awake");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n  ->BAROs Avio :");

    if(BAROs_Avio.wakeAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->BAROs Avio awake");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n\n");

    return result;
}

bool SensingSystem::sleepAll()
{
    if(debug::info()) debug::Serial.println("!! Sensors sleep !!");

    bool result = false;

    if(debug::info()) debug::Serial.println("\n\n  ->IMUs Avio :");

    if(IMUs_Avio.sleepAll())
    { 
      result = true;
      if(debug::info()) debug::Serial.println(" ->IMUs Avio asleep");
    }
    
    if(debug::info()) debug::Serial.println("\n ->GPSs Avio :");

    if(GPSs_Avio.sleepAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->GPSs Avio asleep");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n  ->BAROs Avio :");

    if(BAROs_Avio.sleepAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->BAROs Avio asleep");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n\n");

    return result;
}

bool SensingSystem::calibrateAll()
{
    if(debug::info()) debug::Serial.println("!! Sensors Calibration !!");

    bool result = false;

    if(debug::info()) debug::Serial.println("\n\n  ->IMUs Avio :");

    if(IMUs_Avio.calibrateAll())
    { 
      result = true;
      if(debug::info()) debug::Serial.println(" ->IMUs Avio Calibrated");
    }
    
    if(debug::info()) debug::Serial.println("\n ->GPSs Avio :");

    if(GPSs_Avio.calibrateAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->GPSs Avio Calibrated");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n  ->BAROs Avio :");

    if(BAROs_Avio.calibrateAll()) 
    {
      if(debug::info()) debug::Serial.println(" ->BAROs Avio Calibrated");
    }
    else
    {
      result = false;
    }

    if(debug::info()) debug::Serial.println("\n\n");

    return result;
}

Sensors<IMU_BNO055, double> *SensingSystem::getIMUs_Avio()
{
  return &IMUs_Avio;
}

uint8_t SensingSystem::getIMUs_Avio_rl_amount()
{
  return IMUs_Avio.get_real_amount();
}


Sensors<GPS_UBX9, double> *SensingSystem::getGPSs_Avio()
{
  return &GPSs_Avio;
}

uint8_t SensingSystem::getGPSs_Avio_rl_amount()
{
  return GPSs_Avio.get_real_amount();
}


Sensors<BARO_BMP280, float> *SensingSystem::getBAROs_Avio()
{
  return &BAROs_Avio;
}

uint8_t SensingSystem::getBAROs_Avio_rl_amount()
{
  return BAROs_Avio.get_real_amount();
}


double* SensingSystem::getIMUs_CG_meas()
{
  return IMUs_Avio.poll_process_ave_data();
}

uint16_t SensingSystem::getIMUs_CG_Hz()
{
  return IMUs_Avio.getRefreshRate();
}

double* SensingSystem::getGPSs_meas()
{
  return GPSs_Avio.poll_process_ave_data();
}

uint16_t SensingSystem::getGPSs_Hz()
{
  return GPSs_Avio.getRefreshRate();
}

float* SensingSystem::getBAROs_meas()
{
  return BAROs_Avio.poll_process_ave_data();
}

uint16_t SensingSystem::getBAROs_Hz()
{
  return BAROs_Avio.getRefreshRate();
}
