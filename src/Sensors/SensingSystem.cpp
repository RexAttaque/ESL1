#include <Sensors/SensingSystem.h>

SensingSystem::SensingSystem(IMU_BNO055* IMU_Avio_array, uint8_t qty_IMU_Avio, GPS_UBX9* GPS_Avio_array, uint8_t qty_GPS_Avio, BARO_BMP280* BARO_Avio_array, uint8_t qty_BARO_Avio)
:fault_debug(SS_const::debug_ID, SS_const::debug_lvl),IMUs_Avio(Sensors<IMU_BNO055,double>(IMU_Avio_array,qty_IMU_Avio)),GPSs_Avio(Sensors<GPS_UBX9,double>(GPS_Avio_array,qty_GPS_Avio)),BAROs_Avio(Sensors<BARO_BMP280,float>(BARO_Avio_array,qty_BARO_Avio))
{}

bool SensingSystem::initAll()
{
    d_println(debugLevel::INFO, "!! Sensors Check/Init Start !!\n\n", "initAll()");

    bool result = true;

    d_println(debugLevel::INFO, "  ->IMUs Avio :");

    if(IMUs_Avio.initAll())
    { 
      d_println(debugLevel::INFO, " ->IMUs Avio PASS\n");
    }
    else
    {
      result = false;
    }
    
    d_println(debugLevel::INFO, " ->GPSs Avio :");

    if(GPSs_Avio.initAll()) 
    {
      d_println(debugLevel::INFO, " ->GPSs Avio PASS\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "  ->BAROs Avio :");

    if(BAROs_Avio.initAll()) 
    {
      d_println(debugLevel::INFO, " ->BAROs Avio PASS\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "\n\n!! Sensors Check/Init End (result = " + String(result) + ") !!");

    return result;
}

bool SensingSystem::wakeAll()
{
    d_println(debugLevel::INFO, "!! Sensors Wake Start !!\n\n", "wakeAll()");

    bool result = true;

    d_println(debugLevel::INFO, "  ->IMUs Avio :");

    if(IMUs_Avio.wakeAll())
    { 
      d_println(debugLevel::INFO, " ->IMUs Avio awake\n");
    }
    else
    {
      result = false;
    }
    
    d_println(debugLevel::INFO, " ->GPSs Avio :");

    if(GPSs_Avio.wakeAll()) 
    {
      d_println(debugLevel::INFO, " ->GPSs Avio awake\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, " ->BAROs Avio :");

    if(BAROs_Avio.wakeAll()) 
    {
      d_println(debugLevel::INFO, " ->BAROs Avio awake\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "\n\n!! Sensors Wake End (result = " + String(result) + ") !!");

    return result;
}

bool SensingSystem::sleepAll()
{
    d_println(debugLevel::INFO, "!! Sensors sleep !!\n\n", "sleepAll()");

    bool result = true;

    d_println(debugLevel::INFO, " ->IMUs Avio :");

    if(IMUs_Avio.sleepAll())
    { 
      d_println(debugLevel::INFO, " ->IMUs Avio asleep\n");
    }
    else
    {
      result = false;
    }
    
    d_println(debugLevel::INFO, " ->GPSs Avio :");

    if(GPSs_Avio.sleepAll()) 
    {
      d_println(debugLevel::INFO, " ->GPSs Avio asleep\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, " ->BAROs Avio :");

    if(BAROs_Avio.sleepAll()) 
    {
      d_println(debugLevel::INFO, " ->BAROs Avio asleep\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "\n\n!! Sensors sleep End (result = " + String(result) + ") !!");

    return result;
}

bool SensingSystem::calibrateAll()
{
    d_println(debugLevel::INFO, "!! Sensors Calibration Start !!\n\n", "calibrateAll()");

    bool result = true;

    d_println(debugLevel::INFO, "  ->IMUs Avio :");

    if(IMUs_Avio.calibrateAll())
    { 
      d_println(debugLevel::INFO, " ->IMUs Avio Calibrated\n");
    }
    else
    {
      result = false;
    }
    
    d_println(debugLevel::INFO, " ->GPSs Avio :");

    if(GPSs_Avio.calibrateAll()) 
    {
      d_println(debugLevel::INFO, " ->GPSs Avio Calibrated\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "  ->BAROs Avio :");

    if(BAROs_Avio.calibrateAll()) 
    {
      d_println(debugLevel::INFO, " ->BAROs Avio Calibrated\n");
    }
    else
    {
      result = false;
    }

    d_println(debugLevel::INFO, "\n\n!! Sensors Calibration End (result = " + String(result) + ") !!");

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
