#include <Sensors/SensingSystem.h>

SensingSystem::SensingSystem():IMUs_Avio(Sensors<IMU_BNO055,double>(IMU_Avio_array,qty_IMU_Avio)),GPSs_Avio(Sensors<GPS_UBX9,long>(GPS_Avio_array,qty_GPS_Avio)),BAROs_Avio(Sensors<BARO_BMP280,float>(BARO_Avio_array,qty_BARO_Avio))
{
}

uint32_t* SensingSystem::initAll()
{
    uint32_t initFlags[SS_const::amount_sensor_arrays] = {IMUs_Avio.initAll(), GPSs_Avio.initAll(), BAROs_Avio.initAll()};
    return initFlags;
}

Sensors<IMU_BNO055, double> *SensingSystem::getIMUs_Avio()
{
  return &IMUs_Avio;
}

uint8_t SensingSystem::getIMUs_Avio_rl_amount()
{
  return IMUs_Avio.get_real_amount();
}


Sensors<GPS_UBX9, long> *SensingSystem::getGPSs_Avio()
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


double* SensingSystem::getIMUs_CG_meas(int scalingFactor)
{
  return IMUs_Avio.poll_process_ave_data(scalingFactor);
}

uint16_t SensingSystem::getIMUs_CG_Hz()
{
  return IMUs_Avio.getRefreshRate();
}

long* SensingSystem::getGPSs_meas(int scalingFactor)
{
  return GPSs_Avio.poll_process_ave_data(scalingFactor);
}

uint16_t SensingSystem::getGPSs_Hz()
{
  return GPSs_Avio.getRefreshRate();
}

float* SensingSystem::getBAROs_meas(int scalingFactor)
{
  return BAROs_Avio.poll_process_ave_data(scalingFactor);
}

uint16_t SensingSystem::getBAROs_Hz()
{
  return BAROs_Avio.getRefreshRate();
}
