#include <Sensors/SensingSystem.h>

SensingSystem::SensingSystem():IMUs_Avio(Sensors<IMU_Avio,double>(IMU_Avio_array,qty_IMU_Avio)),GPSs_Avio(Sensors<GPS_Avio,long>(GPS_Avio_array,qty_GPS_Avio)),BAROs_Avio(Sensors<BARO_Avio,float>(BARO_Avio_array,qty_BARO_Avio))
{
}

Sensors<IMU_Avio, double> *SensingSystem::getIMUs_Avio()
{
  return &IMUs_Avio;
}

uint8_t SensingSystem::getIMUs_Avio_rl_amount()
{
  return IMUs_Avio.get_real_amount();
}


Sensors<GPS_Avio, long> *SensingSystem::getGPSs_Avio()
{
  return &GPSs_Avio;
}

uint8_t SensingSystem::getGPSs_Avio_rl_amount()
{
  return GPSs_Avio.get_real_amount();
}


Sensors<BARO_Avio, float> *SensingSystem::getBAROs_Avio()
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

long* SensingSystem::getGPSs_meas(int scalingFactor)
{
  return GPSs_Avio.poll_process_ave_data(scalingFactor);
}

float* SensingSystem::getBAROs_meas(int scalingFactor)
{
  return BAROs_Avio.poll_process_ave_data(scalingFactor);
}