#include <Sensors/BARO_BMP280/ModuleLibs/BARO_BMP280.h>

BARO_BMP280::BARO_BMP280(uint8_t address, Adafruit_BMP280::sensor_sampling over_samp_T, Adafruit_BMP280::sensor_sampling over_samp_P, Adafruit_BMP280::sensor_filter filter, Adafruit_BMP280::standby_duration stb_time, Adafruit_BMP280::sensor_mode mode)
:Base_Sensor(BARO_Avio_varAmount),_address(address),_over_samp_T(over_samp_T),_over_samp_P(over_samp_P),_filter(filter),_stb_time(stb_time),_mode(mode)
{}
    
bool BARO_BMP280::init()
{
  bool status = BARO.begin(_address);

  if(status)
  {
    BARO.setSampling(_mode,_over_samp_T,_over_samp_P,_filter,_stb_time); 
  }

  return status;
}

float* BARO_BMP280::getMeas()
{
  reallocateMemory();

  meas[1] = BARO.readPressure();
  meas[2] = BARO.readTemperature();

  return meas;
}