#include <Sensors/BARO_BMP280/ModuleLibs/BARO_BMP280.h>

BARO_BMP280::BARO_BMP280(uint8_t address, Adafruit_BMP280::sensor_sampling over_samp_T, Adafruit_BMP280::sensor_sampling over_samp_P, Adafruit_BMP280::sensor_filter filter, Adafruit_BMP280::standby_duration stb_time, Adafruit_BMP280::sensor_mode mode)
:Base_Sensor(BARO_Avio_varAmount, default_BARO_BMP280_Hz),_address(address),_over_samp_T(over_samp_T),_over_samp_P(over_samp_P),_filter(filter),_stb_time(stb_time),_mode(mode)
{}
    
bool BARO_BMP280::init()
{
  bool status = BARO.begin(_address); //begin already checks for correct detection of the sensor and I2C start

  if(status)
  {
    if(debug::info()) 
    {
      debug::Serial.println("      --->BMP280 Active and available on I2C...");
      debug::Serial.println("      --->Setting sampling mode...");
    }

    BARO.setSampling(_mode,_over_samp_T,_over_samp_P,_filter,_stb_time); 
    if(_mode == Adafruit_BMP280::MODE_FORCED) 
    {
      if(debug::info()) debug::Serial.println("      --->BMP280 in force mode, taking first measurement...");
      BARO.takeForcedMeasurement(); //take a first measurement to sort of kick start the sensor if it's in Force mode, otherwise it'll do it on it's own
    }  
    else
    {
      if(debug::info()) debug::Serial.println("      --->BMP280 in normal mode, letting it take it's first measurement...");
      delay(5000); //Give the sensor time, whatever the parameters, to take at least one measurement
    }
    //check that the first measurement is valid ?

    return true;
  }

  return false;
}

bool BARO_BMP280::goLive()
{
  if(_mode == Adafruit_BMP280::MODE_FORCED) 
  {
    return BARO.takeForcedMeasurement(); //BMP280 goes live when asked to in forced mode
  }  
  else
  {
    return true; //otherwise it always goes between live and idle on it's own
  }
}

bool BARO_BMP280::goIdle()
{
  return true; //BMP280 goes to sleep on it's own so this kind of always is true
}

float* BARO_BMP280::getMeas()
{
  reallocateMemory();

  meas[1] = BARO.readPressure();
  meas[2] = BARO.readTemperature();

  if(_mode == Adafruit_BMP280::MODE_FORCED) 
  {
    BARO.takeForcedMeasurement(true); //Start the measurement process for next time, skip delays inside takeForceMeasurements as this is running inside a tightly timed loop
  }  

  return meas;
}