#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/BARO_BMP280/InterfaceLibs/Adafruit_BMP280.h>

namespace BMP280_const {
  const String debug_ID = "BMP280";
  const uint8_t debug_lvl = debugLevel::FULL;

  const Adafruit_BMP280::sensor_sampling def_oversampT = Adafruit_BMP280::SAMPLING_X1; //default oversample for temperature
  const Adafruit_BMP280::sensor_sampling def_oversampP = Adafruit_BMP280::SAMPLING_X2; //default oversample for pressure
  const Adafruit_BMP280::sensor_filter def_filt = Adafruit_BMP280::FILTER_X2; //default filter setting
  const Adafruit_BMP280::standby_duration def_stb = Adafruit_BMP280::STANDBY_MS_1; //default standby time
  const Adafruit_BMP280::sensor_mode def_m = Adafruit_BMP280::MODE_FORCED; //default BMP280 measurement mode

  const uint16_t default_BARO_BMP280_Hz = 115; //check paragraph 3.8.1 and 3.8.2 of the datasheet and the default constructor
  const uint8_t BARO_Avio_varAmount = 2; //P,T (Pa ; °C)
  const float C_2_K = 273.15; //conversion from °C to K
};

fault_debug debug_BMP280 = fault_debug(BMP280_const::debug_ID, BMP280_const::debug_lvl); //Debug object for the BMP280 module

class BARO_BMP280 : public Base_Sensor<float> {
  private :
    Adafruit_BMP280 BARO;
    uint8_t _address;

    Adafruit_BMP280::sensor_mode _mode;
    Adafruit_BMP280::sensor_sampling _over_samp_T;
    Adafruit_BMP280::sensor_sampling _over_samp_P; 
    Adafruit_BMP280::sensor_filter _filter; 
    Adafruit_BMP280::standby_duration _stb_time;
  
  public :
    //WARNING : Make sure that the refresh rate with the given config is not lower than the bounding refresh rate
    BARO_BMP280(uint8_t address, TwoWire* i2c_bus = &Wire, Adafruit_BMP280::sensor_sampling over_samp_T = BMP280_const::def_oversampT, Adafruit_BMP280::sensor_sampling over_samp_P = BMP280_const::def_oversampP, Adafruit_BMP280::sensor_filter filter = BMP280_const::def_filt, Adafruit_BMP280::standby_duration stb_time = BMP280_const::def_stb, Adafruit_BMP280::sensor_mode mode = BMP280_const::def_m);
    
    bool init();
    bool calibrate();

    bool goLive(); 
    bool goIdle();

    bool getStatus();

    float* getMeas();
};