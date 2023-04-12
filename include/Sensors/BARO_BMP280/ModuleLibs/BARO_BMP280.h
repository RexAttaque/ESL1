#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/BARO_BMP280/InterfaceLibs/Adafruit_BMP280.h>

BARO_BMP280 BARO1_Avio = BARO_BMP280(BMP280_ADDRESS);
BARO_BMP280 BARO2_Avio = BARO_BMP280(BMP280_ADDRESS_ALT);
const uint8_t qty_BARO_Avio = 2;
BARO_BMP280* BARO_Avio_array;

const uint16_t default_BARO_BMP280_Hz = 115; //check paragraph 3.8.1 and 3.8.2 of the datasheet and the default constructor
const uint8_t BARO_Avio_varAmount = 2; //P,T (Pa ; °C)

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
    BARO_BMP280(uint8_t address, Adafruit_BMP280::sensor_sampling over_samp_T = Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::sensor_sampling over_samp_P = Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::sensor_filter filter = Adafruit_BMP280::FILTER_X2, Adafruit_BMP280::standby_duration stb_time = Adafruit_BMP280::STANDBY_MS_1, Adafruit_BMP280::sensor_mode mode = Adafruit_BMP280::MODE_FORCED);
    
    bool init();
    bool goLive(); 
    bool goIdle();

    float* getMeas();
};