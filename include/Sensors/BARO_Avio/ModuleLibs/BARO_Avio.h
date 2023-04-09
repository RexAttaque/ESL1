#pragma once

#include <Arduino.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/BARO_Avio/InterfaceLibs/Adafruit_BMP280.h>

BARO_Avio BARO1_Avio = BARO_Avio(BMP280_ADDRESS);
BARO_Avio BARO2_Avio = BARO_Avio(BMP280_ADDRESS_ALT);
const uint8_t qty_BARO_Avio = 2;
BARO_Avio* BARO_Avio_array;

const uint8_t BARO_Avio_varAmount = 2; //P,T (Pa ; °C)

class BARO_Avio : public Base_Sensor<float> {
  private :
    Adafruit_BMP280 BARO;
    uint8_t _address;

    Adafruit_BMP280::sensor_mode _mode;
    Adafruit_BMP280::sensor_sampling _over_samp_T;
    Adafruit_BMP280::sensor_sampling _over_samp_P; 
    Adafruit_BMP280::sensor_filter _filter; 
    Adafruit_BMP280::standby_duration _stb_time;
  
  public :
    BARO_Avio(uint8_t address, Adafruit_BMP280::sensor_sampling over_samp_T = Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::sensor_sampling over_samp_P = Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::sensor_filter filter = Adafruit_BMP280::FILTER_X2, Adafruit_BMP280::standby_duration stb_time = Adafruit_BMP280::STANDBY_MS_1, Adafruit_BMP280::sensor_mode mode = Adafruit_BMP280::MODE_FORCED);
    
    bool BARO_init();

    float* getMeas();
};