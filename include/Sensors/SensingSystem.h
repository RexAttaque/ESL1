#pragma once

#include <Sensors/Sensors.h>
#include <Sensors/IMU_Avio/ModuleLibs/IMU_Avio.h>
#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>
#include <Sensors/BARO_Avio/ModuleLibs/BARO_Avio.h>

IMU_Avio IMU1_Avio = IMU_Avio(55, 0x28);
IMU_Avio IMU2_Avio = IMU_Avio(55, 0x29);
const uint8_t qty_IMU_Avio = 2;
IMU_Avio* IMU_Avio_array;

GPS_Avio GPS1_Avio = GPS_Avio(Serial1);
const uint8_t qty_GPS_Avio = 1;
GPS_Avio* GPS_Avio_array;

BARO_Avio BARO1_Avio;
BARO_Avio BARO2_Avio;
const uint8_t qty_BARO_Avio = 2;
BARO_Avio* BARO_Avio_array;

class SensingSystem {
  private :
    Sensors<Adafruit_BNO055,float> IMUs_Avio;
    Sensors<ublox_gen9,float> GPSs_Avio;
    Sensors<Adafruit_BMP280,float> BAROs_Avio;
  
  public :

    SensingSystem();

    float* getIMUs_Avio_meas();

    uint8_t getIMUs_Avio_rl_amount();

    long* getGPSs_Avio_meas();

    uint8_t getGPSs_Avio_rl_amount();

    float* getBAROs_Avio_meas();

    uint8_t getBAROs_Avio_rl_amount();
};