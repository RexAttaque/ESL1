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
    Sensors<IMU_Avio,float> IMUs_Avio;
    Sensors<GPS_Avio,float> GPSs_Avio;
    Sensors<BARO_Avio,float> BAROs_Avio;
  
  public :

    SensingSystem();

    Sensors<IMU_Avio,float>* getIMUs_Avio();
    uint8_t getIMUs_Avio_rl_amount();

    Sensors<GPS_Avio,float>* getGPSs_Avio(); 
    uint8_t getGPSs_Avio_rl_amount();

    Sensors<BARO_Avio,float> getBAROs_Avio();
    uint8_t getBAROs_Avio_rl_amount();
    
    //get the measurements at CG for use in the Kalman (that includes backup altitude computation from BARO data)
    float* getIMUs_CG_meas(int scalingFactor); //Data combination from the entire sensing system to obtain IMU data at CG for use in the Kalman filter
    long* getGPSs_meas(int scalingFactor); //Data combination from the entire sensing system to obtain GPS data for use in the Kalman filter
    float* getBAROs_meas(int scalingFactor); //Data combination from the entire sensing system to obtain BARO data for use in the Kalman filter (as backup altitude computation)
};