#pragma once

#include <Sensors/Sensors.h>
#include <Sensors/IMU_Avio/ModuleLibs/IMU_Avio.h>
#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>
#include <Sensors/BARO_Avio/ModuleLibs/BARO_Avio.h>
class SensingSystem {
  private :
    Sensors<IMU_Avio,double> IMUs_Avio;
    Sensors<GPS_Avio,long> GPSs_Avio;
    Sensors<BARO_Avio,float> BAROs_Avio;
  
  public :

    SensingSystem();

    Sensors<IMU_Avio,double>* getIMUs_Avio();
    uint8_t getIMUs_Avio_rl_amount();

    Sensors<GPS_Avio,long>* getGPSs_Avio(); 
    uint8_t getGPSs_Avio_rl_amount();

    Sensors<BARO_Avio,float>* getBAROs_Avio();
    uint8_t getBAROs_Avio_rl_amount();
    
    //get the measurements at CG for use in the Kalman (that includes backup altitude computation from BARO data)
    double* getIMUs_CG_meas(int scalingFactor); //Data combination from the entire sensing system to obtain IMU data at CG for use in the Kalman filter
    long* getGPSs_meas(int scalingFactor); //Data combination from the entire sensing system to obtain GPS data for use in the Kalman filter
    float* getBAROs_meas(int scalingFactor); //Data combination from the entire sensing system to obtain BARO data for use in the Kalman filter (as backup altitude computation)
};