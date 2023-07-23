#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/Sensors.h>
#include <Sensors/IMU_BNO055/ModuleLibs/IMU_BNO055.h>
#include <Sensors/GPS_UBX9/ModuleLibs/GPS_UBX9.h>
#include <Sensors/BARO_BMP280/ModuleLibs/BARO_BMP280.h>

namespace SS_const {
  const String debug_ID = "SS";
  const uint8_t debug_lvl = debugLevel::FULL;

  const uint8_t amount_sensor_arrays = 3;
};

class SensingSystem {
  private :
    fault_debug debug_SS = fault_debug(SS_const::debug_ID, SS_const::debug_lvl); //Debug object for the SS module

    //Make sure all of the sensors within sensors objects of one "group" use the same refresh rate 
    //(all of the sensors that contribute to one set of measurement must have the same refresh rate)
    Sensors<IMU_BNO055,double> IMUs_Avio;
    Sensors<GPS_UBX9,double> GPSs_Avio;
    Sensors<BARO_BMP280,float> BAROs_Avio;

  public :

    SensingSystem(IMU_BNO055* IMU_Avio_array, uint8_t qty_IMU_Avio, GPS_UBX9* GPS_Avio_array, uint8_t qty_GPS_Avio, BARO_BMP280* BARO_Avio_array, uint8_t qty_BARO_Avio);

    bool initAll();
    bool wakeAll();
    bool sleepAll();
    bool calibrateAll();

    Sensors<IMU_BNO055,double>* getIMUs_Avio();
    uint8_t getIMUs_Avio_rl_amount();

    Sensors<GPS_UBX9,double>* getGPSs_Avio(); 
    uint8_t getGPSs_Avio_rl_amount();

    Sensors<BARO_BMP280,float>* getBAROs_Avio();
    uint8_t getBAROs_Avio_rl_amount();
    
    //get the measurements at CG for use in the Kalman (that includes backup altitude computation from BARO data)
    double* getIMUs_CG_meas(); //Data combination from the entire sensing system to obtain IMU data at CG for use in the Kalman filter
    uint16_t getIMUs_CG_Hz(); //get the refresh rate of the IMUs that from the "CG" group

    double* getGPSs_meas(); //Data combination from the entire sensing system to obtain GPS data for use in the Kalman filter
    uint16_t getGPSs_Hz(); //get the refresh rate of this group of GPSs

    float* getBAROs_meas(); //Data combination from the entire sensing system to obtain BARO data for use in Barometric System
    uint16_t getBAROs_Hz(); //get the refresh rate of this group of BAROs
};