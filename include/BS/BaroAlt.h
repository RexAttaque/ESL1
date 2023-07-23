#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/SensingSystem.h>

namespace BS_const {
  const String debug_ID = "BS";
  const uint8_t debug_lvl = debugLevel::FULL;

  const uint8_t atmo_model_slices = 7;
  const unsigned long delayBeforeInitMeas = 300000; //in ms

  const float g = 9.81; // m/s^2
  const float r = 287.03; // J/kg/K
};

class BS_obj
{
  private:
    fault_debug debug_BS = fault_debug(BS_const::debug_ID, BS_const::debug_lvl);

    double altitude; //Stores current altitude in m
    SensingSystem* BS_components;

    uint16_t refresh_BARO;
    unsigned long time_BARO;
    double delta_t;

    float P; //Stores current pressure Pa
    float T; //Stores current temperature K
    
    //double Tz[BS_const::atmo_model_slices+1] = {-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002, 0}; //Temperature gradient in troposphere, tropopause(0),stratosphere1,stratosphere2,stratopause(0),mesosphere1, mesosphere2 and mesopause(0)
    //long z_atmo[BS_const::atmo_model_slices] = {11000, 20000, 32000, 47000, 51000, 71000, 84852}; //altitude of tropopause start, tropopause(0),stratosphere1,stratosphere2,stratopause(0),mesosphere1, mesosphere2 and mesopause(0)
  
    bool BARO_failure; //failure indicators

  public:  

    BS_obj(SensingSystem* SensorSys);

    unsigned long initBaroAlt();

    double getAltitude();
};