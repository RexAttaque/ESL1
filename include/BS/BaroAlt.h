#pragma once

#include <Arduino.h>
#include <Sensors/SensingSystem.h>

const uint8_t atmo_model_slices = 6;

class BS_obj
{
  private:

    long Altitude;
    SensingSystem* BS_components;

    uint16_t refresh_BARO;
    unsigned long time_BARO;
    double delta_t;

    float alt_init; //Initial altitude
    float P_init; //Pressure at initial altitude
    float T_init; //Temperature at initial altitude
    double Tz[atmo_model_slices+1] = {-0.0065, 0, 0.00133, 0, -0.00291, 0, 0.00108}; //Temperature gradient in troposphere, tropopause(0),stratosphere,stratopause(0),mesosphere,mesopause(0) and thermosphere
    long z_atmo[atmo_model_slices] = {11000, 20000, 50000, 52000, 85000, 87000}; //altitude of tropopause start,stratosphere start,statopause start,mesosphere start,mesopause start and thermosphere start 
  
    bool BARO_failure; //failure indicators

  public:  

    BS_obj(SensingSystem* SensorSys);

    unsigned long initBaroAlt();

    long getAltitude();
};