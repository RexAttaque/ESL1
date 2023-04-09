#pragma once

#include <Arduino.h>
#include <Sensors/BARO_Avio/InterfaceLibs/Adafruit_BMP280.h>

const uint8_t BARO_Avio_varAmount = 2; //P,T (Pa ; °C)

class BARO_Avio {
  private :
    Adafruit_BMP280 BARO;
    uint8_t varAmount;
  
  public :
    BARO_Avio();
    
    uint8_t get_var_amount();

    float* getMeas();
};