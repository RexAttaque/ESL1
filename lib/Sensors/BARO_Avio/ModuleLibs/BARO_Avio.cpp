#include <Sensors/BARO_Avio/ModuleLibs/BARO_Avio.h>

class BARO_Avio {
  private :
    Adafruit_BMP280 BARO;
    uint8_t varAmount;
  
  public :
    BARO_Avio():varAmount(BARO_Avio_varAmount) {}
    
    uint8_t get_var_amount()
    {
      return varAmount;
    }

    float* getMeas()
    {
        
    }
};