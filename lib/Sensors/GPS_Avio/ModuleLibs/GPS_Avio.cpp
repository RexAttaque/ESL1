#include <Sensors/GPS_Avio/ModuleLibs/GPS_Avio.h>

class GPS_Avio {
  private :
    ublox_gen9 GPS;
    uint8_t varAmount;
  
  public :
  
    GPS_Avio(HardwareSerial HWSerial):GPS(ublox_gen9(HWSerial))
    {
      varAmount = GPS_varAmount; 
    };

    uint8_t get_var_amount()
    {
      return varAmount;
    }

    float* getMeas()
    {
      
    }
};