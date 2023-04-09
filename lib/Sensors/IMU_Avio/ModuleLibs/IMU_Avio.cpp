#include <Sensors/IMU_Avio/ModuleLibs/IMU_Avio.h>

class IMU_Avio {
  private :
    Adafruit_BNO055 IMU;
    uint8_t varAmount;
  
  public :
    IMU_Avio(uint8_t type, uint8_t adress)
    {
      IMU = Adafruit_BNO055(type, adress);
      varAmount = IMU_varAmount;  
    };

    uint8_t get_var_amount()
    {
      return varAmount;
    }

    float* getMeas()
    {
      
    }
};