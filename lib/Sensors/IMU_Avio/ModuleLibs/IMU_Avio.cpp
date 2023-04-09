#include <Sensors/IMU_Avio/ModuleLibs/IMU_Avio.h>

IMU_Avio::IMU_Avio(uint8_t type, uint8_t address)
{
  IMU = Adafruit_BNO055(type, address);
  varAmount = IMU_varAmount;  
};

uint8_t IMU_Avio::get_var_amount()
{
  return varAmount;
}

float* IMU_Avio::getMeas()
{
  
}