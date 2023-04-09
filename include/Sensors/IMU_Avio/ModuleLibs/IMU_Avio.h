#pragma once

#include <Wire.h>
#include <Adafruit_Unified_Sensor/Adafruit_Sensor.h>
#include <Sensors/IMU_Avio/InterfaceLib/Adafruit_BNO055.h>
#include <Sensors/IMU_Avio/InterfaceLib/utility/imumaths.h>

IMU_Avio IMU1_Avio = IMU_Avio(55, 0x28);
IMU_Avio IMU2_Avio = IMU_Avio(55, 0x29);
const uint8_t qty_IMU_Avio = 2;
IMU_Avio* IMU_Avio_array;

const uint8_t IMU_varAmount = 12; //a_x,a_y,a_z,theta,phi,psy,theta_point,phi_point,psy_point,m_x,m_y,m_z (m/s^2 ; rad ; rad/s ; microT)

class IMU_Avio {
  private :
    Adafruit_BNO055 IMU;
    uint8_t varAmount;
  
  public :
    IMU_Avio(uint8_t type, uint8_t address);

    uint8_t get_var_amount();

    float* getMeas();
};