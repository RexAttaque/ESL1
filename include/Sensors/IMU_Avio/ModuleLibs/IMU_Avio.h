#pragma once

#include <Wire.h>
#include <Sensors/Base_Sensor.h>
#include <Adafruit_Unified_Sensor/Adafruit_Sensor.h>
#include <Sensors/IMU_Avio/InterfaceLib/Adafruit_BNO055.h>
#include <Sensors/IMU_Avio/InterfaceLib/utility/imumaths.h>

IMU_Avio IMU1_Avio = IMU_Avio(BNO055_ID, BNO055_ADDRESS_A);
IMU_Avio IMU2_Avio = IMU_Avio(BNO055_ID, BNO055_ADDRESS_B);
const uint8_t qty_IMU_Avio = 2;
IMU_Avio* IMU_Avio_array;

const uint8_t IMU_varAmount = 12; //a_x,a_y,a_z,theta,phi,psy,theta_point,phi_point,psy_point,m_x,m_y,m_z (m/s^2 ; rad ; rad/s ; microT) <- ENU referential

class IMU_Avio : public Base_Sensor<double> {
  private :
    Adafruit_BNO055 IMU;
  
  public :
    IMU_Avio(uint8_t type, uint8_t address);

    bool init(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF, Adafruit_BNO055::adafruit_bno055_axis_remap_config_t remapcode = Adafruit_BNO055::REMAP_CONFIG_P1, Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t remapsign = Adafruit_BNO055::REMAP_SIGN_P1);

    uint8_t getCalibration();

    double* getMeas();
};