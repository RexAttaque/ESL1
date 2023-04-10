#include <Sensors/IMU_Avio/ModuleLibs/IMU_Avio.h>

IMU_Avio::IMU_Avio(uint8_t type, uint8_t address):Base_Sensor(IMU_varAmount)
{
  IMU = Adafruit_BNO055(type, address);
}

bool IMU_Avio::init(adafruit_bno055_opmode_t mode, Adafruit_BNO055::adafruit_bno055_axis_remap_config_t remapcode, Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t remapsign)
{
  bool status = IMU.begin(mode);
  if(status)
  {
    IMU.setAxisRemap(remapcode);
    IMU.setAxisSign(remapsign);
  }
  return status;
}

uint8_t IMU_Avio::getCalibration()
{
  return IMU.getRawCalibration();
}

double* IMU_Avio::getMeas()
{
  uint8_t offset = 3;

  reallocateMemory();

  double* lin_accel = IMU.getArray(Adafruit_BNO055::VECTOR_LINEARACCEL);
  double* euler = IMU.getArray(Adafruit_BNO055::VECTOR_EULER);
  double* gyro = IMU.getArray(Adafruit_BNO055::VECTOR_GYROSCOPE);
  double* mag = IMU.getArray(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  for(uint8_t i=0; i<offset; i++)
  {
    meas[i] = lin_accel[i];
    meas[i+offset] = euler[i];
    meas[i+offset*2] = gyro[i];
    meas[i+offset*3] = mag[i];
  }

  delete[] lin_accel;
  delete[] euler;
  delete[] gyro;
  delete[] mag;

  return meas;
}