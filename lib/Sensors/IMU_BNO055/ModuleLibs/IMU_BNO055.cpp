#include <Sensors/IMU_BNO055/ModuleLibs/IMU_BNO055.h>

IMU_BNO055::IMU_BNO055(uint8_t type, uint8_t address, adafruit_bno055_opmode_t mode, Adafruit_BNO055::adafruit_bno055_axis_remap_config_t remapcode, Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t remapsign)
:Base_Sensor(IMU_varAmount, IMU_BNO055_Hz), IMU(Adafruit_BNO055(type,address)),_mode(mode),_remapcode(remapcode),_remapsign(remapsign)
{}

bool IMU_BNO055::init()
{
  bool status = IMU.begin(_mode);

  if(status)
  {
    if(debug::info()) debug::Serial.println("      --->Standby for BIT...");
    delay(10); //wait a little just after startup for BIT to perform itself

    uint8_t sys_status = 0;
    uint8_t self_test = 0;
    uint8_t error = 0;

    IMU.getSystemStatus(&sys_status, &self_test, &error);
    if(debug::info()) 
    {
      debug::Serial.print("        ---->System Status : ");
      debug::Serial.println(sys_status);
      debug::Serial.print("        ---->Self Test Results : ");
      debug::Serial.println(self_test);
      debug::Serial.print("        ---->Error codes : ");
      debug::Serial.println(self_test);
    }

    if(sys_status == 5 && self_test == 15 && error == 0)
    {
      if(debug::info()) debug::Serial.println("      --->BNO055 Axis Remap...");
      IMU.setAxisRemap(_remapcode);
      IMU.setAxisSign(_remapsign);

      if(debug::info()) debug::Serial.println("      --->BNO055 Suspended until calibration");
      goIdle();

      return true;
    }
  }
  return false;
}

bool IMU_BNO055::goLive()
{
  IMU.enterNormalMode();
  return true; //Find a way to check if it did enter normal mode
}

bool IMU_BNO055::goIdle()
{
  IMU.enterSuspendMode();
  return true; //Find a way to check if it did enter suspend mode
}

uint8_t IMU_BNO055::getCalibration()
{
  return IMU.getRawCalibration();
}

double* IMU_BNO055::getMeas()
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