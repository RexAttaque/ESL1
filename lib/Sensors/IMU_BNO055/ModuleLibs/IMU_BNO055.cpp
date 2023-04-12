#include <Sensors/IMU_BNO055/ModuleLibs/IMU_BNO055.h>

IMU_BNO055::IMU_BNO055(uint8_t type, uint8_t address, adafruit_bno055_opmode_t mode, Adafruit_BNO055::adafruit_bno055_axis_remap_config_t remapcode, Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t remapsign)
:Base_Sensor(BNO055_const::varAmount, BNO055_const::Hz), IMU(Adafruit_BNO055(type,address)),_mode(mode),_remapcode(remapcode),_remapsign(remapsign)
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

bool IMU_BNO055::subCalibrate(uint8_t expectedResult)
{
  uint8_t subAttempts = 0;
  uint8_t calData = getCalibration();

  //sub Sensor calibration
  while(subAttempts<BNO055_const::maxCalibrationAttempts && (calData & expectedResult) == expectedResult)
  {
    if(debug::info()) Serial.println("\n        ---->subSensor is not calibrated yet (result is " + String(calData) + "), waiting " + String(BNO055_const::timeBetweenCalibrations/1000) + "s...");
    calData = getCalibration();
    subAttempts++;
    delay(BNO055_const::timeBetweenCalibrations);
  }

  if(subAttempts<BNO055_const::maxCalibrationAttempts)
  {
    if(debug::info()) Serial.println("      --->subSensor calibrated, letting it settle...");
    delay(BNO055_const::calibrationSettleTime);
    return true;
  }
  else
  {
    if(debug::info()) Serial.println("      --->Attempts at calibrating subSensor failed");
    return false;
  }
}

bool IMU_BNO055::calibrate()
{
  uint8_t attempts = 0;

  //System calibration 0xC0
  while(attempts<BNO055_const::maxCalibrationAttempts && (getCalibration() & 0xC0) == 0xC0)
  {
    //Gyroscope calibration 0x30
    if(debug::info()) 
    {
      Serial.println("\n      --->BNO055 Gyroscope subSensor calibration");
      Serial.println("      --->Instructions :");
      Serial.println("        ---->Stand the avionics bay UP RIGHT");
      Serial.println("        ---->Wait for calibration...");
    }
    delay(10000); //reading delay
    subCalibrate(0x30);

    //Accelerometer calibration 0x0C
    if(debug::info()) 
    {
      Serial.println("\n      --->BNO055 Accelerometer subSensor calibration");
      Serial.println("      --->Instructions :");
      Serial.println("        ---->Picture the axis system of the BN055 in the UP RIGHT (+z) Avionics bay position (x and y interchangeable).");
      Serial.println("        ---->SLOWLY from that position, rotate the avionics bay to align it with :");
      Serial.println("          ----->The x axis, put down and hold 5s");
      Serial.println("          ----->The -z axis, put down and hold 5s");
      Serial.println("          ----->The -y axis, put down and hold 5s");
      Serial.println("          ----->The -x axis, put down and hold 5s");
      Serial.println("          ----->The y axis, put down and hold 5s");
      Serial.println("          ----->Then back to the UP RIGHT (+z) position, put down and wait 10s");
      Serial.println("        ---->Repeat these steps until calibrated...");
    }
    delay(30000); //reading delay
    subCalibrate(0x0C);

    //Magnetometer calibration 0x03
    if(debug::info()) 
    {
      Serial.println("\n      --->BNO055 Magnetometer subSensor calibration...");
      Serial.println("      --->Instructions :");
      Serial.println("        ---->Pick up the avionics bay with both hands :");
      Serial.println("          ----->Draw two \"eight\" figures at a time then place the Avionics bay UP RIGHT for 5 seconds");
      Serial.println("        ---->Repeat these steps until calibrated...");
    }
    delay(20000); //reading delay
    subCalibrate(0x03);

    attempts++;
  }

  if(attempts<BNO055_const::maxCalibrationAttempts)
  {
    if(debug::info()) Serial.println("      --->BNO055 System calibrated, waiting for it to settle...");
    delay(BNO055_const::calibrationSettleTime);
    return true;
  }
  else
  {
    if(debug::info()) Serial.println("      --->Attempts at calibrating BNO055 failed");
    return false;
  }
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