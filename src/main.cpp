#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/SensingSystem.h>
#include <EGI/KalmanPos.h>
#include <BS/BaroAlt.h>
#include <GSM/Sim800L.h>
#include <TELEM/ModuleLibs/SX1267MB1MAS.h>

using namespace std;

namespace main_const
{
  const String debug_ID = "MAIN";
  const uint8_t debug_lvl = debugLevel::FULL;

  const unsigned long deployTime = 100*pow(10,6); //time value in microseconds that when passed and when useTime=true, will trigger a parachute deployement
  const long deployAltitude = 3000; //altitude in m at which
};

IMU_BNO055 IMU1_Avio = IMU_BNO055(BNO055_ID, BNO055_ADDRESS_A, &Wire);
IMU_BNO055 IMU2_Avio = IMU_BNO055(BNO055_ID, BNO055_ADDRESS_B, &Wire1);
const uint8_t qty_IMU_Avio = 2;
IMU_BNO055 IMU_Avio_array[qty_IMU_Avio] = {IMU1_Avio, IMU2_Avio};

BARO_BMP280 BARO1_Avio = BARO_BMP280(BMP280_ADDRESS, &Wire);
BARO_BMP280 BARO2_Avio = BARO_BMP280(BMP280_ADDRESS_ALT, &Wire1);
const uint8_t qty_BARO_Avio = 2;
BARO_BMP280 BARO_Avio_array[qty_BARO_Avio] = {BARO1_Avio, BARO2_Avio};

GPS_UBX9 GPS1_Avio = GPS_UBX9(Serial3);
const uint8_t qty_GPS_Avio = 1;
GPS_UBX9 GPS_Avio_array[qty_GPS_Avio] = {GPS1_Avio};


fault_debug debug_main = fault_debug(main_const::debug_ID, main_const::debug_lvl); //Debug object for the MAIN module


SensingSystem SensorsSystem = SensingSystem(IMU_Avio_array, qty_IMU_Avio, GPS_Avio_array, qty_GPS_Avio, BARO_Avio_array, qty_BARO_Avio); //Sensing system which ensures combination and pre-processing of all sensor data, see header file for sensor declaration, type, combination technique etc.
EGI_obj EGI = EGI_obj(&SensorsSystem); //EGI - Embedded GPS/IMU, kalman filter algorithm for data fusion between IMU and GPS etc.
BS_obj BS = BS_obj(&SensorsSystem); //BS - Barometric System, altitude calculation from barometric (P and T) data. Used as a backup only to the EGI provided altitude
GSM_obj GSM = GSM_obj(Serial1);
TELEM_obj TELEM = TELEM_obj();

bool parachutesDeployed = false;
bool useTime; //indicates if time should be used for parachute deployement

uint8_t counter = 0;
unsigned long start_clk; //time at timer start
unsigned long end_clk; //time at timer stop
unsigned long loopTime; //timer in microseconds (used to measure loop() time)
unsigned long loopTimeMax; //time in microseconds that the loop() function must not exceed
unsigned long BaroLoopTimeMax; //time in microseconds that the loop() function must not exceed if barometric altitude is being used
unsigned long timeSinceLastBaroAlt; //time in microseconds since the last barometric altitude reading
unsigned long timeLeft; //time in microseconds that there is left until the next loop can start
bool timeStepChange = false; //indicates if the time quota was breached

void setup() {

  //Sensor Check/Init
  bool Sensor_init = SensorsSystem.initAll();
  //All sensors that can be put to sleep should be to sleep
  
  //GSM Check/Init
  bool GSM_init = GSM.init();
  //GSM module sent to sleep

  //Telemetry Check/Init
  bool TELEM_init = TELEM.init();
  //Telemetry sent to sleep

  //physical hardware Check/Init (parachutes etc.)
  
  //Before calibration and remaining inits, wake sensors
  SensorsSystem.wakeAll();

  //Calibrate Sensors
  bool Sensor_calibration = SensorsSystem.calibrateAll();

  loopTimeMax = EGI.initKalman(); //Initialize EGI (get initial measurements, variance, covariances etc.), sends back the max time between two kalman runs
  BaroLoopTimeMax = BS.initBaroAlt(); //Initialize Barometric System (recover initial altitude, pressure and temperature to initialize atmo model)

  //put sensors back to sleep
  SensorsSystem.sleepAll();

  if(Sensor_calibration && Sensor_init && GSM_init && loopTimeMax != 0 && BaroLoopTimeMax !=0 && TELEM_init) //Check Checks, Init and Check calibration
  {
    //Wait for wake call...
    debug_main.d_println(debugLevel::INFO, "Waiting for Main wake call...", "Setup");
    //SensorsSystem.wakeAll();

    //May need to run calibrations just before going on the pylon ?
    //Sensor_calibration = SensorsSystem.calibrateAll();

    //Put sensors back to sleep ?
    //SensorsSystem.sleepAll();

    //Wake sensors just before launch ?
    //SensorsSystem.wakeAll();

    //Disable USB Serial
    debug_main.d_end();

    //waiting for the launch trigger... (acceleration interrupt from the IMU maybe)
  }
  else
  {
    debug_main.d_println(debugLevel::INFO,"!!!!!! CHECK DEBUG & LOG, EXITING !!!!!!", "Setup");
    delay(10000);
    exit(0);
  }
}

void loop() {
  //main ESL loop with timing integration
  useTime = false;

  //start the clock to monitor for time step changes
  start_clk = micros();

  NavSolution Nav_Data = EGI.getNavSolution();
  double baro_altitude = BS.getAltitude(); //need to make sure we're not polling altitude faster than it can be acquired (BaroLoopTimeMax)

  double final_altitude = Nav_Data.altitude;
  if(final_altitude == faultCodes::altitude) //faultCodes::altitude is the fault indicating value for the altitude
  {
    final_altitude = baro_altitude;
  }
  if(final_altitude == faultCodes::altitude)
  {
    //check sensor failure flags to know if time should be used instead of altitude on top of this basic if else condition
    useTime = true;
  }

  //relay information via telemetry

  //relay information via GSM near the end of flight
  if(parachutesDeployed && GSM.check_REG_GSM() && GSM.check_SIG_GSM(GSM_const::signalQuality_floor))
  {
    bool NextStage = false;

    switch(counter)
    {
      case 0:
      {
        NextStage = GSM.PrepSend_s1();
        counter++;
      }
      case 1:
      {
        NextStage = GSM.PrepSend_s2();
        counter++;
      }
      case 2:
      {
        NextStage = GSM.PrepSend_s3();
        counter++;
      }
      case 3:
      {
        String GSM_message = "ESL1\nt:" + String(Nav_Data._time) + "\nx:" + Nav_Data.x + "\ny:" + Nav_Data.y + "\nz:" + Nav_Data.z + "\nh:" + final_altitude;
        GSM.setTX(GSM_message);
        NextStage = GSM.sendSMS();
        counter = 0;
      }

      if(!NextStage)
      {
        counter = 0;
      }
    }
  }

  //if altimeter or time criteria is reached
  if((final_altitude>main_const::deployAltitude && useTime == false) || (Nav_Data._time>main_const::deployTime && useTime == true))
  {
    GSM.goLive(); //turn on GSM module ?
    parachutesDeployed = true;
    //deploy parachutes
  }

  //check clock and delay accordingly
  end_clk = micros();

  loopTime = end_clk - start_clk;
  timeLeft = loopTimeMax - loopTime;
  if(timeLeft>0)
  { 
    if(timeStepChange == true)
    { 
      //revert the F matrix back to standard if time step was respected this time
      EGI.updateF(false,0);
      timeStepChange = false;
    }
    delayMicroseconds(timeLeft);
  }
  else if(timeLeft!=0)
  {
    //update the Kalman F matrix if time step has changed
    EGI.updateF(true,(double) loopTime/pow(10,6));
    timeStepChange = true;
  }
}