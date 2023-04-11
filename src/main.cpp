#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/SensingSystem.h>
#include <EGI/KalmanPos.h>
#include <BS/BaroAlt.h>
#include <GSM/Sim800L.h>

SensingSystem SensorsSystem = SensingSystem(); //Sensing system which ensures combination and pre-processing of all sensor data, see header file for sensor declaration, type, combination technique etc.
EGI_obj EGI = EGI_obj(&SensorsSystem, true); //EGI - Embedded GPS/IMU, kalman filter algorithm for data fusion between IMU and GPS etc.
BS_obj BS = BS_obj(&SensorsSystem); //BS - Barometric System, altitude calculation from barometric (P and T) data. Used as a backup only to the EGI provided altitude
GSM_obj GSM_module = GSM_obj();

bool parachutesDeployed = false;
bool useTime; //indicates if time should be used for parachute deployement
unsigned long deployTime = 100*pow(10,6); //time value in microseconds that when passed and when useTime=true, will trigger a parachute deployement
long deployAltitude = 3000*100; //altitude in cm at which

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
  
  if(debug::info()) debug::Serial.begin(115200); //initialize serial monitor for debugging/information

  //Sensor Check/Init
  bool Sensor_init = SensorsSystem.initAll();
  //All sensors that can be put to sleep should be to sleep
  
  //GSM Check/Init
  bool GSM_init = GSM_module.init();
  //GSM module sent to sleep

  //Telemetry Check/Init


  //physical hardware check
  //physical hardware init
  
  bool Sensor_wake = SensorsSystem.wakeAll();
  //calibrate IMUs (either before GPS to save time or after to stay as precise as possible until launch)

  loopTimeMax = EGI.initKalman(); //Initialize EGI (get initial measurements, variance, covariances etc.)
  BaroLoopTimeMax = BS.initBaroAlt(); //Initialize Barometric System (recover initial altitude, pressure and temperature to initialize atmo model)

  if(Sensor_init && GSM_init && loopTimeMax != 0 && BaroLoopTimeMax !=0) //Check Checks, Init and Check calibration
  {
    bool Sensor_sleep = SensorsSystem.sleepAll();
    
    //Wait for wake call...

    //NOTE : may need to run calibrations just before launch

    //waiting for the launch trigger... (acceleration interrupt from the IMU maybe)
  }
  else
  {
    if(debug::info()) debug::Serial.println("MAIN INIT FAIL, CHECK DEBUG, EXITING !");
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
  long baro_altitude = BS.getAltitude(); //need to make sure we're not polling altitude faster than it can be acquired (BaroLoopTimeMax)

  long final_altitude = Nav_Data.altitude;
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

  //relay informatio via GSM near the end of flight
  if(parachutesDeployed && GSM_module.check_REG_GSM() && GSM_module.check_SIG_GSM(GSM::signalQuality_floor))
  {
    bool NextStage = false;

    switch(counter%GSM::sendStages)
    {
      case 0:
      {
        NextStage = GSM_module.PrepSend_s1();
        counter++;
      }
      case 1:
      {
        NextStage = GSM_module.PrepSend_s2();
        counter++;
      }
      case 2:
      {
        NextStage = GSM_module.PrepSend_s3();
        counter++;
      }
      case 3:
      {
        String GSM_message = ""; //TODO, fill message
        GSM_module.setTX(GSM_message);
        NextStage = GSM_module.sendSMS();
        counter = 0;
      }

      if(!NextStage)
      {
        counter = 0;
      }
    }
  }

  //if altimeter or time criteria is reached
  if((final_altitude>deployAltitude && useTime == false) || (Nav_Data._time>deployTime && useTime == true))
  {
    GSM_module.goLive(); //turn on GSM module ?
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