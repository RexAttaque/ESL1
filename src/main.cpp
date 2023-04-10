#include <Arduino.h>
#include <faultCodes.h>
#include <EGI/KalmanPos.h>
#include <BS/BaroAlt.h>
#include <Sensors/SensingSystem.h>

SensingSystem SensorsSystem = SensingSystem(); //Sensing system which ensures combination and pre-processing of all sensor data, see header file for sensor declaration, type, combination technique etc.
EGI_obj EGI = EGI_obj(&SensorsSystem, true); //EGI - Embedded GPS/IMU, kalman filter algorithm for data fusion between IMU and GPS etc.
BS_obj BS = BS_obj(&SensorsSystem); //BS - Barometric System, altitude calculation from barometric (P and T) data. Used as a backup only to the EGI provided altitude

bool parachutesDeployed = false;
bool useTime; //indicates if time should be used for parachute deployement
unsigned long deployTime = 100*pow(10,6); //time value in microseconds that when passed and when useTime=true, will trigger a parachute deployement
long deployAltitude = 3000*100; //altitude in cm at which

unsigned long start_clk; //time at timer start
unsigned long end_clk; //time at timer stop
unsigned long loopTime; //timer in microseconds (used to measure loop() time)
unsigned long loopTimeMax; //time in microseconds that the loop() function must not exceed
unsigned long timeLeft; //time in microseconds that there is left until the next loop can start
bool timeStepChange = false; //indicates if the time quota was breached

void setup() {
  
  //init serial for debug
  //init sensors
  //init GPS, wait for nav solution 
  //calibrate IMUs (either before GPS to save time or after to stay as precise as possible until launch)
  loopTimeMax = EGI.initKalman(); //IMU refresh rate, GPS refresh rate

  if(loopTimeMax != 0)
  {
    //NOTE : may need to run calibrations just before launch
    //waiting for the launch trigger... (acceleration interrupt from the IMU maybe)
  }
  else
  {
    //Init fail
  }
}

void loop() {
  //main ESL loop with timing integration
  useTime = false;

  //start the clock to monitor for time step changes
  start_clk = micros();

  NavSolution Nav_Data = EGI.getNavSolution();

  long final_altitude = Nav_Data.altitude;
  if(final_altitude == altitudeFaultCode) //altitudeFaultCode is the fault indicating value for the altitude
  {
    final_altitude = BS.getAltitude();
  }
  if(final_altitude == altitudeFaultCode)
  {
    //check sensor failure lags to know if time should be used instead of altitude on top of this basic if else condition
    useTime = true;
  }

  //relay information via telemetry

  //relay informatio via GSM near the end of flight
  
  //if altimeter or time criteria is reached
  if((final_altitude>deployAltitude && useTime == false) || (Nav_Data._time>deployTime && useTime == true))
  {
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