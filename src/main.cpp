#include <Arduino.h>

//Main

#include <EGI/KalmanPos.h>
#include <Sensors/SensingSystem.h>

EGI_obj EGI_Avio = EGI_obj(100, 25, true); //EGI - Embedded GPS/IMU, kalman filter algorithm for data fusion between IMU and GPS etc.

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
  loopTimeMax = EGI_Avio.initKalman(); //IMU refresh rate, GPS refresh rate

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
  
  //start the clock to monitor for time step changes
  start_clk = micros();

  NavSolution Nav_Data = EGI_Avio.getNavSolution();

  //if altimeter or time criteria is reached, deploy parachutes

  //relay information
  
  //check clock and delay accordingly
  end_clk = micros();

  loopTime = end_clk - start_clk;
  timeLeft = loopTimeMax - loopTime;
  if(timeLeft>0)
  { 
    if(timeStepChange == true)
    { 
      EGI_Avio.updateF(false,0);
      timeStepChange = false;
    }
    delayMicroseconds(timeLeft);
  }
  else if(timeLeft!=0)
  {
    EGI_Avio.updateF(true,(double) loopTime/pow(10,6));
    timeStepChange = true;
  }
}