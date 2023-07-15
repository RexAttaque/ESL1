#include <EGI/KalmanPos.h>

//VarUpd, bool : specifies wether or not the covariance matrices will be updated during flight or not (experimental)
EGI_obj::EGI_obj(SensingSystem* SensorSys, bool VarUpd) : EGI_components(SensorSys),allowVarUpd(VarUpd)
{
  refresh_IMU = EGI_components->getIMUs_CG_Hz();
  refresh_GPS = EGI_components->getGPSs_Hz();

  time_IMU = pow(10,6)/refresh_IMU; //microseconds between each IMU refresh
  pred_Steps = refresh_IMU/refresh_GPS; //Number of time steps (loops) between each estimation of the kalman filter, otherwise simply predict (dead reckoning)
  
  std_delta_t = 1/refresh_IMU; //seconds, time step used by the prediction step of the kalman filter by default
  StepCount = 0;
  counter = 0;

  KalmanOutput._time = 0;

  //Initialize the F matrix with the standard (expected) time step
  updateF(true,std_delta_t);
  std_F = F;
  std_F_t = F_t;
}

//function to initialize the kalman filter (covariance matrices, refresh rate values, delta_t etc.). Must be run after all other sensors are initialized and ready since this uses them.
//returns the time that the loop() function running the kalman filter must not exceed if initialized succesfully, otherwise 0
unsigned long EGI_obj::initKalman()
{
  //Take initial measurements from the GPS and IMU to establish R0 (initial variances)
  
  //Placeholder
  R = {pow(5,2),0,0,0,0,0,
      0,pow(5,2),0,0,0,0,
      0,0,pow(5,2),0,0,0,
      0,0,0,pow(3*pow(10,-2),2),0,0,
      0,0,0,0,pow(3*pow(10,-2),2),0,
      0,0,0,0,0,pow(3*pow(10,-2),2)};
        
  
  //Establish Q0
  
  //Placeholder
  Q = {1,0,0,pow(10,-3),0,0,pow(10,-6),0,0,
      0,1,0,0,pow(10,-3),0,0,pow(10,-6),0,
      0,0,1,0,0,pow(10,-3),0,0,pow(10,-6),
      0,0,0,1,0,0,pow(10,-3),0,0,
      0,0,0,0,1,0,0,pow(10,-3),0,
      0,0,0,0,0,1,0,0,pow(10,-3),
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1};
  Q *= pow(10,-3);


  //Based on the confidence of the initial state, initialize P. 
  //Here we are certain for position, speed and acceleration 
  //(rocket is stationnary (certain about speed and acceleration) and initial position is acquired by the GPS (some uncertainty))
  P = {pow(10,6),0,0,0,0,0,0,0,0,
       0,pow(10,6),0,0,0,0,0,0,0,
       0,0,pow(10,6),0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,
       0,0,0,0,1,0,0,0,0,
       0,0,0,0,0,1,0,0,0,
       0,0,0,0,0,0,1,0,0,
       0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,0,0,1};
  P *= pow(10,-6);


  if(initialState())
  {
    return this->time_IMU;
  }
  else
  {
    return 0;
  }
}

bool EGI_obj::initialState()
{
  bool success = false;

  //From measurements (particularily GPS), initialize state x (particularily position, the rest should be 0)
  //  x = {x0,y0,z0,vx0,vy0,vz0,ax0,ay0,az0};

  double* posECEF_init = EGI_components->getGPSs_meas();

  if(posECEF_init != nullptr)  
  {
    x = {posECEF_init[1], posECEF_init[2], posECEF_init[3], 0, 0, 0, 0, 0, 0};
    success = true;
  }

  delete[] posECEF_init;

  return success;
}

//computes the F matrix (A matrix of the SS representation of the system) which depends on the time step in seconds (this is tied to the system's model)
//stepChange, bool : indicates wether this function should revert to the standard F matrix (standard time step was respected) or to recomputed F (with new time step)
//newTimeStep, seconds : time step used to compute the F matrix (may vary in flight)
void EGI_obj::updateF(bool stepChange, double newTimeStep)
{  
  if(stepChange)
  {
    delta_t = newTimeStep;
    double timeStep_2 = pow(newTimeStep,2)/2;
    
    F = {1,0,0,newTimeStep,0,0,timeStep_2,0,0,
          0,1,0,0,newTimeStep,0,0,timeStep_2,0,
          0,0,1,0,0,newTimeStep,0,0,timeStep_2,
          0,0,0,1,0,0,newTimeStep,0,0,
          0,0,0,0,1,0,0,newTimeStep,0,
          0,0,0,0,0,1,0,0,newTimeStep,
          0,0,0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,0,0};
    
    F_t = ~F;
  }
  else
  {
    delta_t = std_delta_t;
    F = std_F;
    F_t = std_F_t;
  }
}

//getNavSolution provides time since launch, IMU acceleration, IMU euler angles, GPS position, Kalman filter position, BARO altitude, altitude (source may change depending on failures), IMU status, GPS status, BARO status
//
//IMU, array of IMU objects
//IMU_amount, number of IMUs in the array IMU
//GPS, array of GPS objects
//GPS_amount, number of GPSs in the array GPS
//BARO, array of barometer objects
//BARO_amount, number of BAROs in the array BARO
//
//Returns (index specified at the beginning) :
//_time - Time since launch (seconds)
//x - Kalman ECEF Position x (cm)
//y - Kalman ECEF Position y (cm)
//z - Kalman ECEF Position z (cm)
//altitude - WGS84 altitude or baro if GPS and IMU fail (cm)
NavSolution EGI_obj::getNavSolution()
{
  double* IMUpdata = EGI_components->getIMUs_CG_meas();
          
  //if polling one set of measurements failed, discard it, if all failed, switch to GPS only, if all GPS fail, switch to barometric altitude, if that fails, switch to time based    
  if(IMUpdata != nullptr)
  {
    IMU_failure = false;

    //IMU measurement base change (based on orientation, from rocket to ENUmag)

    //IMU measurement base change (based on the previous position, base change from ENUmag to ENUtrue then ECEF) and transfer to measurement vector
    for(int i=0; i<EGI_const::pos_var; i++)
    {
      ENU(i,EGI_const::pos_var) = x(i,1)/EGI_const::WGS84(i,1);
    }
  
    double phi = acos(ENU(3,EGI_const::pos_var));
    double theta = atan(ENU(2,EGI_const::pos_var)/ENU(1,EGI_const::pos_var));
  
    ENU(1,1) = -sin(theta);
    ENU(2,1) = cos(theta);
    ENU(3,1) = 0;
  
    ENU(1,2) = -cos(phi)*cos(theta);
    ENU(2,2) = -cos(phi)*sin(theta);
    ENU(3,2) = sin(phi);

    tempPosPos = ENU*ENUmag;
    
    //store acceleration data into the measurement vector y
    for(int i=EGI_const::pos_var; i<EGI_const::meas_var-EGI_const::pos_var;i++)
    {
      y(i,1)=0;
      
      for(int j=0; j<EGI_const::pos_var; j++)
      {
        y(i,1) += tempPosPos(i,j)*(IMUpdata[j]);
      }
    }

    //acceleration is also part of the input vector u
    u = y.Submatrix<EGI_const::input_var,1>(EGI_const::input_var-1,0);

    //IMU measurement drift correction here or above
  }
  else
  {
      IMU_failure = true;
  }
  
  //estimation step
  if(StepCount%pred_Steps == 0 && StepCount !=0)
  { 
    //GPS measurement recovery
    double* posECEF = EGI_components->getGPSs_meas();
    uint8_t offset_pos_GPS = 1;

    if(posECEF != nullptr) //maybe check also if you're not getting the same position twice or if the GPS comes back online
    {  
      //into the measurement vector for position
      for(uint8_t i=0; i<EGI_const::pos_var;i++)
      {
        y(i,1) = posECEF[i+offset_pos_GPS];
      }

      GPS_failure = false;        
      
      //Gain computation
      tempMeasMeas = H*P*H_t+R;
      K = P*H_t*BLA::Invert(tempMeasMeas);
      
      //Covariance computation
      P = P - K*H*P;
  
      //Update
      x_est = x + K*(y-H*x);

      if(allowVarUpd)
      {
        for(uint8_t i=0;i<EGI_const::pos_var;i++)
        {
          error_pos_meas(i,counter) = y(i,1) - x_est(i,1);
          error_pos_pred(i,counter) = x(i,1) - x_est(i,1);
        }
    
        counter += 1;
        if(counter == EGI_const::noVarUpd_Steps)
        {
          //Recompute R based on what was measured for position and what was estimated
          
          //Recompute Q based on what was predicted for position and what was estimated
    
          counter = 0;
        }
      }
      
      //discard previous prediction as estimation is more accurate anyways
      x = x_est;
    }
    else
    {
      GPS_failure = true;
    }
  }

  if(!IMU_failure)
  {
    //Prediction step
    x = F*x + G*u;
  
    //Covariance computation
    P = F*P*F_t + Q;
  }
  else if(!GPS_failure)
  {
    //simply put the GPS position into the x vector and 0 for the other variables
    for(int i=0;i<EGI_const::ss_var;i++)
    {
      if(i<EGI_const::pos_var)
      {
        x(i,1) = y(i,1);
      }
      else
      {
        x(i,1) = 0;
      }
    }
  }
  
  //Altitude computation in ECEF
  if(!IMU_failure || !GPS_failure)
  {
    //iso Lat,Long ground position in ECEF
    double lg = atan(x(2,1)/x(1,1));
    double la = atan(x(2,1)/sqrt(pow(x(1,1),2)+pow(x(2,1),2)));
  
    double r0 = sqrt(1/( pow(cos(la),2)*pow(cos(lg),2)/pow(EGI_const::WGS84(1,1),2) + pow(cos(la),2)*pow(sin(lg),2)/pow(EGI_const::WGS84(2,1),2) + pow(sin(la),2)/pow(EGI_const::WGS84(3,1),2) ));
  
    double pos_sol[3] = {r0*cos(la)*cos(lg), r0*cos(la)*sin(lg), r0*sin(la)};

    KalmanOutput.x = x(1,1);
    KalmanOutput.y = x(2,1);
    KalmanOutput.y = x(3,1);
    KalmanOutput.altitude = sqrt(pow((x(1,1)-pos_sol[0]),2)+pow((x(2,1)-pos_sol[1]),2)+pow((x(3,1)-pos_sol[2]),2));
  }
  else
  {
    KalmanOutput.x = faultCodes::pos;
    KalmanOutput.y = faultCodes::pos;
    KalmanOutput.z = faultCodes::pos;
    KalmanOutput.altitude = faultCodes::altitude;
  }

  //time computation (approximate)
  KalmanOutput._time += delta_t;

  StepCount+=1;

  return KalmanOutput;
}