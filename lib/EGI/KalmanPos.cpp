#include <EGI/KalmanPos.h>

class EGI_obj {
  private:

    NavSolution KalmanOutput;
    SensingSystem EGI_components;
  
    uint8_t refresh_IMU; //Hz, IMU refresh rate
    unsigned long time_IMU; //microseconds between each IMU refresh
    unsigned long timeLeft; //microseconds, time left till next IMU refresh
    uint8_t refresh_GPS; //Hz, GPS refresh rate
    uint8_t pred_Steps; //Number of time steps (loops) between each estimation of the kalman filter, otherwise simply predict (dead reckoning)

    //Kalman filter Q and R matrices update
    bool allowVarUpd = true;

    double std_delta_t; //seconds, time step used by the prediction step of the kalman filter by default
    double delta_t; //seconds, current time step
    unsigned long StepCount; //current time step
    uint8_t counter; //second counter to start counting from a specific time step

    bool IMU_failure; //failure indicators
    bool GPS_failure;
    bool BARO_failure;

    //Altitude calculation necessities
    //x, y, z ECEF position of ESL at the same Lat,Long but at ground level
    BLA::Matrix<pos_var,1> pos_sol;

    //t_x, t_y, t_z -- east (mag)
    //t'_x, t'_y, t'_z -- north (mag)
    //n_x, n_y, n_z -- up/normal
    //static for now, can depend on the position, will see if it makes a big difference
    const BLA::Matrix<pos_var,pos_var> ENUmag = {cos(mag_dev), -sin(mag_dev), 0, 
                                                 sin(mag_dev), cos(mag_dev), 0, 
                                                 0, 0, 1};
                                         
    //t_x, t_y, t_z -- east (true)
    //t'_x, t'_y, t'_z -- north (true)
    //n_x, n_y, n_z -- up/normal
    BLA::Matrix<pos_var,pos_var> ENU;
    
    
    // x, y, z, v_x, v_y, v_z, a_x, a_y, a_z (state vector, estimated and predicted (dead-reckoning for the latter))
    BLA::Matrix<ss_var,1> x;
    BLA::Matrix<ss_var,1> x_est;
    //error on position between measurement and estimation, allows to recompute the measurement covariance every so often
    BLA::Matrix<pos_var,noVarUpd_Steps> error_pos_meas;
    
    //a_x, a_y, a_z (input vector)
    BLA::Matrix<input_var,1> u;
    
    // x, y, z, a_x, a_y, a_z (final processed measurements from GPS and IMU)
    BLA::Matrix<meas_var,1> y;
    //error on position between prediction and estimation, allows to recompute the state covariance every so often
    BLA::Matrix<pos_var,noVarUpd_Steps> error_pos_pred;


    // y_k = H*x_k + v_k -- y_k measurement vector at time step k, x_k state vector, v_k measurement noise -- this matrix does not change with time in our case
    const BLA::Matrix<meas_var,ss_var, BLA::Array<meas_var,ss_var,uint8_t>> H = {1,0,0,0,0,0,0,0,0,
                                                                                 0,1,0,0,0,0,0,0,0,
                                                                                 0,0,1,0,0,0,0,0,0,
                                                                                 0,0,0,0,0,0,1,0,0,
                                                                                 0,0,0,0,0,0,0,1,0,
                                                                                 0,0,0,0,0,0,0,0,1};
    
    const BLA::Matrix<ss_var,meas_var, BLA::Array<ss_var,meas_var,uint8_t>> H_t = ~H;
    
    // Covariance matrix of the measurement noise (diag if measurement variables are indep.) -- a 0 means no noise if on the diag (variance) and no coupling outside (covariance).
    BLA::Matrix<meas_var,meas_var> R;
    
    // Kalman filter gain for optimal estimation
    BLA::Matrix<ss_var,meas_var> K;
    
    
    // x_k = F_(k-1)*x_(k-1) + G_(k-1)*u_(k-1) + w_(k-1) -- F_(k-1) equivalent to the A matrix of a SS representation at time step k-1, G_(k-1) "" B matrix "", w_(k-1) is the state noise -- this matrix may change if delta_t is not constant between time steps
    BLA::Matrix<ss_var,ss_var> F_t;
    BLA::Matrix<ss_var,ss_var> F;
    
    BLA::Matrix<ss_var,ss_var> std_F_t;
    BLA::Matrix<ss_var,ss_var> std_F;

    // x_k = F_(k-1)*x_(k-1) + G_(k-1)*u_(k-1) + w_(k-1) -- F_(k-1) equivalent to the A matrix of a SS representation at time step k-1, G_(k-1) "" B matrix "", w_(k-1) is the state noise
    const BLA::Matrix<ss_var,input_var> G = {0,0,0,
                                             0,0,0,
                                             0,0,0,
                                             0,0,0,
                                             0,0,0,
                                             0,0,0,
                                             1,0,0,
                                             0,1,0,
                                             0,0,1};
    
    // Covariance matrix of the state noise (diag if SS variables are indep.) -- a 0 means no noise if on the diag (variance) and no coupling outside (covariance).
    BLA::Matrix<ss_var,ss_var> Q;
    
    
    // Covariance matrix of the estimation error (diag) -- initialized at 0 (or close to) if the initial state is perfectly known, +infinity if initial state is unknown
    BLA::Matrix<ss_var,ss_var> P;
    
    
    BLA::Matrix<meas_var,meas_var> tempMeasMeas;
    BLA::Matrix<pos_var,pos_var> tempPosPos;


  public :

    //IMU_refreshRate, Hz : refresh rate of the IMU that will be used
    //GPS_refreshRate, Hz : refresh rate of the GPS that will be used
    //VarUpd, bool : specifies wether or not the covariance matrices will be updated during flight or not (experimental)
    EGI_obj(uint8_t IMU_refreshRate, uint8_t GPS_refreshRate, bool VarUpd) : refresh_IMU(IMU_refreshRate), refresh_GPS(GPS_refreshRate), allowVarUpd(VarUpd)
    {
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
    unsigned long initKalman()
    {
      //Take initial measurements from the GPS and IMU to establish R0 (initial variances)
      //  R = {1,0,0,0,0,0,
      //       0,1,0,0,0,0,
      //       0,0,1,0,0,0,
      //       0,0,0,1,0,0,
      //       0,0,0,0,1,0,
      //       0,0,0,0,0,1};
           
      //Establish Q0
      //  Q = {1,0,0,0,0,0,0,0,0,
      //       0,1,0,0,0,0,0,0,0,
      //       0,0,1,0,0,0,0,0,0,
      //       0,0,0,1,0,0,0,0,0,
      //       0,0,0,0,1,0,0,0,0,
      //       0,0,0,0,0,1,0,0,0,
      //       0,0,0,0,0,0,1,0,0,
      //       0,0,0,0,0,0,0,1,0,
      //       0,0,0,0,0,0,0,0,1};
    
      //From these measurements (particularily GPS), initialize x (particularily position, the rest should be 0)
      //  x = {x0,y0,z0,vx0,vy0,vz0,ax0,ay0,az0};
    
      //Based on the confidence of the initial state, initialize P
      //  P = {1,0,0,0,0,0,0,0,0,
      //       0,1,0,0,0,0,0,0,0,
      //       0,0,1,0,0,0,0,0,0,
      //       0,0,0,1,0,0,0,0,0,
      //       0,0,0,0,1,0,0,0,0,
      //       0,0,0,0,0,1,0,0,0,
      //       0,0,0,0,0,0,1,0,0,
      //       0,0,0,0,0,0,0,1,0,
      //       0,0,0,0,0,0,0,0,1};

      //KalmanOutput init

      if(true)
      {
        return this->time_IMU;
      }
      else
      {
        return 0;
      }
    }
  
    //computes the F matrix (A matrix of the SS representation of the system) which depends on the time step in seconds (this is tied to the system's model)
    //stepChange, bool : indicates wether this function should revert to the standard F matrix (standard time step was respected) or to recomputed F (with new time step)
    //newTimeStep, seconds : time step used to compute the F matrix (may vary in flight)
    void updateF(bool stepChange, double newTimeStep)
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
    //z - Kalman ECEF Position x (cm)
    //altitude - WGS84 altitude or baro if GPS and IMU fail (cm)
    NavSolution getNavSolution()
    {
      float* IMUpdata = EGI_components.getIMUs_Avio_meas();
      uint8_t IMUs_rl_amount = EGI_components.getIMUs_Avio_rl_amount();
              
      //if polling one set of measurements failed, discard it, if all failed, switch to GPS only, if all GPS fail, switch to barometric altitude, if that fails, switch to time based    
      if(IMUs_rl_amount > 0)
      {
        IMU_failure = false;

        //IMU measurement base change (based on the previous position, base change from ENUmag to ENUtrue then ECEF) and transfer to measurement vector
        for(int i=0; i<pos_var; i++)
        {
          ENU(i,pos_var) = x(i,1)/WGS84(i,1);
        }
      
        double phi = acos(ENU(3,pos_var));
        double theta = atan(ENU(2,pos_var)/ENU(1,pos_var));
      
        ENU(1,1) = -sin(theta);
        ENU(2,1) = cos(theta);
        ENU(3,1) = 0;
      
        ENU(1,2) = -cos(phi)*cos(theta);
        ENU(2,2) = -cos(phi)*sin(theta);
        ENU(3,2) = sin(phi);
    
        tempPosPos = ENU*ENUmag;
        
        for(int i=pos_var; i<meas_var-pos_var;i++)
        {
          y(i,1)=0;
          
          for(int j=0; j<pos_var; j++)
          {
            y(i,1) += tempPosPos(i,j)*(IMUpdata[j]);
          }
        }
        
        //IMU measurement drift correction (before or after the voting step) 
      }
      else
      {
          IMU_failure = true;
      }
      
      //estimation step
      if(StepCount%pred_Steps == 0 && StepCount !=0)
      { 
        //GPS measurement recovery
        long* posECEF = EGI_components.getGPSs_Avio_meas();
        uint8_t GPSs_rl_amount = EGI_components.getGPSs_Avio_rl_amount();

          //into the measurement vector for position
    
        if(GPSs_rl_amount > 0) //maybe check also if you're not getting the same position twice or if the GPS comes back online
        {  
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
            for(uint8_t i=0;i<pos_var;i++)
            {
              error_pos_meas(i,counter) = y(i,1) - x_est(i,1);
              error_pos_pred(i,counter) = x(i,1) - x_est(i,1);
            }
        
            counter += 1;
            if(counter == noVarUpd_Steps)
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
        for(int i=0;i<ss_var;i++)
        {
          if(i<pos_var)
          {
            x(i,1) = y(i,1);
          }
          else
          {
            x(i,1) = 0;
          }
        }
      }
      
      //Barometric altitude computation
      if(IMU_failure && GPS_failure)
      {
        uint8_t BAROs_rl_amount = EGI_components.getBAROs_Avio_rl_amount();
        
        if(BAROs_rl_amount > 0)
        {
            BARO_failure = false;

        }
        else
        {
            BARO_failure = true;
        }
      }
      else //Altitude computation in ECEF
      {
        //iso Lat,Long ground position in ECEF
        double lg = atan(x(2,1)/x(1,1));
        double la = atan(x(2,1)/sqrt(pow(x(1,1),2)+pow(x(2,1),2)));
      
        double r0 = sqrt(1/( pow(cos(la),2)*pow(cos(lg),2)/pow(WGS84(1,1),2) + pow(cos(la),2)*pow(sin(lg),2)/pow(WGS84(2,1),2) + pow(sin(la),2)/pow(WGS84(3,1),2) ));
      
        pos_sol(1,1) = r0*cos(la)*cos(lg);
        pos_sol(2,1) = r0*cos(la)*sin(lg);
        pos_sol(3,1) = r0*sin(la);
      }
    
      //time computation (approximate)
      KalmanOutput._time += delta_t;

      StepCount+=1;
    }
};