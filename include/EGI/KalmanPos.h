#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/SensingSystem.h>
//Linear Algebra Library, has been modified so that the default Matrix storage type is double
#include <LinearAlgebra/BasicLinearAlgebra.h>
#include <LinearAlgebra/ElementStorage.h>

namespace EGI_const 
{
  const String debug_ID = "EGI";
  const uint8_t debug_lvl = debugLevel::FULL;

  //matrix sizes
  const uint8_t pos_var = 3; //number of position variables
  const uint8_t imu_var = 6; //number of measurements recovered from the IMUs
  const uint8_t ss_var = 9; //number of state space variables
  const uint8_t meas_var = 6; //number of measured variables
  const uint8_t input_var = 3; //number of input variables

  //Kalman filter Q and R matrices update
  const uint8_t noVarUpd_Steps = 10; //Matrices Q and R are update every "varianceUpd" time steps after the estimation step -- experimental
  const bool allowVarUpd = true;

  //Base change necessities
  //a, b, c m, earth's major, median and minor axis WGS84
  const BLA::Matrix<pos_var,1> WGS84 = {6378137,6378137,6356752};

  //magnetic deviation (+ is east, - is west)
  const float mag_dev = -1.24*PI/180; //for some random place in Portugal, will need to be changed //REVIEW
  //const float mag_incl = 52.5167*PI/180; //is probably compensated by the BNO055 already, otherwise needs to be included in the ENUmag matrix
};
struct NavSolution
{
  unsigned long _time;
  double x;
  double y;
  double z;
  double altitude;
};

class EGI_obj : public fault_debug {
  private:

    NavSolution KalmanOutput;
    SensingSystem* EGI_components;
  
    uint8_t refresh_IMU; //Hz, IMU refresh rate
    unsigned long time_IMU; //microseconds between each IMU refresh
    unsigned long timeLeft; //microseconds, time left till next IMU refresh
    uint8_t refresh_GPS; //Hz, GPS refresh rate
    uint8_t pred_Steps; //Number of time steps (loops) between each estimation of the kalman filter, otherwise simply predict (dead reckoning)

    double std_delta_t; //seconds, time step used by the prediction step of the kalman filter by default
    double delta_t; //seconds, current time step
    unsigned long StepCount; //current time step
    uint8_t counter; //second counter to start counting from a specific time step

    bool IMU_failure; //failure indicators
    bool GPS_failure;

    BLA::Matrix<EGI_const::pos_var,EGI_const::pos_var> RKT;

    //t_x, t_y, t_z -- east (mag)
    //t'_x, t'_y, t'_z -- north (mag)
    //n_x, n_y, n_z -- up/normal
    //static for now, can depend on the position, will see if it makes a big difference
    const BLA::Matrix<EGI_const::pos_var,EGI_const::pos_var> ENUmag = {cos(EGI_const::mag_dev), -sin(EGI_const::mag_dev), 0, 
                                                                       sin(EGI_const::mag_dev), cos(EGI_const::mag_dev), 0, 
                                                                       0, 0, 1};
                                         
    //t_x, t_y, t_z -- east (true)
    //t'_x, t'_y, t'_z -- north (true)
    //n_x, n_y, n_z -- up/normal
    BLA::Matrix<EGI_const::pos_var,EGI_const::pos_var> ENU;
    
    
    // x, y, z, v_x, v_y, v_z, a_x, a_y, a_z (state vector, estimated and predicted (dead-reckoning for the latter))
    BLA::Matrix<EGI_const::ss_var,1> x;
    BLA::Matrix<EGI_const::ss_var,1> x_est;
    //error on position between measurement and estimation, allows to recompute the measurement covariance every so often
    BLA::Matrix<EGI_const::pos_var,EGI_const::noVarUpd_Steps> error_pos_meas;
    
    //a_x, a_y, a_z (input vector)
    BLA::Matrix<EGI_const::input_var,1> u;
    
    // x, y, z, a_x, a_y, a_z (final processed measurements from GPS and IMU)
    BLA::Matrix<EGI_const::meas_var,1> y;
    //error on position between prediction and estimation, allows to recompute the state covariance every so often
    BLA::Matrix<EGI_const::pos_var,EGI_const::noVarUpd_Steps> error_pos_pred;


    // y_k = H*x_k + v_k -- y_k measurement vector at time step k, x_k state vector, v_k measurement noise -- this matrix does not change with time in our case
    const BLA::Matrix<EGI_const::meas_var,EGI_const::ss_var, BLA::Array<EGI_const::meas_var,EGI_const::ss_var,uint8_t>> H = {1,0,0,0,0,0,0,0,0,
                                                                                                                             0,1,0,0,0,0,0,0,0,
                                                                                                                             0,0,1,0,0,0,0,0,0,
                                                                                                                             0,0,0,0,0,0,1,0,0,
                                                                                                                             0,0,0,0,0,0,0,1,0,
                                                                                                                             0,0,0,0,0,0,0,0,1};
    
    const BLA::Matrix<EGI_const::ss_var,EGI_const::meas_var, BLA::Array<EGI_const::ss_var,EGI_const::meas_var,uint8_t>> H_t = ~H;
    
    // Covariance matrix of the measurement noise (diag if measurement variables are indep.) -- a 0 means no noise if on the diag (variance) and no coupling outside (covariance).
    BLA::Matrix<EGI_const::meas_var,EGI_const::meas_var> R;
    
    // Kalman filter gain for optimal estimation
    BLA::Matrix<EGI_const::ss_var,EGI_const::meas_var> K;
    
    
    // x_k = F_(k-1)*x_(k-1) + G_(k-1)*u_(k-1) + w_(k-1) -- F_(k-1) equivalent to the A matrix of a SS representation at time step k-1, G_(k-1) "" B matrix "", w_(k-1) is the state noise -- this matrix may change if delta_t is not constant between time steps
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> F_t;
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> F;
    
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> std_F_t;
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> std_F;

    // x_k = F_(k-1)*x_(k-1) + G_(k-1)*u_(k-1) + w_(k-1) -- F_(k-1) equivalent to the A matrix of a SS representation at time step k-1, G_(k-1) "" B matrix "", w_(k-1) is the state noise
    const BLA::Matrix<EGI_const::ss_var,EGI_const::input_var> G = {0,0,0,
                                                                   0,0,0,
                                                                   0,0,0,
                                                                   0,0,0,
                                                                   0,0,0,
                                                                   0,0,0,
                                                                   1,0,0,
                                                                   0,1,0,
                                                                   0,0,1};
    
    // Covariance matrix of the state noise (diag if SS variables are indep.) -- a 0 means no noise if on the diag (variance) and no coupling outside (covariance).
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> Q;
    
    
    // Covariance matrix of the estimation error (diag) -- initialized at 0 (or close to) if the initial state is perfectly known, +infinity if initial state is unknown
    BLA::Matrix<EGI_const::ss_var,EGI_const::ss_var> P;
    
    
    BLA::Matrix<EGI_const::meas_var,EGI_const::meas_var> tempMeasMeas;
    BLA::Matrix<EGI_const::pos_var,EGI_const::pos_var> tempPosPos;


  public :

    //SensorSys, SensingSystem* : pointer to the object containing the entirety of the sensors on board
    //IMU_refreshRate, Hz : refresh rate of the IMU that will be used
    //GPS_refreshRate, Hz : refresh rate of the GPS that will be used
    //VarUpd, bool : specifies wether or not the covariance matrices will be updated during flight or not (experimental)
    EGI_obj(SensingSystem* SensorSys);
    
    //function to initialize the kalman filter (covariance matrices). Must be run after all other sensors are initialized and ready since this uses them.
    //returns the time that the loop() function running the kalman filter must not exceed if initialized succesfully, otherwise 0
    unsigned long initKalman();

    //function to acquire the initial position of the rocket
    //returns true if succesful, false otherwise
    bool initialState();
  
    //computes the F matrix (A matrix of the SS representation of the system) which depends on the time step in seconds (this is tied to the system's model)
    //stepChange, bool : indicates wether this function should revert to the standard F matrix (standard time step was respected) or to recomputed F (with new time step)
    //newTimeStep, seconds : time step used to compute the F matrix (may vary in flight)
    void updateF(bool stepChange, double newTimeStep);
    
    //getNavSolution provides time since launch, Kalman filter position and altitude
    //Returns
    //_time - Time since launch (milliseconds)
    //x - Kalman ECEF Position x (m)
    //y - Kalman ECEF Position y (m)
    //z - Kalman ECEF Position x (m)
    //altitude - WGS84 altitude (m)
    NavSolution getNavSolution();
};