# ESL1_master

Code Repository for the ESO ESL1 (ESTACA Space Launcher) Student Rocket.
The main goal is to provide accurate position and particularily altitude data via sensor fusion, to ensure parachute deployement at the proper altitude and tracking of the rocket

Structure is as follows :

-> Adafruit BNO055 IMUs, Ublox Gen9 GPS, Adafruit BMP280 Baro sensors 

-> IMU, GPS, BARO objects 

-> Sensors object 

-> EGI (Embedded GPS/INS, a Kalman filter algorithm)

=> Navigation solution

=> Relayed to the ground via telemetry

=> Altitude computation/timer for parachute deployement

=> Backup GSM module to transmit position upon landing
