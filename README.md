# ESL1

Code Repository for the ESO ESL1 (ESTACA Space Launcher) Student Rocket.
The main goal is to provide accurate position and particularily altitude data via sensor fusion, to ensure parachute deployement at the proper altitude and tracking of the rocket

Structure is as follows :

-> Adafruit BNO055 IMUs, Ublox Gen9 GPS, Adafruit BMP280 Baro sensors (and their respective interface librairies to acquire measurements)

-> IMU, GPS, BARO objects (IMU_Avio, GPS_Avio and BARO_Avio objects, with "Avio" indicating their location, the goal is to format measurements into a universal array format) 

-> Sensors object (Sensors object, the goal is to combine the data from all sensors of one type (IMU_Avio for example which for ESL1 consists of 2 IMUs))

-> Sensors System object (SensingSystem object, the goal is to centralize all Sensors objects and combine their data into a final form (to combine the data of IMUs in the Avionics bay and in the tail for example, transparent in the case of ESL1))

-> EGI (Embedded GPS/INS, Kalman filter algorithm, pulls data from the SensingSystem and fuses GPS/IMU measurements to provide a more accurate position/altitude readout in the ECEF referential. Also keeps track of time, incorporates barometric backup for altitude computation)

=> Navigation solution

=> Relayed to the ground via telemetry

=> Altitude computation/timer for parachute deployement

=> Backup GSM module to transmit position upon landing
