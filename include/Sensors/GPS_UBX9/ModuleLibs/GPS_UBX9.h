#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

namespace UBX9_const {
  const String debug_ID = "GPS_UBX9";
  const uint8_t debug_lvl = debugLevel::FULL;

  const uint16_t Hz = 25; //default GPS refresh rate (Hz)
  const uint8_t varAmount = 5; //time,x,y,z,CEP (ms ; cm)
  const double measFactors[varAmount] = {1.0f,100.0f,100.0f,100.0f,100.0f}; //conversion factors for each measurand

  const uint8_t minFixStatus = 3; //When using GPS.getNavFixStatus(), what is the minimum fix state that is acceptable
  const uint8_t maxFixStatus = 3; //When using GPS.getNavFixStatus(), what is the maximum fix state that is acceptable
  const uint8_t maxCalibrationAttempts = 10; //maximum number of allowed calibration attempts before cancelling
  const unsigned long timeBetweenCalibrations = 60000; //millis of delay between each calibration attemps
  const unsigned long calibrationSettleTime = 300000; //millis of delay after calibration complete to allow settling
};

class GPS_UBX9 : public Base_Sensor<double> {
  private :
    fault_debug debug_GPSUBX9 = fault_debug(UBX9_const::debug_ID, UBX9_const::debug_lvl); //Debug object for the GPS_UBX9 module

    ublox_gen9 GPS;
    uint8_t fixType;
  
  public :
  
    GPS_UBX9(HardwareSerial HWSerial = Serial1, long Baud = 115200, uint8_t Pltfrm_Model = 8, uint16_t refreshRate = UBX9_const::Hz, uint8_t Nav_Rate = 1, uint8_t config_Level = 7, long default_Baud = 38400, bool buffered_POS = false, bool NMEA_USB = false, long USB_Baud = 115200, bool NMEA_UART1 = false, uint8_t Stop_Bits = 1);

    ublox_gen9* getSensor();

    bool init();
    bool calibrate();

    bool goLive();
    bool goIdle();

    bool getStatus();

    double* getMeas();
};