#pragma once

#include <Arduino.h>
#include <fault_debug.h>
#include <Sensors/Base_Sensor.h>
#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

GPS_UBX9 GPS1_Avio = GPS_UBX9();
const uint8_t qty_GPS_Avio = 1;
GPS_UBX9* GPS_Avio_array;

namespace UBX9_const {
  const uint16_t Hz = 25; //default GPS refresh rate (Hz)
  const uint8_t varAmount = 5; //time,x,y,z,CEP (ms ; cm)

  const uint8_t minFixStatus = 3; //When using GPS.getNavFixStatus(), what is the minimum fix state that is acceptable
  const uint8_t maxFixStatus = 3; //When using GPS.getNavFixStatus(), what is the maximum fix state that is acceptable
  const uint8_t maxCalibrationAttemps = 10; //maximum number of allowed calibration attempts before cancelling
  const unsigned long timeBetweenCalibrations = 60000; //millis of delay between each calibration attemps
  const unsigned long calibrationSettleTime = 300000; //millis of delay after calibration complete to allow settling
};

class GPS_UBX9 : public Base_Sensor<long> {
  private :
    ublox_gen9 GPS;
  
  public :
  
    GPS_UBX9(HardwareSerial HWSerial = Serial1, long Baud = 115200, uint8_t Pltfrm_Model = 8, uint16_t refreshRate = UBX9_const::Hz, uint8_t Nav_Rate = 1, uint8_t config_Level = 7, long default_Baud = 38400, bool buffered_POS = false, bool NMEA_USB = false, long USB_Baud = 115200, bool NMEA_UART1 = false, uint8_t Stop_Bits = 1);

    bool init();
    bool goLive();
    bool goIdle();
    bool calibrate();

    uint8_t getStatus();

    long* getMeas();
};