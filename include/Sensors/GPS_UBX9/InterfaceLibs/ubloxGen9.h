#pragma once

#ifndef __ublox_Gen9__
#define __ublox_Gen9__

#include <Arduino.h>
#include <HardwareSerial.h>

/////////////////////////////////////////////////// key IDs /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// NAV RATE //////////////////
const long CFG_RATE_MEAS = 0x30210001; //measurement rate, minimum is 25ms (40Hz)
const long CFG_RATE_NAV = 0x30210002; //number of measurements between each nav solution (maximum 128, minimum 1)

/////////////// DYNAMIC MODEL //////////////////
const long CFG_NAVSPG_DYNMODEL = 0x20110021; //dynamic platform model (airborn, on ground etc.)

/////////////// UART1 GENERAL //////////////////
//const long CFG_UART1_ENABLED = 0x10520005; //enable UART1 via ucenter if it is not already (UART1 NEEDS TO BE ENABLED FOR ALL COMMUNICATIONS, IT IS BY DEFAULT)
const long CFG_UART1_BAUDRATE = 0x40520001; //Baud rate of UART1.
const long CFG_UART1_STOPBITS = 0x20520002; //Number of stop bits, test if 0.5 stop bits works, may accelerate transfers (1 stop bit by default)

/////////////// UART1 PROTOCOLS //////////////////
//const long CFG_UART1INPROT_UBX = 0x10730001; //Input protocol for UART1 : UBX. Check that it is enabled via u-center (MUST BE ENABLED FOR ALL COMMUNICATIONS, IT IS BY DEFAULT)
const long CFG_UART1INPROT_NMEA = 0x10730002; //enabled/disabled on demand
const long CFG_UART1INPROT_RTCM3X = 0x10730004; //disabled
const long CFG_UART1OUTPROT_UBX = 0x10740001; //Output protocol for UART 1 : UBX (MUST BE ENABLED FOR ALL COMMUNICATIONS, IT IS BY DEFAULT)
const long CFG_UART1OUTPROT_NMEA = 0x10740002; //enabled/disabled on demand

/////////////// UART2 GENERAL //////////////////
const long CFG_UART2_ENABLED = 0x10530005; //UART2 will be disabled, not used

/////////////// I2C GENERAL //////////////////
const long CFG_I2C_ENABLED = 0x10510003; //I2C will be disabled, not used

/////////////// SPI GENERAL //////////////////
const long CFG_SPI_ENABLED = 0x10640006; //SPI will be disabled, not used

/////////////// USB PROTOCOLS //////////////////
const long CFG_USBINPROT_UBX = 0x10770001; // Input UBX protocol, always allowed
const long CFG_USBINPROT_NMEA = 0x10770002; //Input NMEA protocol through USB is not used by u-center or by us, disabled
const long CFG_USBOUTPROT_UBX = 0x10780001; // Output UBX protocol, always allowed, all output messages disabled though, only for configuration
const long CFG_USBOUTPROT_NMEA = 0x10780002; //Output NMEA protocol allows monitoring of GPS data through USB, will be disabled if not in monitoring mode. USB and UBX protocol through USB will always remain enabled (no message output in UBX by default, simply for configuration). NEVER DISABLE USB THROUGH UCENTER

/////////////// MESSAGES //////////////////
const int msgCount = 46;
const long msgKeys[msgCount] = {0x2091025a, 0x20910350, 0x209101ba, 0x20910355, 0x209101b5, 0x209101a6, 0x20910197, 0x2091035a, 
                                0x209101a1, 0x20910188, 0x2091038c, 0x2091019c, 0x2091007a, 0x20910066, 0x20910084, 0x20910039,
                                0x20910314, 0x20910160, 0x209100a2, 0x2091007f, 0x20910011, 0x20910025, 0x2091002a, 0x20910007, 
                                0x20910016, 0x2091006b, 0x20910346, 0x20910337, 0x2091001b, 0x20910052, 0x20910057, 0x2091004d,
                                0x20910048, 0x20910061, 0x20910387, 0x2091005c, 0x2091003e, 0x20910043, 0x20910205, 0x209102a5,
                                0x2091025f, 0x20910269, 0x20910232, 0x20910179, 0x2091017e, 0x20910093};

// const long CFG_MSGOUT_UBX_LOG_INFO_UART1 = 0x2091025a; //+2 to get that key to get the key for the same message but for the USB interface
// const long CFG_MSGOUT_UBX_MON_COMMS_UART1 = 0x20910350;
// const long CFG_MSGOUT_UBX_MON_HW2_UART1 = 0x209101ba;
// const long CFG_MSGOUT_UBX_MON_HW3_UART1 = 0x20910355;
// const long CFG_MSGOUT_UBX_MON_HW_UART1 = 0x209101b5;
// const long CFG_MSGOUT_UBX_MON_IO_UART1 = 0x209101a6;
// const long CFG_MSGOUT_UBX_MON_MSGPP_UART1 = 0x20910197;
// const long CFG_MSGOUT_UBX_MON_RF_UART1 = 0x2091035a;
// const long CFG_MSGOUT_UBX_MON_RXBUF_UART1 = 0x209101a1;
// const long CFG_MSGOUT_UBX_MON_RXR_UART1 = 0x20910188;
// const long CFG_MSGOUT_UBX_MON_SPAN_UART1 = 0x2091038c;
// const long CFG_MSGOUT_UBX_MON_TXBUF_UART1 = 0x2091019c;
// const long CFG_MSGOUT_UBX_NAV_AOPSTATUS_UART1 = 0x2091007a;
// const long CFG_MSGOUT_UBX_NAV_CLOCK_UART1 = 0x20910066; //CLOCK
// const long CFG_MSGOUT_UBX_NAV_COV_UART1 = 0x20910084; //COVariance matrix
// const long CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039; //DOP - Degree Of Precision
// const long CFG_MSGOUT_UBX_NAV_EELL_UART1 = 0x20910314;
// const long CFG_MSGOUT_UBX_NAV_EOE_UART1 = 0x20910160;
// const long CFG_MSGOUT_UBX_NAV_GEOFENCE_UART1 = 0x209100a2;
// const long CFG_MSGOUT_UBX_NAV_ODO_UART1 = 0x2091007f;
// const long CFG_MSGOUT_UBX_NAV_ORB_UART1 = 0x20910011;
const long CFG_MSGOUT_UBX_NAV_POSECEF_UART1 = 0x20910025; //POSECEF
// const long CFG_MSGOUT_UBX_NAV_POSLLH_UART1 = 0x2091002a; //POSLLH
// const long CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007; //PVT
// const long CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016;
// const long CFG_MSGOUT_UBX_NAV_SBAS_UART1 = 0x2091006b;
// const long CFG_MSGOUT_UBX_NAV_SIG_UART1 = 0x20910346;
// const long CFG_MSGOUT_UBX_NAV_SLAS_UART1 = 0x20910337;
// const long CFG_MSGOUT_UBX_NAV_STATUS_UART1 = 0x2091001b;
// const long CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1 = 0x20910052;
// const long CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1 = 0x20910057;
// const long CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1 = 0x2091004d;
// const long CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1 = 0x20910048;
// const long CFG_MSGOUT_UBX_NAV_TIMELS_UART1 = 0x20910061;
// const long CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1 = 0x20910387;
// const long CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1 = 0x2091005c;
// const long CFG_MSGOUT_UBX_NAV_VELECEF_UART1 = 0x2091003e;
// const long CFG_MSGOUT_UBX_NAV_VELNED_UART1 = 0x20910043;
// const long CFG_MSGOUT_UBX_RXM_MEASX_UART1 = 0x20910205;
// const long CFG_MSGOUT_UBX_RXM_RAWX_UART1 = 0x209102a5;
// const long CFG_MSGOUT_UBX_RXM_RLM_UART1 = 0x2091025f;
// const long CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269;
// const long CFG_MSGOUT_UBX_RXM_SFRBX_UART1 = 0x20910232;
// const long CFG_MSGOUT_UBX_TIM_TM2_UART1 = 0x20910179;
// const long CFG_MSGOUT_UBX_TIM_TP_UART1 = 0x2091017e;
// const long CFG_MSGOUT_UBX_TIM_VRFY_UART1 = 0x20910093;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ublox_gen9 {
  
  private:

    //////////////////////////////////////////////////// CONFIG //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long USB_Baudrate; // USB Baudrate for communication with the motherboard

    uint8_t configLevel; //level at which all configs will be applied
    // 0 - RAM
    // 1 - BBR
    // 2 - Flash
    // 7 - Default value (All of the above)  

    uint16_t GPS_TimeToRefresh; // time between measurements in ms
    uint8_t GPS_NavRate; // Time to refresh divider for more robust Nav solutions

    uint8_t GPS_PltfrmModel; //Airborn <4g platform model
    //0 Portable
    //2 Stationary
    //3 Pedestrian
    //4 Automotive
    //5 Sea
    //6 Airborne with <1g acceleration
    //7 Airborne with <2g acceleration
    //8 Airborne with <4g acceleration
    //9 Wrist-worn watch (not available in all products)

    long GPS_default_Baudrate; // UART baud rate for communication with the GPS (default 38400)
    long GPS_Baudrate; // UART baud rate for communication with the GPS (max 115200)
    uint8_t GPS_StopBits; // Number of stop bits for Serial communications with the GPS
    //0 is 0.5 stop bit
    //1 is 1 stop bit
    //2 is 1.5 stop bit
    //3 is 2 stop bits
    
    bool UART1_NMEA; //Input and Output NMEA protocol toggle for UART1, for troubleshooting with another setup only. NOT COMPATIBLE WITH "bufferedPOSECEF" which will take precedence.
    bool USB_NMEA; //Input and Output NMEA protocol toggle for USB, for monitoring through ucenter only.

    bool bufferedPOSECEF; //All output messages by the GPS are disabled by default, however it is possible to activate the output of the main message we need, POSECEF, to accelerate it's recovery. Will make polling other values almost impossible however and the time saving is so marginal, not worth it.
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    HardwareSerial GPS_Serial;
    
    //////////////////////////////////////////////////////// Init ///////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool setBasicConfig();

    /////////////////////////////////////////////// Support Core ////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    bool calculateCheckSum(uint8_t* frame, uint16_t frameSize, bool receiveMode);
    
    //splits a 64 bit uint to an array of 8 bit uints
    //value and value length in bytes
    uint8_t* splitTo8bit(uint64_t value, uint16_t val_length);
    
    /////////////////////////////////////////////// Core UART ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //extracted length in bytes counting start bytes etc. (full frame length)
    uint8_t* receiveUBXframe(uint16_t extractedLength);
    
    bool sendUBXframe(uint8_t cmdClass, uint8_t messageID, uint16_t payloadSize, uint8_t* payload, bool configMode);
    
    /////////////////////////////////////////////// Core GPS Config //////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //level 0 -> RAM
    //level 2 -> Battery backup RAM (BBR)
    //level 4 -> Flash
    //level 7 -> All
    //One or all of the above can be edited at once (All == 7)
    bool setConfig(uint8_t level, long keyID, uint64_t value, uint8_t valueSize);

    // 0 - RAM
    // 1 - BBR
    // 2 - Flash
    // 7 - Default value
    uint8_t* getConfig(long keyID, uint8_t level, uint8_t expectedValueSize);

    /////////////////////////////////////////////// Core GPS Polling /////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    uint8_t* PollValue(uint8_t cmdClass, uint8_t messageID, uint16_t expectedPayloadSize, bool expectedInBuffer);

    /////////////////////////////////////////////// GPS Interface /////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool setRefreshRate(uint16_t refreshRate);

    bool setNavRate();
    
    bool setPltfrmModel();
    
    bool setUART1gen_config();
    
    bool setUART1prot_config();
    
    bool setUART1msg_config();
    
    bool setUART2gen_config();
    
    bool setSPIgen_config();
    
    //already disabled via SEL pin, crashes if you try to set it
    //bool setI2Cgen_config();
    
    bool setUSBprot_config();
    
    bool setUSBmsg_config();


  public:

    //////////////////////////////////////////////////////// Init ///////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    ublox_gen9(HardwareSerial HWSerial = Serial1, long GPS_Baud = 115200, uint8_t GPS_Pltfrm_Model = 8, uint16_t GPS_Time_To_Refresh = 40, uint8_t GPS_Nav_Rate = 1, uint8_t config_Level = 7, long GPS_default_Baud = 38400, bool buffered_POS = false, bool NMEA_USB = false, long USB_Baud = 115200, bool NMEA_UART1 = false, uint8_t GPS_Stop_Bits = 1);
    
    bool initGPS();

    /////////////////////////////////////////////// Support Debug ///////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //void print64(uint64_t value);

    //void print64ln(uint64_t value);
    
    /////////////////////////////////////////////// GPS Interface /////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //can either reset the GPS to load from flash or load from flash simply
    void resetGPS();
    
    //uint16_t getRefreshRate(uint8_t level);
    
    long* getPOSECEF();

    uint8_t getNavFixStatus();
};

#endif