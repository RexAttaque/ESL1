#pragma once

#include <Arduino.h>
#include <SD/SolidDisk.h>

namespace faultCodes {
    enum faultCodes {
        pos = 0,
        altitude = -9999, //if this altitude is thrown out, whatever altitude measurement method failed
    };
}

namespace debugLevel {
    const uint8_t FULL = 3; //All debug logs
    const uint8_t TRACE = 2; //All debug logs that allow to track progress
    const uint8_t INFO = 1; //Only information
    const uint8_t NONE = 0; //Nothing

    const String name[FULL] = {"INFO", "TRACE", "FULL"}; //Name of the debug levels for display
}

namespace debug {
    const bool useSD = true;
    const bool externalLogger = false;
    const bool externalLoggerDebug = true;

    const unsigned long USB_baud = 115200; //USB baud rate (115200 by def)

    const unsigned long HW_baud = 115200; //Default hardware serial channel baud for debug (115200 by def)
}

class fault_debug
{
private:
    bool chan_rdy = false;
    bool sd_rdy = false;

    SD_obj SolidDisk;

    bool USB_chan = true;

    bool HW_chan = false;
    HardwareSerial Serial_HW; //if a hardware serial needs to be used for debug, to send to telem or something

    String ID; //ID of the debug instance
    String _IDsub = ""; //Set sub ID that will be printed along with the ID, can be changed
    uint8_t l; //Debug level of the debug instance (the higher the level the more is displayed, see debugLevel namespace for levels and comments)
public:
    fault_debug(String module_ID, uint8_t level = debugLevel::FULL, bool enableHW_chan = debug::externalLogger, HardwareSerial hard_chan = Serial7);

    bool begin(bool attemptSD = debug::useSD, bool USB_and_HW = debug::externalLoggerDebug);
    void end(bool closeSD_chan = !debug::useSD, bool closeHW_chan = !debug::externalLogger);

    SD_obj* get_SD_obj();

    void set_level(uint8_t level);
    void set_subID(String sub_ID);

    void print(uint8_t level, String msg, String sub_ID = "", bool skipFormat = false);
    void println(uint8_t level, String msg, String sub_ID = "");
    void skipln(uint8_t level, uint8_t amount = 1);
    void write(uint8_t level, char c);

    bool isLogged(uint8_t level);
};