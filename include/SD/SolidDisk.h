#pragma once

#include <Arduino.h>
#include <SD.h>
#include <fault_debug.h>

namespace SD_const {
    const String debug_ID = "SD";
    const uint8_t debug_lvl = debugLevel::FULL;

    const String logname = "log_";
    const String logext = ".txt";
}

class SD_obj {
    private :
        fault_debug debug_SD = fault_debug(SD_const::debug_ID, SD_const::debug_lvl); //Debug object for the SD module

        SDClass SD;

        unsigned long logID = 1;
        String log_filename = SD_const::logname + logID + SD_const::logext;
        char* log_filename_charred;
        File log;

    public :

        SD_obj(); 

        void setLogName(String newName);
        //void setLogID(unsigned long setID);

        SDClass* getSD();
        String getLogName();
        char* getCharredLogName();
        File getLog();

        bool init(uint8_t cspin = BUILTIN_SDCARD);

        void writeLog(String toWrite);
        void writeLogLine(String toWrite);
        bool openLog(String filename);
        void closeLog();
};