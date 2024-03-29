#pragma once

#include <Arduino.h>
#include <SD.h>

namespace SD_const {
    const String logname = "log_";
    const String logext = ".txt";
}

class SD_obj {
    private :
        unsigned long logID = 1;
        String log_filename = SD_const::logname + logID + SD_const::logext;
        char* log_filename_charred;
        File log;

    public :

        SD_obj(); 

        void setLogName(String newName);
        //void setLogID(unsigned long setID);

        unsigned long getLogID();
        String getLogName();
        char* getCharredLogName();
        File getLog();

        bool init(uint8_t cspin = BUILTIN_SDCARD);

        void writeLog(String toWrite);
        void writeLogLine(String toWrite);
        void flushLog();

        bool openLog(String filename, bool truncate = false);
        void closeLog();
};