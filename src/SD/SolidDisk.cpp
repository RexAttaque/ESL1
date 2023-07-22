#include <SD/SolidDisk.h>

SD_obj::SD_obj()
{
    log_filename.toCharArray(log_filename_charred,log_filename.length());
}

void SD_obj::setLogName(String newName)
{
    log_filename = newName;
    log_filename.toCharArray(log_filename_charred, log_filename.length());
}

SDClass* SD_obj::getSD()
{
    return &SD;
}

String SD_obj::getLogName()
{
    return log_filename;
}

char* SD_obj::getCharredLogName()
{
    return log_filename_charred;
}

File SD_obj::getLog()
{
    return log;
}

bool SD_obj::init(uint8_t cspin)
{
    bool success = false;

    if(SD.begin(cspin))
    {    
        while(SD.exists(log_filename_charred))
        {
            logID++;
            setLogName(SD_const::logname + logID + SD_const::logext);
        }

        log = SD.open(log_filename_charred, FILE_WRITE_BEGIN);

        if(log)
        {
            log.println("Log ID : " + String(logID) + " created ! SD card reader initialized...");
            log.flush();
            success = true;
        }
    }

    if(success)
    {
        debug_SD.println(debugLevel::INFO, "SD Init SUCCESS !", "init()");
        return true;
    }
    else
    {
        debug_SD.println(debugLevel::INFO, "SD Init Failed !", "init()");
        return false;
    }
}

void SD_obj::writeLog(String toWrite)
{
    log.print(toWrite);
}

void SD_obj::writeLogLine(String toWrite)
{
    log.println(toWrite);
}

bool SD_obj::openLog(String logname)
{
    closeLog();
    setLogName(logname);
    File log_temp = SD.open(log_filename_charred, FILE_WRITE_BEGIN);
    if(log_temp)
    {
        log = log_temp;
        return true;
    }
    else
    {
        setLogName(log.name());
        return false;
    }
}

void SD_obj::closeLog()
{
    if(log) log.close();
}
