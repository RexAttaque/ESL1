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

unsigned long SD_obj::getLogID()
{
    return logID;
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
    if(SD.begin(cspin))
    {    
        while(SD.exists(log_filename_charred))
        {
            logID++;
            setLogName(SD_const::logname + logID + SD_const::logext);
        }

        if(openLog(log_filename))
        {
            writeLogLine("Log ID : " + String(logID) + " created ! SD card reader initialized...");
            flushLog();
            return true;
        }
    }

    return false;
}

void SD_obj::writeLog(String toWrite)
{
    log.print(toWrite);
}

void SD_obj::writeLogLine(String toWrite)
{
    log.println(toWrite);
}

void SD_obj::flushLog()
{
    log.flush();
}

bool SD_obj::openLog(String logname, bool truncate)
{
    closeLog();
    setLogName(logname);

    uint16_t flag = O_WRITE | O_CREAT;
    if(truncate)
    {
        flag |= O_TRUNC;
    }

    File log_temp = SD.open(log_filename_charred, flag);
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
