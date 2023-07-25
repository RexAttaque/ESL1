#include <TELEM\ModuleLibs\SX1276MB1MAS.h>

TELEM_obj::TELEM_obj(HardwareSerial HWSerial, long Baud, unsigned long freq, int8_t power, uint8_t spreadfactor, unsigned long bandwidth, uint8_t codingRate):fault_debug(TELEM_const::debug_ID, TELEM_const::debug_lvl),TELEM_Serial(HWSerial),TELEM_baudrate(Baud)
{}

bool TELEM_obj::setFreq()
{
    return false;
}

bool TELEM_obj::setPwr()
{
    return false;
}

bool TELEM_obj::setSF()
{
    return false;
}

bool TELEM_obj::setBW()
{
    return false;
}

bool TELEM_obj::setCR()
{
    return false;
}


unsigned long TELEM_obj::getFreq()
{
    return false;
}

int8_t TELEM_obj::getPwr()
{
    return false;
}

uint8_t TELEM_obj::getSF()
{
    return false;
}

unsigned long TELEM_obj::getBW()
{
    return false;
}

uint8_t TELEM_obj::getCR()
{
    return false;
}



bool TELEM_obj::init()
{
    return false;
}

bool TELEM_obj::goLive()
{
    return false;
}

bool TELEM_obj::goIdle()
{
    return false;
}


bool TELEM_obj::uploadPacket()
{
    return false;
}
