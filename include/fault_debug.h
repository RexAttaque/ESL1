#pragma once

#include <Arduino.h>

namespace debug {
    bool _full = false;
    bool _trace = false;
    bool _info = false;

    usb_serial_class Serial = Serial;
    //HardwareSerial Serial = SerialX; //if a hardware serial wants to be used for debug, to send to telem or something

    bool full()
    {
        return _full;
    }

    bool trace()
    {
        return (_full || _trace);
    }

    bool info()
    {
        return (_full || _trace || _info);
    }

    void disable()
    {
        //if(debug::info()) debug::Serial.println("Good night ESO, see you after the flight... TM"); //To re-enable last minute
        _full = false;
        _trace = false;
        _info = false;
        debug::Serial.end();
    }
}

namespace faultCodes {
    enum faultCodes {
        pos = 0,
        altitude = -9999, //if this altitude is thrown out, whatever altitude measurement method failed
    };
}
