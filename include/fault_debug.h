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
}

namespace faultCodes {
    typedef enum faultCodes {
        pos = 0,
        altitude = -9999, //if this altitude is thrown out, whatever altitude measurement method failed
    };
}
