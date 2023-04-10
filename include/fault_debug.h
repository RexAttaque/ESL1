#pragma once

namespace debug {
    bool _full = false;
    bool _trace = false;
    bool _info = false;

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
        altitude = -9999, //if this altitude is thrown out, whatever altitude measurement method failed
    };
}
