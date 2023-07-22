#include <fault_debug.h>

//Initializes debug instance with full debug and Serial7 hardware channel by default, ID must be specified (and for clarity, should be unique)
fault_debug::fault_debug(String module_ID, uint8_t level, HardwareSerial hard_chan):ID(module_ID),l(level),Serial_HW(hard_chan)
{}

//Starts serial ports (Serial_USB or Serial_HW (at 115200 by def)). The use of the hardware channel is not mandatory but replaces the USB channel. 
bool fault_debug::begin(bool enableHW_chan, unsigned long baud_hard)
{
    if(!enableHW_chan && !debug::Serial_USB)
    {
        debug::Serial_USB.begin(debug::USB_baud);
    }

    if(enableHW_chan && !Serial_HW)
    {
        set_HWbaud(baud_hard);
        Serial_HW.begin(HW_baud);
    }

    if((!enableHW_chan && debug::Serial_USB) || (enableHW_chan && Serial_HW))
    {
        chan_rdy = true;
        HW_chan = enableHW_chan;
        println(debugLevel::INFO, "Debug Initialized", "fault_debug::begin");
        return true;
    }
    else
    {
        return false;
    }
}

void fault_debug::set_HWbaud(unsigned long baud_hard)
{
    HW_baud = baud_hard;
}

//Simply sets the debug level to 0 (no output), none of the Serial ports are closed to avoid conflicts unless specified for HW channel
void fault_debug::end(bool closeHW_chan)
{
    l = debugLevel::NONE;

    if(closeHW_chan)
    {
        Serial_HW.end();
    }
}

void fault_debug::set_subID(String subID)
{
    _IDsub = subID;
}

// Prints a line without going to the next, msg structure is "ID | subID | debugLevelName | msg", subID is by default "" but is stored in memory if specified once, use set_subID if you only want to change the subID without printing
void fault_debug::print(uint8_t level, String msg, String sub_ID, bool skipFormat)
{
    if(chan_rdy && l>=level && level != 0)
    {
        if(sub_ID != "") set_subID(sub_ID);
        String compl_msg = ID + " | " + _IDsub + " | " + debugLevel::name[level-1] + " | " + msg;

        if(skipFormat)
        {
            compl_msg = msg;
        }
        
        if(HW_chan && Serial_HW)
        {
            Serial_HW.print(compl_msg);
        }
        else if(debug::Serial_USB && !HW_chan)
        {
            debug::Serial_USB.print(compl_msg);
        }
    }
}

// Prints a line and goes to the next, see fault_debug::print()
void fault_debug::println(uint8_t level, String msg, String sub_ID)
{
    print(level, msg + "\r\n", sub_ID);
}

void fault_debug::write(uint8_t level, char c)
{
    print(level,c,"",true);
}

bool fault_debug::isLogged(uint8_t level)
{
    return l>=level;
}