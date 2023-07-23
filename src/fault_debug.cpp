#include <fault_debug.h>

//Initializes debug instance with full debug and Serial7 hardware channel by default, ID must be specified (and for clarity, should be unique)
fault_debug::fault_debug(String module_ID, uint8_t level, HardwareSerial hard_chan):ID(module_ID),l(level),Serial_HW(hard_chan)
{}

//Starts serial ports (Serial or Serial_HW (at 115200 by def)). The use of the hardware channel is not mandatory but replaces the USB channel. 
bool fault_debug::begin(bool enableHW_chan, unsigned long baud_hard)
{
    //SD Check/Init
    sd_rdy = SolidDisk.init();

    if(!enableHW_chan && !Serial)
    {
        Serial.begin(debug::USB_baud);
    }

    if(enableHW_chan && !Serial_HW)
    {
        set_HWbaud(baud_hard);
        Serial_HW.begin(HW_baud);
    }

    if((!enableHW_chan && Serial) || (enableHW_chan && Serial_HW))
    {
        chan_rdy = true;
        HW_chan = enableHW_chan;
        println(debugLevel::INFO, "Debug Initialized Args : " + String(enableHW_chan) + " , " + String(baud_hard), "begin(enableHW_chan, HW_baud)");
        return true;
    }
    else
    {
        return false;
    }
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

SD_obj* fault_debug::get_SD_obj()
{
    return &SolidDisk;
}

void fault_debug::set_HWbaud(unsigned long baud_hard)
{
    HW_baud = baud_hard;
}

void fault_debug::set_level(uint8_t level)
{
    l = level;
}

void fault_debug::set_subID(String subID)
{
    _IDsub = subID;
}

// Prints a line without going to the next, msg structure is "ID | subID | debugLevelName | msg", subID is by default "" but is stored in memory if specified once, use set_subID if you only want to change the subID without printing
void fault_debug::print(uint8_t level, String msg, String sub_ID, bool skipFormat)
{
    if(isLogged(level))
    {
        if(sub_ID != "") set_subID(sub_ID);
        String compl_msg = ID + " | " + _IDsub + " | " + debugLevel::name[level-1] + " | " + msg;

        if(skipFormat)
        {
            compl_msg = msg;
        }

        if(sd_rdy) SolidDisk.writeLog(compl_msg);
        
        if(HW_chan && Serial_HW)
        {
            Serial_HW.print(compl_msg);
        }
        else if(Serial && !HW_chan)
        {
            Serial.print(compl_msg);
        }
    }
}

// Prints a line and goes to the next, see fault_debug::print()
void fault_debug::println(uint8_t level, String msg, String sub_ID)
{
    print(level, msg + '\r\n', sub_ID);
}

void fault_debug::write(uint8_t level, char c)
{
    print(level,c,"",true);
}

void fault_debug::skipln(uint8_t level, uint8_t amount)
{
    for(int i=0;i<amount;i++)
    {
        write(level, '\n');
    }
}

bool fault_debug::isLogged(uint8_t level)
{
    return chan_rdy && l>=level && level != 0;
}