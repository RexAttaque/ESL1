#include <fault_debug.h>

//Initializes debug instance with full debug and Serial7 hardware channel by default, ID must be specified (and for clarity, should be unique) + Starts serial ports (Serial or Serial_HW (at 115200 by def) (both can be started using USB_and_HW for debug)). The use of the hardware channel is not mandatory but replaces the USB channel. 
fault_debug::fault_debug(String module_ID, uint8_t level, bool enableHW_chan, HardwareSerial hard_chan):ID(module_ID),l(level),HW_chan(enableHW_chan),Serial_HW(hard_chan)
{
    if(level!=debugLevel::NONE) begin();
}

//Starts serial ports (Serial or Serial_HW (at 115200 by def)). The use of the hardware channel is not mandatory but replaces the USB channel unless specified otherwise (for debugging of the hardware pass through). SD card init is attempted as well if desired.
bool fault_debug::begin(bool attemptSD, bool USB_and_HW)
{
    if(attemptSD)
    {
        //SD Check/Init
        sd_rdy = SolidDisk.init();
    }

    if((!HW_chan || USB_and_HW) && !Serial)
    {
        Serial.begin(debug::USB_baud);
    }

    if(HW_chan && !Serial_HW)
    {
        Serial_HW.begin(debug::HW_baud);
    }

    if((!HW_chan && Serial) || (HW_chan && Serial_HW))
    {
        chan_rdy = true;
        println(debugLevel::INFO, "Debug Module ID : " + ID + " Initialized at level : " + String(l) + " on hardware channel : " + String(HW_chan), "begin(ID,lvl,HW_chan,HW_serial)");
        if(sd_rdy) println(debugLevel::INFO, "Debug is being written to SD");
    } 
}

//Disables USB serial logging and SD and/or hardware Serial logging without changing any of the current instance 
void fault_debug::end(bool closeSD_chan, bool closeHW_chan)
{
    if(Serial) Serial.end();

    if(closeSD_chan && sd_rdy)
    {
        sd_rdy = false;
        if(HW_chan == false) chan_rdy = false;
        SolidDisk.closeLog();
    }    

    if(closeHW_chan && chan_rdy && Serial_HW)
    {
        Serial_HW.end();
    }
}

SD_obj* fault_debug::get_SD_obj()
{
    return &SolidDisk;
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