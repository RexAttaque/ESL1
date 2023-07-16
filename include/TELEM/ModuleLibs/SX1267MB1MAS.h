#pragma once

#include <Arduino.h>
#include <fault_debug.h>

namespace TELEM_const {
    const uint8_t PacketSize = 38;
    // const unsigned long FREQ = 433E6;   // Frequency band (This unit is optimized for 433MHz (Low Freq. / LF) and 868MHz (High Freq. / HF))
    // const int8_t PWR = 20;              // Power (0-20)
    // const uint8_t SF = 9;               // Spreading factor (6-12)
    // const unsigned long BW = 250E3;     // Bandwidth (10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500 kHz)
    // const uint8_t CR = 8;               // Coding rate (5 -> 4/5 (1.25 overhead), 6 -> 4/6 (1.5), 7 -> 4/7 (1.75), 9 -> 4/8 (2))
}

struct TELEM_packet {
    float data[TELEM_const::PacketSize];
};

class TELEM_obj {
    private :
        HardwareSerial TELEM_Serial; // SIM800L Serial channel, max AutoBaud 115200, max baud rate (set manually) 460800.
        long TELEM_baudrate;

        TELEM_packet packet;

    public :

        TELEM_obj(HardwareSerial HWSerial = Serial3, long Baud = 115200, unsigned long freq = 433E6, int8_t power = 20, uint8_t spreadfactor = 9, unsigned long bandwidth = 250E3, uint8_t codingRate = 8); 

        bool setFreq();
        bool setPwr();
        bool setSF();
        bool setBW();
        bool setCR();

        unsigned long getFreq();
        int8_t getPwr();
        uint8_t getSF();
        unsigned long getBW();
        uint8_t getCR();

        bool init(); //initializes the module (sends parameters to remote microprocessor), returns false if not able
        bool goLive(); //wakes the TELEM module
        bool goIdle(); //sends the TELEM module to sleep

        bool uploadPacket(); //Sends the packet structure to the remote microprocessor

        //??? receivePacket(); //may be usefull to remote in the mainboard
};