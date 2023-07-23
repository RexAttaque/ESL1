#pragma once

#include <Arduino.h>
#include <fault_debug.h>

namespace GSM_const {
    const String debug_ID = "GSM";
    const uint8_t debug_lvl = debugLevel::FULL;

    const HardwareSerial def_Serial_GSM = Serial2;
    const unsigned long def_GSM_baud = 115200;
    const String def_Number = "+33640697567";

    const uint8_t maxRegAttempts = 10; //maximum number of allowed registration attempts before cancelling
    const unsigned long timeBetweenRegCheck = 5000; //millis of delay between each check of registration
    const uint8_t signalQuality_floor = 10; // goes from 0 to 31 with 31 being excellent
    
    const uint8_t sendStages = 4;
}

class GSM_obj : public fault_debug {
    private :
        HardwareSerial GSM_Serial; // SIM800L Serial channel, max AutoBaud 115200, max baud rate (set manually) 460800.
        long GSM_baudrate;

        String phoneNumber; //phone number of the inserted SIM
        String RX = ""; //last transmission from the module
        String TX = ""; //last message sent to the module
        //String PIN = "0000"; //remove the PIN from the inserted SIM card before use

    public :

        GSM_obj(HardwareSerial HWSerial = GSM_const::def_Serial_GSM, String GSM_num = GSM_const::def_Number, long Baud = GSM_const::def_GSM_baud); 

        String getLastTX();
        
        void setPhoneNumber(String GSM_num);
        void setTX(String message);
        //void setPIN();

        bool check_SIM_GSM(); //Checks wether a SIM is present or not
        bool check_REG_GSM(); //Checks wether or not the module has registered on a network
        bool check_SIG_GSM(int qualityFloor); //Checks the reported signal quality, returns false if it is below the floor
        
        void read_RX(); //reads the incoming module transmission and stores it in RX
        bool check_RX_OK(); //Checks that the last two characters of RX are OK
        bool check_AT_OK_GSM(char c); //Sends then checks that the module has received the character c
        bool check_AT_OK_GSM(String AT_COMMAND); //Sends then checks that the module has received the command AT_COMMAND
        void TRX_AT_GSM(char c); //Sends the charater c to the module and listens for the answer that is stored in RX
        void TRX_AT_GSM(String AT_COMMAND); //Sends the command AT_COMMAND to the module and listens for the answer that is stored in RX

        bool init(); //initializes the module (checks SIM, REG and SIG), returns false if no SIM, no REG or low SIG quality
        bool goLive(); //wakes the GSM module
        bool goIdle(); //sends the GSM module to sleep

        bool PrepSend_s1(); //Sending an SMS is cut into multiple stages because the module is too slow to execute everything at once
        bool PrepSend_s2();
        bool PrepSend_s3();
        bool sendSMS(); //Sends the TX string via SMS
};