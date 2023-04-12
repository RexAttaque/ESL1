#pragma once

#include <Arduino.h>
#include <fault_debug.h>

template <class T>
class Base_Sensor {

    protected :
        T* meas;
        uint8_t _varAmount;
        uint16_t _refreshHz;

        void allocateMemory();
        void freeMeasMemory();
        void reallocateMemory();

    public :
        Base_Sensor(uint8_t varAmount, uint16_t refreshHz);

        virtual bool init();
        virtual bool calibrate();
        virtual bool reset();

        virtual bool goLive();
        virtual bool goIdle();

        virtual bool getStatus();
        virtual uint8_t getCalibration();
        virtual T* getMeas();
        void deleteMeas();

        uint16_t get_refresh_rate();
        uint8_t get_var_amount();
};