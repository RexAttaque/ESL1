#pragma once

#include <Arduino.h>
#include <faultCodes.h>

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
        virtual uint8_t getStatus();
        virtual uint8_t getCalibration();
        virtual T* getMeas();

        virtual bool goLive();
        virtual bool goIdle();

        uint16_t get_refresh_rate();
        uint8_t get_var_amount();
};