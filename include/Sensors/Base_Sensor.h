#pragma once

#include <Arduino.h>

template <class T>
class Base_Sensor {

    protected :
        T* meas;
        uint8_t _varAmount;

        void allocateMemory();
        void freeMeasMemory();
        void reallocateMemory();

    public :
        Base_Sensor(uint8_t varAmount);

        uint8_t get_var_amount();
};