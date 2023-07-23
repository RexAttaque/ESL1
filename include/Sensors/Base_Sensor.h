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

template <class T>
Base_Sensor<T>::Base_Sensor(uint8_t varAmount, uint16_t refreshHz)
:_varAmount(varAmount),_refreshHz(refreshHz)
{
    allocateMemory();
}

//private
template <class T>
void Base_Sensor<T>::allocateMemory()
{
    meas = new (T)(_varAmount);
}

template <class T>
void Base_Sensor<T>::freeMeasMemory()
{
    delete[] meas;
}

template <class T>
void Base_Sensor<T>::reallocateMemory()
{
    freeMeasMemory();
    allocateMemory();
}


//virtual methods that should be common to all sensors but vary from one to the other
template <class T>
bool Base_Sensor<T>::init()
{
    return false;
}

template <class T>
bool Base_Sensor<T>::calibrate()
{
    return false;
}

template <class T>
bool Base_Sensor<T>::reset()
{
    return false;
}

template <class T>
bool Base_Sensor<T>::goLive()
{
    return false;
}

template <class T>
bool Base_Sensor<T>::goIdle()
{
    return false;
}

template <class T>
bool Base_Sensor<T>::getStatus()
{
    return false;
}

template <class T>
uint8_t Base_Sensor<T>::getCalibration()
{
    return 0;
}

template <class T>
T* Base_Sensor<T>::getMeas()
{
    return nullptr;
}


//Actual common methods
template <class T>
void Base_Sensor<T>::deleteMeas()
{
    freeMeasMemory();
    meas = nullptr;
}

template <class T>
uint16_t Base_Sensor<T>::get_refresh_rate()
{
    return _refreshHz;
}

template <class T>
uint8_t Base_Sensor<T>::get_var_amount()
{
    return _varAmount;
}