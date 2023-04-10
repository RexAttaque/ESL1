#include <Sensors/Base_Sensor.h>

template <class T>
Base_Sensor<T>::Base_Sensor(uint8_t varAmount, uint16_t refreshHz)
:_varAmount(varAmount),_refreshHz(refreshHz)
{
    allocateMemory();
}

//virtual methods that should be common to all sensors but vary from one to the other
template <class T>
bool Base_Sensor<T>::init()
{
    return false;
}

template <class T>
uint8_t Base_Sensor<T>::getStatus()
{
    return 0;
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


//Actual common methods
template <class T>
uint8_t Base_Sensor<T>::get_var_amount()
{
    return _varAmount;
}

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