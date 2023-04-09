#include <Sensors/Base_Sensor.h>

template <class T>
Base_Sensor<T>::Base_Sensor(uint8_t varAmount):_varAmount(varAmount)
{
    allocateMemory();
}

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