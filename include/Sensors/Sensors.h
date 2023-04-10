#pragma once

#include <Arduino.h>

//Class containing sensors of type S, allows for the polling of all sensors (measurands of type T) and then processing of this data
template <class S, class T> class Sensors {
  private:
    S* _sensors; //array of sensors of type S
    uint8_t _th_amount; //theoretical number of sensors
    uint8_t _real_amount; //real number of sensors that are returning data
    uint8_t* _var_amount; //address to the number of measurands that are supposed to be extracted from this type of sensor
    uint32_t _flags; //flags indicating which sensors are working or not (32 max of each type)
    
    T** _data; //all of the data, each line for one sensor, each column for one measurand
    T* _pdata;//processed data array of measurands across all working sensors

    void allocateDataMemory();
    void freeDataMemory();
    void reallocateDataMemory();
     
  public:

    //s is an array of pointers of all sensors of type S (of which there are thAmount)
    Sensors(S* s, uint8_t th_amount) : _sensors(s), _th_amount(th_amount);

    // Sensors<S,T> &operator=(Sensors<S,T> Sensors_a, Sensors<S,T> Sensors_b);

    uint32_t initAll();

    uint8_t getThAmount();
    S* getSensors();
    T** get_data();
    T* get_pdata();
    uint32_t get_failure_flags();
    uint8_t get_real_amount();

    //method to poll data from all sensors (stored in _data) and process it (stored in _pdata)
    //data is multiplied by "factor" before being stored in _pdata
    //returns _pdata
    T* poll_process_ave_data(int factor);
};