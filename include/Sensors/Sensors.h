#pragma once

#include <Arduino.h>
#include <fault_debug.h>

namespace sens_const {
  const String debug_ID = "SENS";
  const uint8_t debug_lvl = debugLevel::FULL;
};

//Class containing sensors of type S, allows for the polling of all sensors (measurands of type T) and then processing of this data
//WARNING : All sensors of the same type introduced in this class should have the same refresh rate
template <class S, class T> class Sensors : public fault_debug {
  private:

    S* _sensors; //array of sensors of type S
    uint8_t _th_amount; //theoretical number of sensors
    uint8_t _real_amount; //real number of sensors that are returning data
    uint8_t* _var_amount; //address to the number of measurands that are supposed to be extracted from this type of sensor
    uint32_t _flags; //flags indicating which sensors are working or not (32 max for each Sensors class)
    
    T** _data; //all of the data, each line for one sensor, each column for one measurand
    T* _pdata;//processed data array of measurands across all working sensors

    void allocateDataMemory();
    void freeDataMemory();
    void reallocateDataMemory();
     
  public:

    //s is an array of pointers of all sensors of type S (of which there are thAmount)
    Sensors(S* s, uint8_t th_amount);

    // Sensors<S,T> &operator=(Sensors<S,T> Sensors_a, Sensors<S,T> Sensors_b);

    bool initAll();
    bool wakeAll();
    bool sleepAll();
    bool calibrateAll();

    uint16_t getRefreshRate();
    uint8_t getThAmount();
    S* getSensors();
    T** get_data();
    T* get_pdata();
    uint32_t get_failure_flags();
    uint8_t get_real_amount();

    //method to poll data from all sensors (stored in _data) and process it (stored in _pdata)
    //returns _pdata
    T* poll_process_ave_data();
};

//s is an array of pointers of all sensors of type S (of which there are thAmount)
template <class S, class T>
Sensors<S, T>::Sensors(S* s, uint8_t th_amount) : fault_debug(sens_const::debug_ID, sens_const::debug_lvl), _sensors(s), _th_amount(th_amount) {
  uint8_t _var_amount_temp = s[0].get_var_amount();
  _var_amount = &_var_amount_temp;
  allocateDataMemory();
}

template <class S, class T>
void Sensors<S, T>::allocateDataMemory() {
  _pdata = new T[*_var_amount];

  for(uint8_t j=0; j<*_var_amount; j++)
  {
    _pdata[j] = 0;
  }
}

template <class S, class T>
void Sensors<S, T>::freeDataMemory() {
  delete[] _pdata;
  _pdata = nullptr;
}

template <class S, class T>
void Sensors<S, T>::reallocateDataMemory() {
  freeDataMemory();
  allocateDataMemory();
}

//template <class S, class T>
// Sensors<S,T> Sensors<S, T>::&operator=(Sensors<S,T> Sensors_a, Sensors<S,T> Sensors_b)
// {
//   Sensors_a._sensors = Sensors_b._sensors;
//   Sensors_a._th_amount = Sensors_b._th_amount;
//   Sensors_a._var_amount = Sensors.b._var_amount;
// }

template <class S, class T>
bool Sensors<S, T>::initAll()
{
  bool result = false;

  for(uint8_t i=0; i<_th_amount; i++)
  {
    println(debugLevel::INFO, "\n    -->Sensor" + i, "initAll()");

    if(_sensors[i].init()) 
    {
      println(debugLevel::INFO, "    -->INIT PASS\n");
    
      result = result && true;
    }
    else
    {
      result = false;
    }
  }

  return result;
}

template <class S, class T>
bool Sensors<S, T>::wakeAll()
{
  bool result = false;

  for(uint8_t i=0; i<_th_amount; i++)
  {
    println(debugLevel::INFO, "\n    -->Sensor" + i, "wakeAll()");

    if(_sensors[i].goLive()) 
    {
      println(debugLevel::INFO, "    -->IS LIVE\n");
    
      result = result && true;
    }
    else
    {
      result = false;
    }
  }

  return result;
}

template <class S, class T>
bool Sensors<S, T>::sleepAll()
{
  bool result = false;

  for(uint8_t i=0; i<_th_amount; i++)
  {
    println(debugLevel::INFO, "\n    -->Sensor" + i, "sleepAll()");

    if(_sensors[i].goIdle()) 
    {
      println(debugLevel::INFO, "    -->IS IDLE\n");
    
      result = result && true;
    }
    else
    {
      result = false;
    }
  }

  return result;
}

template <class S, class T>
bool Sensors<S, T>::calibrateAll()
{
  bool result = false;

  for(uint8_t i=0; i<_th_amount; i++)
  {
    println(debugLevel::INFO, "\n    -->Sensor" + i, "calibrateAll()");

    if(_sensors[i].calibrate()) 
    {
      println(debugLevel::INFO, "    -->IS CALIBRATED\n");
    
      result = result && true;
    }
    else
    {
      result = false;
    }
  }

  return result;
}

template <class S, class T>
uint16_t Sensors<S, T>::getRefreshRate()
{
  return _sensors[0].get_refresh_rate();
}

template <class S, class T>
uint8_t Sensors<S, T>::getThAmount()
{
  return _th_amount;
}

template <class S, class T>
S *Sensors<S, T>::getSensors()
{
  return _sensors;
}

template <class S, class T>
T **Sensors<S, T>::get_data()
{
  return _data;
}

template <class S, class T>
T* Sensors<S, T>::get_pdata()
{
  return _pdata;
}

template <class S, class T>
uint32_t Sensors<S, T>::get_failure_flags()
{
  return _flags;
}

template <class S, class T>
uint8_t Sensors<S, T>::get_real_amount()
{
  return _real_amount;
}

//method to poll data from all sensors (stored in _data) and process it (stored in _pdata) using an average across working sensors
//data is multiplied by "factor" before being stored in _pdata
//returns _pdata
template <class S, class T>
T* Sensors<S, T>::poll_process_ave_data(){

  //reset indicators and the amount of sensors currently in operation
  _flags = 0;
  _real_amount = 0;

  reallocateDataMemory();
  
  for(uint8_t i=0; i<_th_amount; i++)
  { 
    //if sensor is responsive
    if(_sensors[i].getStatus()) 
    {
      _data[i] = _sensors[i].getMeas();

      //if data was correctly recovered
      if(_data[i] != nullptr)
      {
        //set flag corresponding to the sensor to 1
        _flags += pow(2,i);
        //add one sensor to the set of currently operating sensors
        _real_amount += 1;

        //processing algo (resulting in a single measurand array, here a simple average) (part1)
        for(uint8_t j=0; j<_sensors[0].get_var_amount(); j++)
        {
          _pdata[j] += _data[i][j];
        }
      }
    }
    else
    {
      _sensors[i].deleteMeas();
    }
  }
  
  if(_real_amount>0)
  {
    //processing algo (resulting in a single measurand array, here a simple average) (part2)
    for(uint8_t j=0; j<*_var_amount; j++)
    {
      _pdata[j] /= _real_amount;
    }
  }
  else
  {
    freeDataMemory();
  }

  return _pdata;
}