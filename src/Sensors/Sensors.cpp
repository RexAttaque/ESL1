#include <Sensors/Sensors.h>

//s is an array of pointers of all sensors of type S (of which there are thAmount)
template <class S, class T>
Sensors<S, T>::Sensors(S* s, uint8_t th_amount) : _sensors(s), _th_amount(th_amount) {
  _var_amount = s[0].get_var_amount();
  allocateDataMemory();
}

template <class S, class T>
void Sensors<S, T>::allocateDataMemory() {
  _pdata = new (T)(_var_amount);

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
    debug_sens.println(debugLevel::INFO, "\n    -->Sensor" + i, "initAll()");

    if(_sensors[i].init()) 
    {
      debug_sens.println(debugLevel::INFO, "    -->INIT PASS\n");
    
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
    debug_sens.println(debugLevel::INFO, "\n    -->Sensor" + i, "wakeAll()");

    if(_sensors[i].goLive()) 
    {
      debug_sens.println(debugLevel::INFO, "    -->IS LIVE\n");
    
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
    debug_sens.println(debugLevel::INFO, "\n    -->Sensor" + i, "sleepAll()");

    if(_sensors[i].goIdle()) 
    {
      debug_sens.println(debugLevel::INFO, "    -->IS IDLE\n");
    
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
    debug_sens.println(debugLevel::INFO, "\n    -->Sensor" + i, "calibrateAll()");

    if(_sensors[i].calibrate()) 
    {
      debug_sens.println(debugLevel::INFO, "    -->IS CALIBRATED\n");
    
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
        for(uint8_t j=0; j<_sensors[0].get_varAmount(); j++)
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