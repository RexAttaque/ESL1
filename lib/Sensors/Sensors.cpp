#include <Sensors/Sensors.h>

//Class containing sensors of type S, allows for the polling of all sensors (measurands of type T) and then processing of this data
template <class S, class T> class Sensors {
  private:
    S* _sensors; //array of sensors of type S
    uint8_t _th_amount = 0; //theoretical number of sensors
    uint8_t _real_amount = 0; //real number of sensors that are returning data
    uint8_t* _var_amount; //adress to the number of measurands that are supposed to be extracted from this type of sensor
    uint32_t _flags = 0; //flags indicating which sensors are working or not (32 max of each type)
    
    T** _data = nullptr; //all of the data, each line for one sensor, each column for one measurand
    T* _pdata = nullptr;//processed data array of measurands across all working sensors

    void allocateDataMemory() {
      this._data = new (T*)(this._th_amount);
      for(uint8_t i=0; i<this._th_amount; i++){
        this._data[i] = new (T)(this._sensors[0]->get_varAmount());
      }

      this._pdata = new (T)(this._var_amount);
    }

    void freeDataMemory() {
      for(uint8_t i=0; i<this._th_amount; i++){
        delete[] this._data[i];
      }
      
      delete[] this._data;
      delete[] this._pdata;
    }

    void reallocateDataMemory() {
        this.freeMemory();
        this.allocateMemory();
    }
     
  public:

    //s is an array of pointers of all sensors of type S (of which there are thAmount)
    Sensors(S* s, uint8_t th_amount) : _sensors(s), _th_amount(th_amount) {
      this._var_amount = &(s[0]->get_var_amount());
      this.allocateDataMemory();
    };

    // Sensors<S,T> &operator=(Sensors<S,T> Sensors_a, Sensors<S,T> Sensors_b)
    // {
    //   Sensors_a._sensors = Sensors_b._sensors;
    //   Sensors_a._th_amount = Sensors_b._th_amount;
    //   Sensors_a._var_amount = Sensors.b._var_amount;
    // }

    T** get_data()
    {
      return _data;
    }

    T* get_pdata()
    {
      return _pdata;
    }

    uint32_t get_failure_flags()
    {
      return _flags;
    }

    uint8_t get_real_amount()
    {
      return _real_amount;
    }

    //method to poll data from all sensors (stored in _data) and process it (stored in _pdata) using an average across working sensors
    //data is multiplied by "factor" before being stored in _pdata
    //returns _pdata
    T* poll_process_ave_data(int factor){

      //reset indicators and the amount of sensors currently in operation
      this._flags = 0;
      this._real_amount = 0;
      
      for(uint8_t i=0; i<this._th_amount; i++)
      {
        this.reallocateDataMemory();
        this._data[i] = this._sensors[i]->getMeas();

        //if data was correctly recovered
        if(this._data[i] != nullptr)
        {
          //set flag corresponding to the sensor to 1
          this._flags += pow(2,i);
          //add one sensor to the set of currently operating sensors
          this._real_amount += 1;

          //processing algo (resulting in a single measurand array, here a simple average) (part1)
          for(uint8_t j=0; j<this._sensors[0]->get_varAmount(); j++)
          {
            *this._pdata[j] += *this._data[i][j];
          }
        }
      }
      
      //processing algo (resulting in a single measurand array, here a simple average) (part2)
      for(uint8_t j=0; j<this._sensors[0]->varAmount; j++)
      {
        *this._pdata[j] *= factor/this._real_amount;
      }

      return this._pdata;
    }
};