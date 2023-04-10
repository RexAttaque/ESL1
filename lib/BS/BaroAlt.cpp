#include <BS/BaroAlt.h>

BS_obj::BS_obj(SensingSystem* SensorSys, float BARO_refreshRate):BS_components(SensorSys),time_BARO((double) 1/BARO_refreshRate)
{}

unsigned long BS_obj::initBaroAlt()
{
    //get P_init, T_init and alt_init from sensors/user

    if(true)
    {
        return (unsigned long) pow(10,6)*time_BARO;
    }
    else
    {
        return 0;
    }
}

long BS_obj::getAltitude()
{
    float* BAROpdata = BS_components->getBAROs_meas(1);
    uint8_t BAROs_rl_amount = BS_components->getBAROs_Avio_rl_amount();
    
    if(BAROs_rl_amount > 0)
    {
        BARO_failure = false;

        //ISA atmosphere altitude computation
        float gam = 1.4;
        float g = 9.81; // m/s^2
        float r = 287.03; // J/kg/K
    }
    else
    {
        BARO_failure = true;
    }

    return 0.0f;
}