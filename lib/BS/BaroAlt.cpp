#include <BS/BaroAlt.h>

BS_obj::BS_obj(SensingSystem* SensorSys)
:BS_components(SensorSys)
{
    refresh_BARO = BS_components->getBAROs_Hz();
    time_BARO = (unsigned long) pow(10,6)/refresh_BARO;
    delta_t = (double) 1/refresh_BARO;

    BARO_failure = false;
}

unsigned long BS_obj::initBaroAlt()
{
    //get P_init, T_init and alt_init from sensors
    if(debug::info()) 
    {
        debug::Serial.println("!! Barometric System Init !!\n\n");
        debug::Serial.println(" ->Instructions :");
        debug::Serial.println("     -->Place the Avionics bay in the shade outside (try to ventilate the BARO sensors), wait " + String(BS_const::delayBeforeInitMeas/60000) + "min...");
    }
    delay(BS_const::delayBeforeInitMeas);

    //Call for GPS measurements through the convential manner to get the real amount of GPS working
    BS_components->getGPSs_meas(); 
    uint8_t GPSs_real_amount = BS_components->getGPSs_Avio_rl_amount();
    //Then call for LLH GPS measurements (which gives height MSL)
    long* GPS_LLH = (BS_components->getGPSs_Avio()->getSensors())[0].getSensor()->getPOSLLH();
    
    float* BAROpdata = BS_components->getBAROs_meas(); //initial Pressure and Temperature
    uint8_t BAROs_rl_amount = BS_components->getBAROs_Avio_rl_amount();

    if(BAROs_rl_amount>0 && GPSs_real_amount>0)
    {
        P = BAROpdata[0];
        T = BAROpdata[1];
        altitude = (double) GPS_LLH[4]/1000.0f; 

        delete[] BAROpdata;
        delete[] GPS_LLH;

        if(debug::info()) 
        {
            debug::Serial.println("   ->Recovered Initial :");
            debug::Serial.println("     -->Altitude : " + String(altitude)) + "m";
            debug::Serial.println("     -->Pressure : " + String(P)) + "Pa";
            debug::Serial.println("     -->Temperature : " + String(T)) + "K";
        }

        return (unsigned long) pow(10,6)*time_BARO;
    }
    else
    {
        return 0;
    }
}

double BS_obj::getAltitude()
{
    float* BAROpdata = BS_components->getBAROs_meas();
    uint8_t BAROs_rl_amount = BS_components->getBAROs_Avio_rl_amount();
    
    if(BAROs_rl_amount > 0)
    {
        BARO_failure = false;

        //Current Temperature Based Altitude Step Integral Computation (CTBASIC)
        //Should work well for small steps in altitude (10m ish)
        
        //Compute new altitude based on the old (P,T) and the new Pressure and Temperature stored in BAROpdata
        altitude = altitude - BS_const::r*BAROpdata[2]*log(BAROpdata[1]/P)/BS_const::g;

        //Update the current (P,T)
        P = BAROpdata[1];
        T = BAROpdata[2];

        delete[] BAROpdata;
    }
    else
    {
        altitude = faultCodes::altitude;
        BARO_failure = true;
    }

    return altitude;
}