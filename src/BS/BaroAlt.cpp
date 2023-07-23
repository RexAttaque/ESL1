#include <BS/BaroAlt.h>

BS_obj::BS_obj(SensingSystem* SensorSys)
:fault_debug(BS_const::debug_ID, BS_const::debug_lvl),BS_components(SensorSys)
{
    refresh_BARO = BS_components->getBAROs_Hz();
    time_BARO = (unsigned long) pow(10,6)/refresh_BARO;
    delta_t = (double) 1/refresh_BARO;

    BARO_failure = false;
}

unsigned long BS_obj::initBaroAlt()
{
    unsigned long result=0;

    //get P_init, T_init and alt_init from sensors
    println(debugLevel::INFO,"!! Barometric System Init Start !!\n\n\r\n ->Instructions : \r\n     -->Place the Avionics bay in the shade outside (try to ventilate the BARO sensors), wait " + String(BS_const::delayBeforeInitMeas/60000) + "min... \r\n", "initBaroAlt()");
    delay(BS_const::delayBeforeInitMeas);

    bool initializerGPS_status = (BS_components->getGPSs_Avio()->getSensors())[0].getStatus(); //check if the initializer GPS is working
    float* BAROpdata = BS_components->getBAROs_meas(); //initial Pressure and Temperature

    if(BAROpdata != nullptr && initializerGPS_status)
    {
        //Then call for LLH GPS measurements (which gives height MSL)
        long* GPS_LLH = (BS_components->getGPSs_Avio()->getSensors())[0].getSensor()->getPOSLLH();

        if(GPS_LLH != nullptr)
        {
            P = BAROpdata[0];
            T = BAROpdata[1];
            altitude = (double) GPS_LLH[4]/1000.0f; 

            delete[] BAROpdata;
            delete[] GPS_LLH;

            println(debugLevel::INFO,"   ->Recovered Initial : \r\n     -->Altitude : " + String(altitude) + "m \r\n     -->Pressure : " + String(P) + "Pa \r\n     -->Temperature : " + String(T) + "K");

            result = (unsigned long) pow(10,6)*time_BARO;
        }

        delete[] GPS_LLH;
    }
    
    println(debugLevel::INFO,"\n\n!! Barometric System Init End (result = " + String(result) + ") !!");
    return 0;
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