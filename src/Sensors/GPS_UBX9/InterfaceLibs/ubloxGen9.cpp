#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

//////////////////////////////////////////////////////// Init ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ublox_gen9::setBasicConfig()
{   
  bool refresh = setRefreshRate(1000);
  
  if(refresh)
  {
    delay(1000);
    bool rate = setNavRate();
    bool dynMod = setPltfrmModel();
    bool UART1gen = setUART1gen_config();
    bool UART1prot = setUART1prot_config();
    delay(1000);
    //bool UART1msg = setUART1msg_config(); //UART messages are all already disabled for UBX
    delay(1000);
    bool UART2gen = setUART2gen_config();
    bool SPIgen = setSPIgen_config();
    //bool I2Cgen = setI2Cgen_config(); //already disabled via SEL pin, crashes if you try to set it
    //bool USBprot = setUSBprot_config(); //NMEA on USB is fairly useful
    delay(1000);
    //bool USBmsg = setUSBmsg_config(); //already all deactivated
    delay(1000);
    refresh = setRefreshRate(GPS_TimeToRefresh);
    delay(1000);

    return refresh && rate && dynMod && UART1gen && UART1prot && UART2gen && SPIgen;
    //return refresh && rate && dynMod && UART1gen && UART1prot && UART1msg && UART2gen && SPIgen && I2Cgen && USBprot && USBmsg;
  }

  return false;
}

/////////////////////////////////////////////// Support Core ////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ublox_gen9::calculateCheckSum(uint8_t* frame, uint16_t frameSize, bool receiveMode)
{ 
  uint8_t checksum[] = {0, 0};

  for(int i=2; i<frameSize-2; i++)
  {
    checksum[0] = *(frame+i) + checksum[0];
    checksum[1] = checksum[1] + checksum[0];

    checksum[0] = checksum[0] & 0xFF;
    checksum[1] = checksum[1] & 0xFF;
  }

  if(receiveMode == false)
  {
      println(debugLevel::FULL,"TX\n","calculateCheckSum()");

    
    *(frame+frameSize-2) = checksum[0];
    *(frame+frameSize-1) = checksum[1];

    return true;
  }
  else
  {
    println(debugLevel::FULL,"RX","calculateCheckSum()");
    
    if( *(frame+frameSize-2) == checksum[0] && *(frame+frameSize-1) == checksum[1] )
    {
      println(debugLevel::FULL,"Pass\n");

      return true;
    }
    else
    {
      println(debugLevel::FULL,"Fail\n");

      return false;
    }
  }
}

//splits a 64 bit uint to an array of 8 bit uints
//value and value length in bytes
uint8_t* ublox_gen9::splitTo8bit(uint64_t value, uint16_t val_length)
{
  println(debugLevel::FULL,"Args : " + String(value) + " , " + String(val_length), "splitTo8bit(value,length)");
  
  if(val_length <= 0)
  {
    return nullptr;
  }
  
  uint8_t* buff = new uint8_t[val_length];
  uint64_t temp;

  println(debugLevel::FULL,"Split number value : ");
  for(long i=val_length-1; i>=0; i--)
  {
    temp = value>>(val_length-i-1)*8;

    print(debugLevel::FULL,String(temp) + "/", "", true);

    *(buff+i) = temp & 0xFF;

    print(debugLevel::FULL,String(*(buff+i), HEX) + " ", "", true);
  }

  skipln(debugLevel::FULL, 2);

  return buff;
}

/////////////////////////////////////////////// Core UART ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//extracted length in bytes counting start bytes etc. (full frame length)
uint8_t* ublox_gen9::receiveUBXframe(uint16_t extractedLength)
{
  println(debugLevel::TRACE,"Args : " + String(extractedLength),"receiveUBXframe(extractedlength)");

  if(extractedLength <= 0) 
  {
    return nullptr;
  }
  
  uint8_t* buff = new uint8_t[extractedLength];

  for(int i=0; i<extractedLength; i++)
  {
    *(buff+i) = 0x00;
  }

  long ReceivedFrameSum = 0;
  uint8_t ReceivedFrameSize = 0;
  
  while(GPS_Serial.available() && ReceivedFrameSum == 0 && ReceivedFrameSize != extractedLength)
  {
    print(debugLevel::TRACE,"Attempted RX... ");
    ReceivedFrameSize = GPS_Serial.readBytes(buff, extractedLength);

    for(int i=0; i<ReceivedFrameSize; i++)
    {
      ReceivedFrameSum += *(buff+i);
    }
  }
  
  if(isLogged(debugLevel::FULL))
  {
    println(debugLevel::FULL,"Received following message : ");
    for(long i=0; i<extractedLength; i++)
    {
      print(debugLevel::FULL,String(*(buff+i), HEX) + " ", "", true);
    }
    skipln(debugLevel::FULL);
  }


  //flush whatever is left in the GPS_Serial buffer to prevent interactions
  while(GPS_Serial.available())
  {
    GPS_Serial.read();
  }

  if(calculateCheckSum(buff, extractedLength, true))
  {
    println(debugLevel::TRACE,"Checksum Pass on Receive\n");

    return buff;
  }
  else
  {
    println(debugLevel::TRACE,"Checksum Fail on Receive\n");

    delete[] buff;
    return nullptr;
  }
}

bool ublox_gen9::sendUBXframe(uint8_t cmdClass, uint8_t messageID, uint16_t payloadSize, uint8_t* payload, bool configMode)
{
  println(debugLevel::TRACE,"Args : " + String(cmdClass) + " , " + String(messageID) + " , " + String(payloadSize) + " , " + String(*payload) + " , " + String(configMode), "sendUBXframe(cmdClass, msgID, payloadSize, payload, cfgMode)");

  if(cmdClass <= 0 || messageID <= 0)
  {
    return false;
  }

  uint8_t* splittedSize;
  
  if(payloadSize > 0)
  {
    splittedSize = splitTo8bit(payloadSize, 2);
  }
  else
  {
    splittedSize = new uint8_t[2];
    *splittedSize = 0;
    *(splittedSize+1) = 0;
  }
  
  uint16_t frameSize = 8 + payloadSize; //2 for start, 2 for class/ID, 2 for size, 2 for checksum and payloadSize for the payload
  uint8_t* buff = new uint8_t[frameSize]; 
  *buff = 0xb5;
  *(buff+1) = 0x62; //start characters
  *(buff+2) = cmdClass; //UBX setValue class
  *(buff+3) = messageID; //UBX setValue message ID
  *(buff+4) = *(splittedSize+1); //size of the payload LSB
  *(buff+5) = *splittedSize; //size of the payload MSB

  for(int i=0; i<payloadSize; i++)
  {
    *(buff+i+6) = *(payload+i);
  }

  calculateCheckSum(buff, frameSize, false);

  if(isLogged(debugLevel::FULL)) 
  {
    print(debugLevel::FULL,"UBX frame : ");
    for (int i=0; i<frameSize; i++)
    {
      print(debugLevel::FULL, String(*(buff+i), HEX) + " ", "", true);
    }
    skipln(debugLevel::FULL);
  }


  size_t SentSize = 0;
  
  while(SentSize != frameSize)
  {
    SentSize = GPS_Serial.write(buff, frameSize);
    println(debugLevel::FULL, SentSize);
  }

  bool outcome = false;

  if(configMode == true)
  { 
    delay(100);
    uint8_t* ack_msg = receiveUBXframe(10);
  
    if(ack_msg != nullptr && *(ack_msg+2) == 0x05 && *(ack_msg+3) == 0x01 && *(ack_msg+6) == cmdClass && *(ack_msg+7) == messageID)
    {
      outcome = true; 
    }

    delete[] ack_msg;
  }
  else
  {
    outcome = true;
  }

  delete[] buff;
  delete[] splittedSize;

  println(debugLevel::TRACE, "Outcome : " + String(outcome) + '\r\n');

  return outcome;
}


/////////////////////////////////////////////// Core GPS Config //////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//level 0 -> RAM
//level 2 -> Battery backup RAM (BBR)
//level 4 -> Flash
//level 7 -> All
//One or all of the above can be edited at once (All == 7)
bool ublox_gen9::setConfig(uint8_t level, long keyID, uint64_t value, uint8_t valueSize)
{ 
  println(debugLevel::TRACE,"Args : " + String(level, DEC) + " , " + String(keyID, HEX) + " , " + String(value) + " , " + String(valueSize), "setConfig(level, keyID, value, valueSize)");

  if(level < 0 || level > 7)
  {
    level = 0;
  }

  uint8_t* splittedKey = splitTo8bit(keyID, 4);
  uint8_t* splittedValue = splitTo8bit(value, valueSize);

  uint16_t payloadSize = 8+valueSize;
  uint8_t* payload = new uint8_t[payloadSize];

  *payload = 0x00;
  *(payload+1) = level;
  *(payload+2) = 0x00;
  *(payload+3) = 0x00;

  for(int i=0; i<4; i++)
  {
    *(payload-i+7) = *(splittedKey+i);
  }
  
  for(int i=0; i<valueSize; i++)
  {
    *(payload-i+7+valueSize) = *(splittedValue+i);
  }

  if(isLogged(debugLevel::FULL))
  {
    print(debugLevel::FULL,"Split Payload : ");
    for (int i=0; i<payloadSize; i++)
    {
      print(debugLevel::FULL,String(*(payload+i), HEX) + " ", "", true);
    }
    skipln(debugLevel::FULL);
  }

  bool success = false;
  uint8_t i = 0;
  
  while(success == false && i<3) 
  {
    success = sendUBXframe(0x06, 0x8a, payloadSize, payload, true);
    i++;
  }

  //check wether the config was indeed applied
  if(success)
  {
    uint8_t* splittedGetValue = getConfig(keyID, 0, valueSize);

    if(splittedGetValue != nullptr)
    {
      for(int i=0; i<valueSize && success; i++)
      {
        print(debugLevel::FULL,String(*(splittedGetValue+i), HEX) + "/" + String(*(splittedValue-i-1+valueSize), HEX) + " - ", "", true);

        success = *(splittedGetValue+i)==*(splittedValue+valueSize-i-1);
      }
      skipln(debugLevel::FULL);
    }

    delete[] splittedGetValue;
  }
  
  delete[] payload;
  delete[] splittedValue;
  delete[] splittedKey;

  if(isLogged(debugLevel::TRACE) && success) println(debugLevel::TRACE, "Config successfully applied and checked\n");

  return success;
}


// 0 - RAM
// 1 - BBR
// 2 - Flash
// 7 - Default value
uint8_t* ublox_gen9::getConfig(long keyID, uint8_t level, uint8_t expectedValueSize)
{
  println(debugLevel::TRACE,"Args : " + String(keyID, HEX) + " , " + String(level, DEC) + " , " + String(expectedValueSize), "getConfig(keyID, level, expectedValueSize)");
  
  if(level < 0 || level > 7)
  {
    level = 7;
  }

  uint8_t* splittedKey = splitTo8bit(keyID, 4);

  uint16_t payloadSize = 8;
  uint8_t* payload = new uint8_t[payloadSize];

  *payload = 0x00;
  *(payload+1) = level;
  *(payload+2) = 0x00;
  *(payload+3) = 0x00;

  for(int i=0; i<4; i++)
  {
    *(payload-i+7) = *(splittedKey+i);
  }

  bool success = false;
  uint8_t i = 0;
  
  while(success == false && i<3) 
  {
    success = sendUBXframe(0x06, 0x8b, payloadSize, payload, false);
    i++;
  }
  
  delete[] payload;
  delete[] splittedKey;

  if(success == true)
  {
    delay(100);
    uint8_t* frame = receiveUBXframe(16+expectedValueSize);
    uint8_t* value = new uint8_t[expectedValueSize];

    if(frame != nullptr)
    {
      for(uint8_t i=0; i<expectedValueSize; i++)
      {
        *(value+i) = *(frame+i+14);
      }
    }
    else
    {
      delete[] value;
      value = nullptr;
    }

    delete[] frame;

    println(debugLevel::TRACE,"getConfig - Success\n");
    
    return value;
  }
  else
  {
    println(debugLevel::TRACE,"getConfig - Fail\n");
    
    return nullptr;
  }
}

/////////////////////////////////////////////// Core GPS Polling /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t* ublox_gen9::PollValue(uint8_t cmdClass, uint8_t messageID, uint16_t expectedPayloadSize, bool expectedInBuffer)
{
  println(debugLevel::TRACE,"Args : " + String(cmdClass) + " , " + String(messageID) + " , " + String(expectedPayloadSize) + " , " + String(expectedInBuffer), "PollValue(cmdClass, messageID, expectedPayloadSize, expectedInBuffer)");
  
  if(expectedPayloadSize > 0)
  {
    bool success = false;
    uint8_t i = 0;
    
    while(success == false && i<3 && expectedInBuffer == false) 
    {
      success = sendUBXframe(cmdClass, messageID, 0, 0, false);
      i++;
    }
  
    if(success == true || expectedInBuffer)
    {      
      uint8_t* frame = receiveUBXframe(8+expectedPayloadSize);
      uint8_t* payload = new uint8_t[expectedPayloadSize];
  
      if(frame != nullptr)
      {
        for(uint8_t i=0; i<expectedPayloadSize; i++)
        {
          *(payload+i) = *(frame+i+6);
        }

        if(isLogged(debugLevel::FULL))
        {
          print(debugLevel::FULL,"Got the following poll response : ");
          for(long i=0; i<expectedPayloadSize; i++)
          {
            print(debugLevel::FULL, String(*(payload+i), HEX) + " ", "", true);
          }
          skipln(debugLevel::FULL);
        }
      }
      else
      {
        println(debugLevel::TRACE,"PollValue - Fail - No RX\n");

        delete[] payload;
        payload = nullptr;
      }
  
      delete[] frame;
      
      return payload;
    }
    else
    {
      println(debugLevel::TRACE,"PollValue - Fail - No TX\n");

      return nullptr;
    }
  }
  else
  {
    return nullptr;
  }
}

/////////////////////////////////////////////// GPS Interface /////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ublox_gen9::setRefreshRate(uint16_t refreshRate)
{
  println(debugLevel::TRACE,"Args : " + String(refreshRate), "setRefreshRate(rate)");

  return setConfig(configLevel, CFG_RATE_MEAS, refreshRate, 2);
}

bool ublox_gen9::setNavRate()
{
  println(debugLevel::TRACE,"Args : ", "setNavRate()");
  
  return setConfig(configLevel, CFG_RATE_NAV, GPS_NavRate, 2);
}

bool ublox_gen9::setPltfrmModel()
{
  println(debugLevel::TRACE,"Args : ", "setPlatformModel()");

  return ublox_gen9::setConfig(configLevel, CFG_NAVSPG_DYNMODEL, GPS_PltfrmModel, 1);
}

bool ublox_gen9::setUART1gen_config()
{
  println(debugLevel::TRACE,"Args : ", "setUART1general_config()");

  return setConfig(configLevel, CFG_UART1_STOPBITS, GPS_StopBits, 1);
}

bool ublox_gen9::setUART1prot_config()
{
  println(debugLevel::TRACE,"Args : ", "setUART1protocol(I/O)_config()");

  uint8_t valueSize = 1;
  bool NMEA_statement = UART1_NMEA && !bufferedPOSECEF;
  
  return setConfig(configLevel, CFG_UART1INPROT_NMEA, NMEA_statement, valueSize) && setConfig(configLevel, CFG_UART1INPROT_RTCM3X, 0, valueSize) && setConfig(configLevel, CFG_UART1OUTPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_UART1OUTPROT_NMEA, NMEA_statement, valueSize);
}

bool ublox_gen9::setUART1msg_config()
{
  println(debugLevel::TRACE,"Args : ", "setUART1msg_config()");

  bool toggle = 0;
  uint8_t valueSize = 1;

  bool statement = true;

  for(int i=0; i<msgCount; i++)
  {
    statement = statement && setConfig(configLevel, msgKeys[i], toggle, valueSize);
  }

  if(bufferedPOSECEF)
  {
    statement = statement && setConfig(configLevel, CFG_MSGOUT_UBX_NAV_POSECEF_UART1, 1, valueSize);
  }
  
  return statement;
}

bool ublox_gen9::setUART2gen_config()
{
  println(debugLevel::TRACE,"Args : ", "setUART2general_config()");
  
  return setConfig(configLevel, CFG_UART2_ENABLED, 0, 1);
}

bool ublox_gen9::setSPIgen_config()
{
  println(debugLevel::TRACE,"Args : ", "setSPIgeneral_config()");

  return setConfig(configLevel, CFG_SPI_ENABLED, 0, 1);
}

//already disabled via SEL pin, crashes if you try to set it
//bool setI2Cgen_config()
//{
//  println(debugLevel::TRACE,"Args : ", "setI2Cgeneral_config()");
//
//  return setConfig(configLevel, CFG_I2C_ENABLED, 0, 1);
//}

bool ublox_gen9::setUSBprot_config()
{
  println(debugLevel::TRACE,"Args : ", "setUART2general_config()");

  uint8_t valueSize = 1;

  return setConfig(configLevel, CFG_USBINPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_USBINPROT_NMEA, USB_NMEA, valueSize) && setConfig(configLevel, CFG_USBOUTPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_USBINPROT_NMEA, USB_NMEA, valueSize);
}

bool ublox_gen9::setUSBmsg_config()
{
  println(debugLevel::TRACE,"Args : ", "setUSBmsg_config()");

  bool toggle = 0;
  uint8_t valueSize = 1;

  bool statement = true;

  for(int i=0; i<msgCount; i++)
  {
    statement = statement && setConfig(configLevel, msgKeys[i]+2, toggle, valueSize);
  }
  
  return statement;
}

//////////////////////////////////////////////////////// Init ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ublox_gen9::ublox_gen9(HardwareSerial HWSerial, long GPS_Baud, uint8_t GPS_Pltfrm_Model, uint16_t GPS_Time_To_Refresh, uint8_t GPS_Nav_Rate, uint8_t config_Level, long GPS_default_Baud, bool buffered_POS, bool NMEA_USB, long USB_Baud, bool NMEA_UART1, uint8_t GPS_Stop_Bits)
:fault_debug(ublox9_const::debug_ID, ublox9_const::debug_lvl),GPS_Serial(HWSerial),GPS_Baudrate(GPS_Baud),GPS_PltfrmModel(GPS_Pltfrm_Model),GPS_TimeToRefresh(GPS_Time_To_Refresh),GPS_NavRate(GPS_Nav_Rate),configLevel(config_Level), GPS_default_Baudrate(GPS_default_Baud), bufferedPOSECEF(buffered_POS),USB_NMEA(NMEA_USB),USB_Baudrate(USB_Baud),UART1_NMEA(NMEA_UART1),GPS_StopBits(GPS_Stop_Bits)
{}

bool ublox_gen9::initGPS()
{
  GPS_Serial.begin(GPS_Baudrate); //GPS Serial channel, if configLevel in flash : GPS_Serial.begin(GPS_default_Baudrate)
  
  if(!GPS_Serial) //If Serial port failed to open 
  {
    return false;
  }

  println(debugLevel::INFO,"     --->Successfully started GPS Serial channel");

  if(true) //if configLevel in flash : setConfig(configLevel, CFG_UART1_BAUDRATE, GPS_Baudrate, 4) //will not work since setConfig needs an answer from the GPS which will be given at a new baud rate, need a specific method
  {
    if(setBasicConfig())
    {
      println(debugLevel::INFO,"     --->Successfully loaded parameters\r\n     --->Waiting for GPS Fix...");
      //check wether we have a NAV solution already or not, wait if we don't

      uint8_t NavStatus = 0;

      do {
        delay(10000);
        NavStatus = getNavFixStatus();

        println(debugLevel::INFO,"       ---->Nav fix status (2=2D, 3=3D, Otherwise not acquired): " + String(NavStatus, DEC));

      } while(NavStatus<2 || NavStatus>3);

      println(debugLevel::INFO,"     --->Putting GPS to sleep...");
      
      lowPower(); //Send GPS module to low power state

      return true;
    }
  }
  else
  {      
    println(debugLevel::INFO,"     --->Failed to change Serial channel to final GPS_baudrate, check debug/use ucenter via USB");

    return false;
  }

  return false;
}

/////////////////////////////////////////////// Support Debug ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void ublox_gen9::print64(uint64_t value)
// { 
//     long left = value >> 32;
//     long right = (value << 32) >> 32;

//     if(left>0)
//     {
//       debug::Serial_USB.print(left, HEX);
      
//       long temp = right;
//       int baseRight = 32;

//       while(baseRight>=0 && (temp & 0x80000000)>>31 == 0)
//       {
//         baseRight--;
//         temp = temp << 1;
//       }
      
//       for(int i=0; i<7-floor(baseRight/4); i++)
//       {
//         debug::Serial_USB.print("0");
//       }
//     }

//     debug::Serial_USB.print(right, HEX);
// }

// void ublox_gen9::print64ln(uint64_t value)
// {  
//     long left = value >> 32;
//     long right = (value << 32) >> 32;

//     if(left>0)
//     {
//       debug::Serial_USB.print(left, HEX);

//       long temp = right;
//       int baseRight = 32;

//       while(baseRight>=0 && (temp & 0x80000000)>>31 == 0)
//       {
//         baseRight--;
//         temp = temp << 1;
//       }

//       for(int i=0; i<7-floor(baseRight/4); i++)
//       {
//         debug::Serial_USB.print("0");
//       }
//     }

//     debug::Serial_USB.println(right, HEX);
// }

/////////////////////////////////////////////// GPS Interface /////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//can either reset the GPS to load from flash or load from flash simply
void ublox_gen9::resetGPS()
{
  uint16_t payloadSize = 4;
  
  uint8_t* payload = new uint8_t[payloadSize];
  //clearMask - just have to put a single one somewhere and it'll clear all CFG items
  *payload = 0xFF;
  *(payload+1) = 0xFF;
  *(payload+2) = 0x04;
  *(payload+3) = 0x00;

  sendUBXframe(0x06, 0x04, payloadSize, payload, false); 

  delete[] payload;
}

uint16_t ublox_gen9::getRefreshRate(uint8_t level)
{
  println(debugLevel::TRACE,"Args : " + String(level), "getRefreshRate(level)");

  uint8_t* SplittedRefreshRate = getConfig(CFG_RATE_MEAS, level, 2);
  if(SplittedRefreshRate != nullptr) 
  {
      println(debugLevel::FULL,"18 - Success\n");
    
      return (uint16_t)*(SplittedRefreshRate+1)<<8 + *SplittedRefreshRate;
  }
  else
  {
      println(debugLevel::FULL,"18 - Fail\n");
    
      return 0; //impossible value for refresh rate 
  }
}

long* ublox_gen9::getPOS(bool LLH)
{
  println(debugLevel::TRACE,"Args : " + String(LLH), "getPOS(LLH)");

  int payloadSize = 20; //
  int payloadElementSize = 4;
  int msgID = 0x01;
  
  if(LLH)
  {
    payloadSize = 28;
    msgID = 0x02;
  }

  int payloadNumberElements = payloadSize/payloadElementSize;
  
  uint8_t* SplittedPOS = PollValue(0x01, msgID, payloadSize, bufferedPOSECEF && !LLH);
  
  if(SplittedPOS != nullptr)
  {
    uint64_t POS_sum = 0;
    long* POS = new long[payloadNumberElements];

    for(int i=0; i<payloadNumberElements; i++)
    {
      *(POS+i) = ((long) *(SplittedPOS+i*payloadElementSize+3))<<24 | ((long) *(SplittedPOS+i*payloadElementSize+2))<<16 | ((long) *(SplittedPOS+i*payloadElementSize+1))<<8 | *(SplittedPOS+i*payloadElementSize);
      POS_sum += *(POS+i);
    }

    delete[] SplittedPOS;

    if(POS_sum != 0)
    {
      if(isLogged(debugLevel::FULL))
      {
        println(debugLevel::FULL,"Got the following POS : ");
        for(long i=0; i<5; i++)
        {
          print(debugLevel::FULL, String(*(POS+i), HEX) + " ", "", true);
        }
        skipln(debugLevel::FULL);
      }

      return POS;   
    }
    else
    {
      println(debugLevel::TRACE,"POS - Fail 1\n");
      
      delete[] POS;
      return nullptr;
    }
      
  }
  else
  {
    println(debugLevel::TRACE,"POS - Fail 2\n");
    
    return nullptr;
  }
}

long *ublox_gen9::getPOSECEF()
{
  println(debugLevel::TRACE,"Args : ", "getPOSECEF()");
  return getPOS();
}

long *ublox_gen9::getPOSLLH()
{
  println(debugLevel::TRACE,"Args : ", "getPOSLLH()");
  return getPOS(true);
}

uint8_t ublox_gen9::getNavFixStatus()
{
  println(debugLevel::TRACE,"Args :", "getNavFixStatus()");
  
  int payloadSize = 92;
  int fixTypePos = 20;
  int flag1Pos = 21;
  int gnssFixOkFlagBit = 0;

  uint16_t PVTtime_sum = 0;
  uint8_t* SplittedPVT = PollValue(0x01, 0x07, payloadSize, false);
  
  if(SplittedPVT != nullptr)
  {
    for(int i=4; i<11; i++)
    {
      PVTtime_sum += *(SplittedPVT+i);
    }

    uint8_t fixType = *(SplittedPVT+fixTypePos);
    uint8_t flags1 = *(SplittedPVT+flag1Pos);
    uint8_t gnssFixOkFlag = (flags1<<(7-gnssFixOkFlagBit))>>7;

    if(isLogged(debugLevel::FULL))
    {
      println(debugLevel::FULL, "Got the following PVT : ");
      for(long i=0; i<payloadSize; i++)
      {
        print(debugLevel::FULL, String(*(SplittedPVT), HEX) + " ", "", true);
      }
      skipln(debugLevel::FULL);
    }

    delete[] SplittedPVT;
    
    if(PVTtime_sum != 0)
    {
      return fixType;   
    }
    else
    {
      println(debugLevel::TRACE,"Nav Fix Status - Fail to RX, try again\n");

      return 6;
    }
      
  }
  else
  {
    println(debugLevel::TRACE,"Nav Fix Status - Fail to Poll, try again\n");
    
    return 7;
  }
}

bool ublox_gen9::lowPower()
{
  //set measurement rate to 60s, set power mode to PSMCT (see integration manual on Power Management)
  return setConfig(configLevel, CFG_RATE_MEAS, 60000, 2) && setConfig(configLevel, CFG_PM_OPERATEMODE, 2, 1);
}

bool ublox_gen9::highPower()
{
  //set measurement rate to nominal, set power mode to FULL (see integration manual on Power Management)
  return setNavRate() && setConfig(configLevel, CFG_PM_OPERATEMODE, 0, 1);
}