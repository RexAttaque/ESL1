#include <Sensors/GPS_UBX9/InterfaceLibs/ubloxGen9.h>

//////////////////////////////////////////////////////// Init ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ublox_gen9::setBasicConfig()
{   
  bool refresh = setRefreshRate(1000);
  
  if(refresh)
  {
    delay(5000);
    bool rate = setNavRate();
    bool dynMod = setPltfrmModel();
    bool UART1gen = setUART1gen_config();
    bool UART1prot = setUART1prot_config();
    delay(5000);
    //bool UART1msg = setUART1msg_config();
    delay(5000);
    bool UART2gen = setUART2gen_config();
    bool SPIgen = setSPIgen_config();
    //bool I2Cgen = setI2Cgen_config(); //already disabled via SEL pin, crashes if you try to set it
    //bool USBprot = setUSBprot_config();
    delay(5000);
    //bool USBmsg = setUSBmsg_config();
    delay(5000);
    refresh = setRefreshRate(GPS_TimeToRefresh);
    delay(5000);

    return refresh && rate && dynMod && UART1gen && UART1prot && UART2gen && SPIgen;
    //return refresh && rate && dynMod && UART1gen && UART1prot && UART1msg && UART2gen && SPIgen && I2Cgen && USBprot && USBmsg;
  }

  return false;
}

/////////////////////////////////////////////// Support Core ////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ublox_gen9::calculateCheckSum(uint8_t* frame, uint16_t frameSize, bool receiveMode)
{
//  Serial.println("calculateCheckSum");
  
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
//    Serial.println("TX");
//    Serial.println("");
    
    *(frame+frameSize-2) = checksum[0];
    *(frame+frameSize-1) = checksum[1];

    return true;
  }
  else
  {
//    Serial.print("RX ");
    
    if( *(frame+frameSize-2) == checksum[0] && *(frame+frameSize-1) == checksum[1] )
    {
//      Serial.println("Pass");
//      Serial.println("");
      return true;
    }
    else
    {
//      Serial.println("Fail");
//      Serial.println("");
      return false;
    }
  }
}

//splits a 64 bit uint to an array of 8 bit uints
//value and value length in bytes
uint8_t* ublox_gen9::splitTo8bit(uint64_t value, uint16_t val_length)
{
  //Serial.println("splitTo8Bit");
  //print64ln(value);
  //Serial.println(val_length);
  
  if(val_length <= 0)
  {
    return nullptr;
  }
  
  uint8_t* buff = new uint8_t[val_length];
  uint64_t temp;

  //Serial.print("Split number value : ");
  for(long i=val_length-1; i>=0; i--)
  {
    temp = value>>(val_length-i-1)*8;

    //print64(temp);
    //Serial.print("/");
    
    *(buff+i) = temp & 0xFF;

    //Serial.print(*(buff+i), HEX);
    //Serial.print(" ");
  }

  //Serial.println("");
  //Serial.println("");

  return buff;
}

/////////////////////////////////////////////// Core UART ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//extracted length in bytes counting start bytes etc. (full frame length)
uint8_t* ublox_gen9::receiveUBXframe(uint16_t extractedLength)
{
//  Serial.println("receiveUBXframe");
//  Serial.print("extractedLength : ");
//  Serial.println(extractedLength);
  
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
  
  while(Serial1.available() && ReceivedFrameSum == 0 && ReceivedFrameSize != extractedLength)
  {
//    Serial.print("Attempted RX... ");
    ReceivedFrameSize = Serial1.readBytes(buff, extractedLength);

    for(int i=0; i<ReceivedFrameSize; i++)
    {
      ReceivedFrameSum += *(buff+i);
    }
  }
  
//      Serial.println("Received following message : ");
//      for(long i=0; i<extractedLength; i++)
//      {
//        Serial.print(*(buff+i), HEX);
//        Serial.print(" ");
//      }
//      Serial.println("");

  //flush whatever is left in the Serial1 buffer to prevent interactions
  while(Serial1.available())
  {
    Serial1.read();
  }

  if(calculateCheckSum(buff, extractedLength, true))
  {
//    Serial.println("Checksum Pass on Receive");
//    Serial.println("");
    return buff;
  }
  else
  {
//    Serial.println("Checksum Fail on Receive");
//    Serial.println("");
    return nullptr;
  }
}

bool ublox_gen9::sendUBXframe(uint8_t cmdClass, uint8_t messageID, uint16_t payloadSize, uint8_t* payload, bool configMode)
{
//  Serial.println("sendUBXframe");
  
  if(cmdClass <= 0 || messageID <= 0)
  {
    return false;
  }

  uint8_t* splittedSize;
  
  if(payloadSize > 0)
  {
//    Serial.print("Payload exists of size ");
//    Serial.println(payloadSize);
    splittedSize = splitTo8bit(payloadSize, 2);
  }
  else
  {
//    Serial.println("Payload does not exist");
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

//      Serial.print("UBX frame : ");
//      for (int i=0; i<frameSize; i++)
//      {
//        Serial.print(*(buff+i), HEX);
//        Serial.print(" ");
//      }
//      Serial.println("");

  size_t SentSize = 0;
  
  while(SentSize != frameSize)
  {
    SentSize = Serial1.write(buff, frameSize);
//  Serial.println(SentSize);
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

//  Serial.print("Outcome :");
//  Serial.println(outcome);
//  Serial.println("");

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
//      Serial.print("Level :");
//      Serial.println(level, DEC);
//      Serial.print("KeyID :");
//      Serial.println(keyID, HEX);
  
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

//  Serial.print("Split Payload : ");
//  for (int i=0; i<payloadSize; i++)
//  {
//    Serial.print(*(payload+i), HEX);
//    Serial.print(" ");
//  }
//  Serial.println("");

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
//        Serial.print(*(splittedGetValue+i), HEX);
//        Serial.print("/");
//        Serial.print(*(splittedValue-i-1+valueSize), HEX);
        
        success = *(splittedGetValue+i)==*(splittedValue+valueSize-i-1);

//        Serial.print(" - ");
      }

//      Serial.println("");
    }

    delete[] splittedGetValue;
  }
  
  delete[] payload;
  delete[] splittedValue;
  delete[] splittedKey;

//      if(success) 
//      {
//        Serial.println("Config successfully applied and checked");
//        Serial.println("");
//      }


  return success;
}


// 0 - RAM
// 1 - BBR
// 2 - Flash
// 7 - Default value
uint8_t* ublox_gen9::getConfig(long keyID, uint8_t level, uint8_t expectedValueSize)
{
//  Serial.println("getConfig");
  
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
      value = nullptr;
    }

    delete[] frame;

//    Serial.println("getConfig - Success");
//    Serial.println("");
    
    return value;
  }
  else
  {
//    Serial.println("getConfig - Fail");
//    Serial.println("");
    
    return nullptr;
  }
}

/////////////////////////////////////////////// Core GPS Polling /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t* ublox_gen9::PollValue(uint8_t cmdClass, uint8_t messageID, uint16_t expectedPayloadSize, bool expectedInBuffer)
{
//  Serial.println("PollValue");
  
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

//        Serial.println("Got the following poll response : ");
//        for(long i=0; i<expectedPayloadSize; i++)
//        {
//          Serial.print(*(payload+i), HEX);
//          Serial.print(" ");
//        }
//        Serial.println("");
      }
      else
      {
//        Serial.println("PollValue - Fail - No RX");
//        Serial.println("");
        payload = nullptr;
      }
  
      delete[] frame;
      
      return payload;
    }
    else
    {
//      Serial.println("PollValue - Fail - No TX");)
//      Serial.println("");

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
//  Serial.println("setRefreshRate");
//  Serial.println("");
  
  return setConfig(configLevel, CFG_RATE_MEAS, refreshRate, 2);
}

bool ublox_gen9::setNavRate()
{
//  Serial.println("setNavRate");
//  Serial.println("");
  
  return setConfig(configLevel, CFG_RATE_NAV, GPS_NavRate, 2);
}

bool ublox_gen9::setPltfrmModel()
{
//  Serial.println("setPlatformModel");
//  Serial.println("");

  return ublox_gen9::setConfig(configLevel, CFG_NAVSPG_DYNMODEL, GPS_PltfrmModel, 1);
}

bool ublox_gen9::setUART1gen_config()
{
//  Serial.println("setUART1general_config");
//  Serial.println("");

  return setConfig(configLevel, CFG_UART1_STOPBITS, GPS_StopBits, 1);
}

bool ublox_gen9::setUART1prot_config()
{
//  Serial.println("setUART1protocol(I/O)_config");
//  Serial.println("");

  uint8_t valueSize = 1;
  bool NMEA_statement = UART1_NMEA && !bufferedPOSECEF;
  
  return setConfig(configLevel, CFG_UART1INPROT_NMEA, NMEA_statement, valueSize) && setConfig(configLevel, CFG_UART1INPROT_RTCM3X, 0, valueSize) && setConfig(configLevel, CFG_UART1OUTPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_UART1OUTPROT_NMEA, NMEA_statement, valueSize);
}

bool ublox_gen9::setUART1msg_config()
{
//  Serial.println("setUART1msg_config");
//  Serial.println("");

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
//  Serial.println("setUART2general_config");
//  Serial.println("");

  return setConfig(configLevel, CFG_UART2_ENABLED, 0, 1);
}

bool ublox_gen9::setSPIgen_config()
{
//  Serial.println("setSPIgeneral_config");
//  Serial.println("");

  return setConfig(configLevel, CFG_SPI_ENABLED, 0, 1);
}

//already disabled via SEL pin, crashes if you try to set it
//bool setI2Cgen_config()
//{
////  Serial.println("setI2Cgeneral_config");
////  Serial.println("");
//
//  return setConfig(configLevel, CFG_I2C_ENABLED, 0, 1);
//}

bool ublox_gen9::setUSBprot_config()
{
//  Serial.println("setUART2general_config");
//  Serial.println("");

  uint8_t valueSize = 1;

  return setConfig(configLevel, CFG_USBINPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_USBINPROT_NMEA, USB_NMEA, valueSize) && setConfig(configLevel, CFG_USBOUTPROT_UBX, 1, valueSize) && setConfig(configLevel, CFG_USBINPROT_NMEA, USB_NMEA, valueSize);
}

bool ublox_gen9::setUSBmsg_config()
{
//  Serial.println("setUSBmsg_config");
//  Serial.println("");

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
:GPS_Serial(HWSerial),GPS_Baudrate(GPS_Baud),GPS_PltfrmModel(GPS_Pltfrm_Model),GPS_TimeToRefresh(GPS_Time_To_Refresh),GPS_NavRate(GPS_Nav_Rate),configLevel(config_Level), GPS_default_Baudrate(GPS_default_Baud), bufferedPOSECEF(buffered_POS),USB_NMEA(NMEA_USB),USB_Baudrate(USB_Baud),UART1_NMEA(NMEA_UART1),GPS_StopBits(GPS_Stop_Bits)
{
}

bool ublox_gen9::initGPS()
{
  GPS_Serial.begin(GPS_Baudrate); //GPS Serial channel
  while (!GPS_Serial) delay(10);  // wait for serial1 port to open!
//      Serial.println("Successfully started Serial 1 (GPS) channel");
//      Serial.println("");


  if(true) //setConfig(configLevel, CFG_UART1_BAUDRATE, GPS_Baudrate, 4)
  {
    if(setBasicConfig())
    {
//          Serial.println("Successfully loaded parameters");
//          Serial.println(""); 
  
//      //check wether we have a NAV solution already or not, wait if we don't
//      Serial.println("Waiting for Nav Fix...");
//  
//      uint8_t NavStatus = 0;
//      do {
//        delay(10000);
//        NavStatus = getNavFixStatus();
//        Serial.print("Nav fix status: "); Serial.println(NavStatus, DEC); 
//      } while(NavStatus<3 || NavStatus>4);
    }
    else
    {
//          Serial.println("Failed to load parameters, check debug, exit");
//          delay(100000);
//          exit(0);
    }
  }
  else
  {
//        Serial.println("Failed to initialize GPS Serial channel, exit");
//        delay(100000);
//        exit(0);
  }
}

/////////////////////////////////////////////// Support Debug ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//void ublox_gen9::print64(uint64_t value)
//{  
//  long left = value >> 32;
//  long right = (value << 32) >> 32;
//
//  if(left>0)
//  {
//    Serial.print(left, HEX);
//    
//    long temp = right;
//    int baseRight = 32;
//  
//    while(baseRight>=0 && (temp & 0x80000000)>>31 == 0)
//    {
//      baseRight--;
//      temp = temp << 1;
//    }
//    
//    for(int i=0; i<7-floor(baseRight/4); i++)
//    {
//      Serial.print("0");
//    }
//  }
//  
//  Serial.print(right, HEX);
//}
//
//void print64ln(uint64_t value)
//{  
//  long left = value >> 32;
//  long right = (value << 32) >> 32;
//
//  if(left>0)
//  {
//    Serial.print(left, HEX);
//
//    long temp = right;
//    int baseRight = 32;
//
//    while(baseRight>=0 && (temp & 0x80000000)>>31 == 0)
//    {
//      baseRight--;
//      temp = temp << 1;
//    }
//  
//    for(int i=0; i<7-floor(baseRight/4); i++)
//    {
//      Serial.print("0");
//    }
//  }
//  
//  Serial.println(right, HEX);
//}

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

//uint16_t getRefreshRate(uint8_t level)
//{
//  Serial.println("getRefreshRate");
//  
//  uint8_t* SplittedRefreshRate = getConfig(CFG_RATE_MEAS, level, 2);
//  if(SplittedRefreshRate != nullptr) 
//  {
////    Serial.println("18 - Success");
////    Serial.println("");
//    
//    return (uint16_t)*(SplittedRefreshRate+1)<<8 + *SplittedRefreshRate;
//  }
//  else
//  {
////    Serial.println("18 - Fail");
////    Serial.println("");
//    
//    return 0; //impossible value for refresh rate 
//  }
//}

long* ublox_gen9::getPOSECEF()
{
//  Serial.println("getPOSECEF");
  int payloadSize = 20; //
  int payloadElementSize = 4;
  int payloadNumberElements = payloadSize/payloadElementSize;
  
  uint8_t* SplittedPOSECEF = PollValue(0x01, 0x01, payloadSize, bufferedPOSECEF);
  
  if(SplittedPOSECEF != nullptr)
  {
    uint64_t POSECEF_sum = 0;
    long* POSECEF = new long[payloadNumberElements];

    for(int i=0; i<payloadNumberElements; i++)
    {
      *(POSECEF+i) = ((long) *(SplittedPOSECEF+i*payloadElementSize+3))<<24 | ((long) *(SplittedPOSECEF+i*payloadElementSize+2))<<16 | ((long) *(SplittedPOSECEF+i*payloadElementSize+1))<<8 | *(SplittedPOSECEF+i*payloadElementSize);
      POSECEF_sum += *(POSECEF+i);
    }

    delete[] SplittedPOSECEF;

    if(POSECEF_sum != 0)
    {
//      Serial.println("Got the following POSECEF : ");
//      for(long i=0; i<5; i++)
//      {
//        Serial.println(*(POSECEF+i), HEX);
//      }
//      Serial.println("");

      return POSECEF;   
    }
    else
    {
//      Serial.println("POSECEF - Fail");
//      Serial.println("");

      return nullptr;
    }
      
  }
  else
  {
//    Serial.println("POSECEF - Fail");
//    Serial.println("");
    
    return nullptr;
  }
}

uint8_t ublox_gen9::getNavFixStatus()
{
//      Serial.println("getNavFixStatus");
  
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

//        Serial.println("Got the following PVT : ");
//        for(long i=0; i<payloadSize; i++)
//        {
//          Serial.print(*(SplittedPVT), HEX);
//          Serial.print(" ");
//        }
//        Serial.println("");

    delete[] SplittedPVT;
    
    if(PVTtime_sum != 0)
    {
      return fixType;   
    }
    else
    {
//          Serial.println("Nav Fix Status - Fail to RX, try again");
//          Serial.println("");

      return 6;
    }
      
  }
  else
  {
//        Serial.println("Nav Fix Status - Failed to Poll, try again");
//        Serial.println("");
    
    return 7;
  }
}