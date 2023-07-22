#include <GSM/Sim800L.h>

// Check wether the SIM is detected or not
GSM_obj::GSM_obj(HardwareSerial HWSerial, String GSM_num, long Baud)
:GSM_Serial(HWSerial),phoneNumber(GSM_num),GSM_baudrate(Baud)
{}

String GSM_obj::getLastTX()
{
    return TX;
}

void GSM_obj::setPhoneNumber(String GSM_num)
{
    phoneNumber = GSM_num;
}

void GSM_obj::setTX(String message)
{
    TX = message;
}

bool GSM_obj::check_SIM_GSM()
{
  int ICCID_start_pos = 10;

  //check if the ICCID end with 89 which should always be the case
  if(check_AT_OK_GSM("AT+CCID") && RX.substring(ICCID_start_pos, ICCID_start_pos+2) == "89")
  {
    //code to fetch only the whole ICCID
    if(debug_GSM.isLogged(debugLevel::FULL)) 
    {
        int i=0;
        while(RX.charAt(ICCID_start_pos+i) != char(13)) //char(13) is the return carriage character
        {
          i++;
        }
        debug_GSM.println(debugLevel::FULL,"ICCID : " + RX.substring(ICCID_start_pos,ICCID_start_pos+i), "check_SIM_GSM()"); // Print ICCID
    }
    
    return true;
  }

  return false;
}

// Check whether the module has registered on the network
bool GSM_obj::check_REG_GSM()
{
  int REG_status_pos = 20;

  if(check_AT_OK_GSM("AT+CREG?") && RX.charAt(REG_status_pos) == char(49)) //char(49) is "1"
  { 
    return true;
  }

  return false;
}

// Signal quality test, value range is 0 to 31 with 31 being excellent. Returns true of signal quality is above the floor, false otherwise
bool GSM_obj::check_SIG_GSM(int qualityFloor)
{
  int QL_start_pos = 15;

  if(check_AT_OK_GSM("AT+CSQ"))
  {
    int i=0;
    while(RX.charAt(QL_start_pos+i) != char(44)) //char(44) is ","
    {
      i++;
    }
  
    int signalQuality = RX.substring(QL_start_pos, QL_start_pos+i).toInt();
  
    if(signalQuality != 99 && signalQuality >= qualityFloor)
    {  
      return true;
    }
  }

  return false;
}

void GSM_obj::read_RX()
{
  RX = GSM_Serial.readString();
  debug_GSM.println(debugLevel::FULL,RX,"read_RX()"); 
}

// Function to check wether the last two characters of an answer are "OK" indicating the receiver answered and understood, returns the RX if OK is found, returns an empty string otherwise
bool GSM_obj::check_RX_OK()
{
    int RX_length = RX.length();
    //check the substring before the /r/n characters for "OK"
    if(RX.substring(RX_length-4, RX_length-2) == "OK")
    {
        debug_GSM.println(debugLevel::FULL,"PASS\n","check_RX_OK()");

        return true;
    }

    debug_GSM.println(debugLevel::FULL,"FAIL\n","check_RX_OK()");
    RX="";
    return false;
}

bool GSM_obj::check_AT_OK_GSM(char c)
{
    TRX_AT_GSM(c);
    return check_RX_OK();
}

bool GSM_obj::check_AT_OK_GSM(String AT_COMMAND)
{ 
    TRX_AT_GSM(AT_COMMAND);
    return check_RX_OK();
}

void GSM_obj::TRX_AT_GSM(char c)
{
    GSM_Serial.write(c);
    read_RX();
}

// Function to send an AT command to the SIM800L module, returns the answer
void GSM_obj::TRX_AT_GSM(String AT_COMMAND) 
{
    GSM_Serial.println(AT_COMMAND);
    read_RX();
}



bool GSM_obj::init()
{
  debug_GSM.println(debugLevel::INFO,"!! GSM Check/Init !!","init()");

  GSM_Serial.begin(GSM_baudrate);

  if(GSM_Serial)
  {
    debug_GSM.println(debugLevel::INFO,"\n ->Succesfully opened GSM Serial channel");
    
    // Send attention command to check if all fine, module should answer by OK -> true ; check that a SIM is in the module
    if(check_AT_OK_GSM("AT") && check_SIM_GSM())
    {
      uint8_t attempts = 0;

      debug_GSM.println(debugLevel::INFO," ->Module is responsive, SIM detected \r\n ->Waiting for registration on network... \r\n");

      // Wait for network registration
      while(attempts<GSM_const::maxRegAttempts && check_REG_GSM() == false)
      {
        attempts++;
        delay(GSM_const::timeBetweenRegCheck);
      }

      if(attempts<GSM_const::maxRegAttempts)
      {
        debug_GSM.println(debugLevel::INFO," ->Registered on network !");

        if(!check_SIG_GSM(GSM_const::signalQuality_floor)) 
        {
          debug_GSM.println(debugLevel::INFO,"   -->WARNING, SIGNAL QUALITY LOW");
        }

        bool send = PrepSend_s1();
        if(send)
        {
          delay(100);
          send = PrepSend_s2();
          if(send)
          {
            delay(100);
            send = PrepSend_s3();
            if(send)
            {
              delay(100);
              setTX("ESL1 GSM Init send test");
              send = sendSMS();
              delay(100);
            }
          }
        }

        if(send)
        {
            if(goIdle())
            {
              debug_GSM.println(debugLevel::INFO," ->Module idling, waiting for wake up call...");
            }

            debug_GSM.println(debugLevel::INFO,"\n ->GSM INIT PASS\n");
            return true;
        }
      }
      else
      {
        debug_GSM.println(debugLevel::INFO," ->Could not register on network");
      } 
    }
  }

  debug_GSM.println(debugLevel::INFO," ->ERROR, GSM INIT FAIL, CHECK DEBUG\n");
  return false;
}

bool GSM_obj::goLive()
{
  return check_AT_OK_GSM("AT+CFUN=1"); //set full functionnality
}

bool GSM_obj::goIdle()
{
  return check_AT_OK_GSM("AT+CFUN=0"); //set minimum functionnality
}

bool GSM_obj::PrepSend_s1()
{
  // Starts SMS mode
  return check_AT_OK_GSM("AT+CMGF=1"); //if check_AT_OK_GSM doesn't work, simply use GSM_Serial.print
}

bool GSM_obj::PrepSend_s2()
{
  // Receiving phone number
  return check_AT_OK_GSM("AT+CMGS=\"" + phoneNumber + "\"\r");
}

bool GSM_obj::PrepSend_s3()
{
  // Message
  return check_AT_OK_GSM(TX);
}


bool GSM_obj::sendSMS()
{
  debug_GSM.println(debugLevel::TRACE,"Sending text message...", "sendSMS()");
  
  // CTR+Z in ASCII, indicates the end of the message
  bool status = check_AT_OK_GSM(26); //check_AT_OK_GSM didn't work previously, use check_AT_OK_GSM("")
  
  while(GSM_Serial.available()) {
    if(debug_GSM.isLogged(debugLevel::FULL))
    {
      debug_GSM.write(debugLevel::FULL,GSM_Serial.read()); //Clear the contents of the sim800L TX buffer and display them in the console for debugging
    }
    else
    {
      GSM_Serial.read(); //Clear the contents of the sim800L TX buffer
    }
  }
  
  debug_GSM.println(debugLevel::TRACE,"\nText sent");

  return status;
}