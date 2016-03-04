//#######################################################################################################
//#################################### Plugin-017: NodoTest    ############################################
//#######################################################################################################

/*********************************************************************************************\
* Funktionele beschrijving: Deze plugin toont de Hardware status, Bedoeld voor testdoeleinden
*                           Nodo hardwrae
* 
* <Geef hier een beschrijving van de funktionele werking van het plugin en hoe deze door de gebruiker
* moet worden aangeroepen.>
*
* Auteur             : Paul Tonkes (p.k.tonkes@gmail.com)
* Support            : www,nodo-domotica.nl
* Datum              : 16-09-2013
* Versie             : 1.0
* Nodo productnummer : <Nodo productnummer. Toegekend door Nodo team>
* Compatibiliteit    : Vanaf Nodo build nummer 580
* Syntax             : NodoTest
*
\***********************************************************************************************/

#define WIRED_OK            0
#define WIRED_NOT_CONNECTED 1
#define WIRED_SHORT         2
#define WIRED_OPEN          3

void HWSendTestEvent(byte port)
  {
  // Verzend naar een andere Nodo de opdracht om een UserEvent te versturen
  // Dit voor alle poorten. Ans dit event weer wordt intvangen dan kunnen we met zekerheid
  // stellen dat de zend/ontvang delen van de poorten goed funktioneren.        
  struct NodoEventStruct TempEvent;
  ClearEvent(&TempEvent);    
  TempEvent.Type                  = NODO_TYPE_COMMAND;
  TempEvent.Command               = CMD_SEND_USEREVENT;  
  TempEvent.Par1                  = 255;
  TempEvent.Par2                  = 255;
  TempEvent.DestinationUnit       = 0;
  TempEvent.Port                  = port;
  TempEvent.Flags                 = TRANSMISSION_CONFIRM;
  SendEvent(&TempEvent, false, true,Settings.WaitFree==VALUE_ON);
  }

byte CheckWiredLines(void)
  {
  byte w,x;

  bitWrite(HW_Config,HW_WIRED_IN, false);
  bitWrite(HW_Config,HW_WIRED_OUT,false);
  for(x=0;x<WIRED_PORTS;x++)
    pinMode(PIN_WIRED_OUT_1+x,OUTPUT); // definieer Arduino pin's voor Wired-Out als output
  
  // Alle WiredOut lijnen hoog.
  for(x=0;x<WIRED_PORTS;x++)
    digitalWrite(PIN_WIRED_OUT_1+x,HIGH);
  
  // Alle WiredOut lijnen laag.
  for(x=0;x<WIRED_PORTS;x++)
    digitalWrite(PIN_WIRED_OUT_1+x,LOW);
  
  // Tel hoeveel WiredIn lijnen hoog zijn
  w=0;
  for(x=0;x<WIRED_PORTS;x++)
    if((analogRead(PIN_WIRED_IN_1+x)>1000))
      w++;

  // Zijn de Wired-In verbonden met de Wired-out? Dan zijn ze allemaal hoog.
  if(w==WIRED_PORTS)
    return WIRED_NOT_CONNECTED;


  // Loop alle lijnen langs on te kijken of de WiredIn de WiredOut volgt        
  for(w=0;w<WIRED_PORTS;w++)
    {
    // Maak steeds een lijn hoog.
    for(x=0;x<WIRED_PORTS;x++)
      digitalWrite(PIN_WIRED_OUT_1+x,LOW);

    digitalWrite(PIN_WIRED_OUT_1+w,HIGH);

   // Is de Wired-In meegenomen naar HIGH door de Wired-Out?
   // Als laag, dan vermoedelijk onterecht verbonden met GND of andere Wired.
   if((analogRead(PIN_WIRED_IN_1+w)<1000))
     return WIRED_SHORT;
        
   // Zijn de andere lijnen allemaal nog laag?
   // Als andere lijn hoog, dan fout in de lijn
   for(x=0;x<WIRED_PORTS;x++)
     if(x!=w && (analogRead(PIN_WIRED_IN_1+x)>50))
       return WIRED_OPEN;
    }

  for(x=0;x<WIRED_PORTS;x++)
    digitalWrite(PIN_WIRED_OUT_1+x,LOW);

  bitWrite(HW_Config,HW_WIRED_IN, true);
  bitWrite(HW_Config,HW_WIRED_OUT,true);

  return WIRED_OK;
  }

void ShowTestResults(void)
  {
  int w,x,Ok=0;
  
  Ok=CheckWiredLines();
            
  Serial.println(F("!******************************************************************************!"));
  Serial.print(F("HWConfig      = 0x"));Serial.println(HW_Config,HEX);
  Serial.print(F("BIC-0         = "));if(bitRead(HW_Config,HW_BIC_0)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("BIC-1         = "));if(bitRead(HW_Config,HW_BIC_1)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("BIC-2         = "));if(bitRead(HW_Config,HW_BIC_2)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("BIC-3         = "));if(bitRead(HW_Config,HW_BIC_3)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("Ethernet      = "));if(bitRead(HW_Config,HW_ETHERNET)){Serial.println("Ok ");}else{Serial.println("-");}
  Serial.print(F("SDCard        = "));if(bitRead(HW_Config,HW_SDCARD)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("Serial        = "));if(bitRead(HW_Config,HW_SERIAL)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("Clock         = "));if(bitRead(HW_Config,HW_CLOCK)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("RF_RX         = "));if(bitRead(HW_Config,HW_RF_RX)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("RF_TX         = "));if(bitRead(HW_Config,HW_RF_TX)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("IR_RX         = "));if(bitRead(HW_Config,HW_IR_RX)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("IR_TX         = "));if(bitRead(HW_Config,HW_IR_TX)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("I2C           = "));if(bitRead(HW_Config,HW_I2C)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("Pulsecount    = "));if(bitRead(HW_Config,HW_PULSE)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("WebApp        = "));if(bitRead(HW_Config,HW_WEBAPP)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("WiredIn       = "));if(bitRead(HW_Config,HW_WIRED_IN)){Serial.println("Ok");}else{Serial.println("-");}
  Serial.print(F("WiredOut      = "));if(bitRead(HW_Config,HW_WIRED_IN)){Serial.println("Ok");}else{Serial.println("-");}

  
  if (Ok==WIRED_NOT_CONNECTED)
    {
    Serial.println(F("Wired         = Are all Wired-In (1..) lines connected to Wired-Out (1..) ?"));
    }
  else if (Ok==WIRED_SHORT)
    {
    Serial.println(F("Wired         = Short circuit in Wired lines?"));
    }
  else if (Ok==WIRED_OPEN)
    {
    Serial.println(F("Wired         = No connection in Wired lines?"));
    }
  if(Ok!=WIRED_OK)
    {
    // Omdat er een fout zit, direct de lijnen WiredOut definieren als een input zodat er geen 
    // schade ontstaat aan de ATMega
    for(x=0;x<WIRED_PORTS;x++)
      pinMode(PIN_WIRED_OUT_1+x,INPUT); // definieer Arduino pin's voor Wired-Out

    for(x=0;x<WIRED_PORTS;x++)
      {
      Serial.print(F("WiredIn-"));Serial.print(x);Serial.print(F("     = "));Serial.println(analogRead(PIN_WIRED_IN_1+x)+1); 
      }        
    }

  Serial.println(F("!******************************************************************************!"));        

  for(x=0;x<WIRED_PORTS;x++)
    digitalWrite(PIN_WIRED_OUT_1+x,LOW);
  }


#define PLUGIN_NAME "NodoTest"
#define PLUGIN_ID   17

boolean Active=false;
unsigned long HW_ConfigCheck=0L;
int HWSecCounter;
  
boolean Plugin_017(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  
  int x;
  
  switch(function)
    {    
    case PLUGIN_INIT:
      {
      for(x=0;x<WIRED_PORTS;x++)
        pinMode(PIN_WIRED_OUT_1+x,INPUT); // definieer Arduino pin's voor Wired-Out
      }

    case PLUGIN_EVENT_IN:
      {
      if(event->Type     == NODO_TYPE_EVENT && 
         event->Command  == EVENT_USEREVENT &&
         event->Par1     == 255 &&
         event->Par2     == 255)
        {
        switch(event->Port)
          {
          case VALUE_SOURCE_IR:
            bitWrite(HW_Config,HW_IR_TX,true);
            break;
          case VALUE_SOURCE_RF:
            bitWrite(HW_Config,HW_RF_TX,true);
            break;
          }
        }          
      }

    case PLUGIN_ONCE_A_SECOND:
      {
      if(Active)
        {
        switch(HWSecCounter++)
          {
          case 1:
            HWSendTestEvent(VALUE_SOURCE_RF);
            break;

          case 5:
            HWSendTestEvent(VALUE_SOURCE_IR);
            break;

          case 7:
            HWSendTestEvent(VALUE_SOURCE_I2C);
            break;

          case 8:
            {            
            CheckWiredLines();          
            ShowTestResults();
            if(HW_ConfigCheck)
              {
              if(HW_Config==HW_ConfigCheck)
                {
                for(x=0;x<=4;x++)
                  {
                  Beep(2500,25);
                  delay(100);
                  }
                Serial.println(F("NodoTest: HWConfig is correct!"));
                }
              else
                {
                Beep(500,1000);
                Serial.println(F("NodoTest: HWConfig is NOT correct!"));
                }
              }
            Active=false;
            }
          }
        }
      break;
      }
      
    case PLUGIN_COMMAND:
      {
      Beep(1000,250);
      Settings.TransmitIR=VALUE_ON;
      Settings.TransmitRF=VALUE_ON;
      Settings.RawSignalReceive=VALUE_OFF;
    
      HWSecCounter=0;

      for(x=0;x<WIRED_PORTS;x++)
        {
        Settings.WiredInputPullUp[x]=VALUE_ON;
        pinMode(A0+PIN_WIRED_IN_1+x,INPUT_PULLUP);
        }

      HW_ConfigCheck=event->Par2;
      Active=true;
      Serial.println(F("NodoTest: Running...(please wait a few seconds)\n"));
      Beep(1000,50);

      success=true;
      break;
      }      
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
          event->Type    = NODO_TYPE_PLUGIN_COMMAND;
          event->Command = PLUGIN_ID; // nummer van dit plugin
          if(GetArgv(string,TempStr,2))
            event->Par2    = str2int(TempStr);
          success=true;
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME);               // Commando 
      strcat(string," ");
      strcat(string,int2str(event->Par2));

      break;
      }
    #endif //MMI
    }
    
  return success;
  }
