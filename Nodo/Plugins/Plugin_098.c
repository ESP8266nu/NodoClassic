//#######################################################################################################
//############################### Plugin-098: SendHTTP demo for Domoticz JSON API #######################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides direct peer-to-peer HTTP communication between Nodo unit's
 * 
 * Author             : M vd Broek
 * Date               : Feb 2015
 * Version            : Beta 0.1
 * Compatibility      : R787
 * Syntax             : "DomoticzHTTP <variablesend/variableget>,<var>,<sensortype>,<IDX>
 \*********************************************************************************************/

boolean P098SendCustomHTTPRequest(char* Request, byte ip, unsigned long port, byte varindex);

#define PLUGIN_098_IP 8		 // x.x.x.8 = adress of Domoticz
#define PLUGIN_098_PORT 8080	 // Default port of Domoticz

#define PLUGIN_ID 98
#define PLUGIN_NAME "DomoticzHTTP"

boolean Plugin_098(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_098_CORE

  #if NODO_MEGA
  case PLUGIN_COMMAND:
    {
					  // Par 1 ='command/type'
      byte Par2=event->Par2 & 0xff;	  // Par 2 = Variabel nr or 0=Off, 1=On 
      byte Par3=event->Par2>>8 & 0xff; 	  // Par 3 = sensortype
      byte Par4=event->Par2>>16 & 0xffff; // Par 4 = IDX

      byte x;
      char *HttpRequest=(char*)malloc(INPUT_LINE_SIZE+1);

      if (event->Par1 == CMD_VARIABLE_SEND)
        {
          if (Par3 == 1)			// Single value Sensor
            {
              strcpy(HttpRequest,"/json.htm?type=command&param=udevice&idx=");
              strcat(HttpRequest,int2str(Par4));
              strcat(HttpRequest,"&nvalue=0&svalue=");
              //strcat(HttpRequest,int2str(UserVar[Par2-1]));
              dtostrf(UserVar[Par2-1], 0, 2,HttpRequest+strlen(HttpRequest));
            }

          if (Par3 == 10)		// switch
            {
              strcpy(HttpRequest,"/json.htm?type=command&param=switchlight&idx=");
              strcat(HttpRequest,int2str(Par4));
              strcat(HttpRequest,"&switchcmd=");
              if (Par3==1)
                strcat(HttpRequest,"On");
              else
                strcat(HttpRequest,"Off");
              strcat(HttpRequest,"&level=0");
            }

        }

      if (event->Par1 == CMD_VARIABLE_GET)
        {
          if (Par3 == 1)			// Get Sensor Data
            {
              strcpy(HttpRequest,"/json.htm?type=devices&rid=");
              strcat(HttpRequest,int2str(Par4));
            }
        }

      x=P098SendCustomHTTPRequest(HttpRequest,PLUGIN_098_IP,PLUGIN_098_PORT,Par2);
      free(HttpRequest);

      success=true;
      break;
     }
  #endif

#endif // PLUGIN_098_CORE

   #if NODO_MEGA
   case PLUGIN_MMI_IN:
     {
     char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);
     if(GetArgv(string,TempStr,1))
       {
       if(strcasecmp(TempStr,PLUGIN_NAME)==0)
         {
           if(GetArgv(string,TempStr,3))
               event->Par2=str2int(TempStr);

           if(GetArgv(string,TempStr,4))
               event->Par2|=str2int(TempStr)<<8;

           if(GetArgv(string,TempStr,5))
               event->Par2|=str2int(TempStr)<<16;

           event->Type = NODO_TYPE_PLUGIN_COMMAND;
           event->Command = PLUGIN_ID; // Plugin nummer  
           success=true;
         }
       }
     free(TempStr);
     break;
     }

   case PLUGIN_MMI_OUT:
     {
     strcpy(string,PLUGIN_NAME);            // Eerste argument=het commando deel
     strcat(string," ");
     strcat(string,cmd2str(event->Par1));
     strcat(string,",");
     strcat(string,int2str(event->Par2 & 0xff));
     strcat(string,",");
     strcat(string,int2str(event->Par2>>8 & 0xff));
     strcat(string,",");
     strcat(string,int2str(event->Par2>>16 & 0xffff));
     break;
     }
   #endif //NODO_MEGA

  }

  return success;
}

#define IP_BUFFER_SIZE            256

boolean P098SendCustomHTTPRequest(char* Request, byte ip, unsigned long port, byte varindex)
  {
  byte targetIP[4];
  int InByteCounter,x,y;
  byte InByte;
  unsigned long TimeoutTimer;
  const int TimeOut=5000;
  EthernetClient HTTPClient; // Client class voor HTTP sessie.
  byte State=0;
  char *IPBuffer=(char*)malloc(IP_BUFFER_SIZE+1);

  targetIP[0] = EthernetNodo.localIP()[0];
  targetIP[1] = EthernetNodo.localIP()[1];
  targetIP[2] = EthernetNodo.localIP()[2];
  targetIP[3] = ip;

  strcpy(IPBuffer,"GET ");

  // Alle spaties omzetten naar %20 en toevoegen aan de te verzenden regel.
  y=strlen(IPBuffer);
  for(x=0;x<strlen(Request);x++)
    {            
    if(Request[x]==32)
      {
      IPBuffer[y++]='%';
      IPBuffer[y++]='2';
      IPBuffer[y++]='0';
      }
    else
      {
      IPBuffer[y++]=Request[x];
      }
    }
  IPBuffer[y]=0;

  strcat(IPBuffer," HTTP/1.1");

  if(Settings.Debug==VALUE_ON)
    Serial.println(IPBuffer);

  if(HTTPClient.connect(targetIP,port))
    {
      HTTPClient.println(IPBuffer);
      HTTPClient.println("Host: 192.168.0.123");
      HTTPClient.println("User-Agent: Mozilla/5.0");
      HTTPClient.println(F("Connection: Close"));
      HTTPClient.println();// Afsluiten met een lege regel is verplicht in http protocol/

      TimeoutTimer=millis()+TimeOut; // Als er te lange tijd geen datatransport is, dan wordt aangenomen dat de verbinding (om wat voor reden dan ook) is afgebroken.
      IPBuffer[0]=0;
      InByteCounter=0;
      while(TimeoutTimer>millis() && HTTPClient.connected())
        {
        if(HTTPClient.available())
          {
            InByte=HTTPClient.read();
          
            if(isprint(InByte) && InByteCounter<IP_BUFFER_SIZE)
              IPBuffer[InByteCounter++]=InByte;

            else if(InByte==0x0A)
              {
                IPBuffer[InByteCounter]=0;
                // De regel is binnen

                if(Settings.Debug==VALUE_ON)
                  Serial.println(IPBuffer);

                String line = IPBuffer;

                if (varindex > 0 && line.substring(10, 14) == "Data")
                  {
                    String strValue = line.substring(19);
                    byte pos = strValue.indexOf(',')-1;
                    strValue = strValue.substring(0, pos);
                    strValue.trim();
                    char tmp[80];
                    strValue.toCharArray(tmp,79);
                    float value=0;

                    if (tmp[0]=='O')
                      value = (strValue == "On") ? 1 : 0;
                    else
                      value = atof(tmp);

                    UserVar[varindex-1] = value;
                    if(Settings.Debug==VALUE_ON)
                      {
                        Serial.println(strValue);
                        Serial.println("Succes!");
                      }
                  }

                InByteCounter=0;          
              }
          }
        }
      delay(500);
      HTTPClient.flush();// Verwijder eventuele rommel in de buffer.
      HTTPClient.stop();
    }
  free(IPBuffer);
  return State;
  }
