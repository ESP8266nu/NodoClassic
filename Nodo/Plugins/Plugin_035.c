//#######################################################################################################
//##################################### Plugin-035 Ping #################################################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides support for ICMP check
 * 
 * Author             : Martinus van den Broek
 * Support            : None!
 * Date               : Dec 2014
 * Versie             : 0.2
 * Compatibility      : R744
 * Syntax             : "Ping <result var>, <ip address>"
 * Sample             : Ping 10,8.8.8.8
 *                      this will set Nodo variable 10 to '1' if the target 8.8.8.8 is alive
 *********************************************************************************************
 * Technical data:
 * This plugin sends a simple single ICMP request with 64 bytes of null data
 * If the target host replies in any way, chosen result var will be set to '1'
 * ICMP is implemented here in a bare minimum approach, just to save code...
 \*********************************************************************************************/
byte Plugin_035_GetArgv(char *string, char *argv, int argc, char delimiter);

#define PLUGIN_ID 35
#define PLUGIN_NAME "Ping"

#define PING_TIMEOUT 1000

#include <utility/w5100.h>

boolean Plugin_035(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_035_CORE
  case PLUGIN_COMMAND:
    {
      // find available W5100 socket
      byte i=0;
      for (i = 0; i < MAX_SOCK_NUM; i++)
        {
          uint8_t s = W5100.readSnSR(i);
          if (s == SnSR::CLOSED || s == SnSR::FIN_WAIT)
            break;
        }

      if (!TerminalClient.connected())  // Ping only if telnet is not connected or we might run short on W5100 sockets!
        {
          Serial.println("Pinging...");

          UserVar[event->Par1 - 1] = 0;

          uint8_t IPaddress[4];
          IPaddress[0] = (event->Par2 >>24) & 0xff;
          IPaddress[1] = (event->Par2 >>16) & 0xff;
          IPaddress[2] = (event->Par2 >> 8) & 0xff;
          IPaddress[3] = event->Par2 & 0xff;

          SOCKET _icmpsocket=i;

          // init
          W5100.execCmdSn(_icmpsocket, Sock_CLOSE);
          W5100.writeSnIR(_icmpsocket, 0xFF);
          W5100.writeSnMR(_icmpsocket, SnMR::IPRAW);
          W5100.writeSnPROTO(_icmpsocket, IPPROTO::ICMP);
          W5100.writeSnPORT(_icmpsocket, 0);
          W5100.execCmdSn(_icmpsocket, Sock_OPEN);
          W5100.writeSnDIPR(_icmpsocket, IPaddress);
          W5100.writeSnTTL(_icmpsocket, 42);
          W5100.writeSnDPORT(_icmpsocket, 0);

          // prepare packet
          uint8_t packet[76];
          for (byte x=0; x < 76; x++) packet[x]=0;

          packet[0]=8;   // ICMP Request
          packet[2]=247; // (sum >> 8) & 0xff
          packet[3]=255; // sum & 0xff

          W5100.send_data_processing(_icmpsocket, packet, 76);
          W5100.execCmdSn(_icmpsocket, Sock_SEND);
          W5100.writeSnIR(_icmpsocket, SnIR::SEND_OK);

          // wait for reply
          Serial.println("Waiting...");
          unsigned long start = millis();
          byte octet_matched = 0;

          while (millis() - start < PING_TIMEOUT)
          {
            if (W5100.getRXReceivedSize(_icmpsocket))
              {
                  uint8_t ipHeader [6];
                  uint8_t buffer = W5100.readSnRX_RD(_icmpsocket);
                  W5100.read_data(_icmpsocket, (uint8_t *)buffer, ipHeader, sizeof(ipHeader));

                  // check peer IP, must be same as requested
                  for (int x=0; x<4; x++)
                    if (IPaddress[x] == ipHeader[x])
                      octet_matched++;

                  break;
              }
          }
 
          if (octet_matched  == 4)
            {
              Serial.println("OK");
              UserVar[event->Par1 - 1] = 1;
            }
          else 
            Serial.println("Not OK");
  
          // close socket
          W5100.execCmdSn(_icmpsocket, Sock_CLOSE);
          W5100.writeSnIR(_icmpsocket, 0xFF);
        } // endif terminalsession active

      success = true;
      break;
    }
#endif // PLUGIN_035_CORE

#if NODO_MEGA
    case PLUGIN_MMI_IN:
      {

      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
          if(GetArgv(string,TempStr,2)) 
            {
            event->Par1=str2int(TempStr);
            if(Plugin_035_GetArgv(string,TempStr,3,'.'))
              {
              event->Par2=str2int(TempStr)<<24;
              if(Plugin_035_GetArgv(string,TempStr,4,'.'))
                {
                event->Par2|=str2int(TempStr)<<16;
                if(Plugin_035_GetArgv(string,TempStr,5,'.'))
                  {
                  event->Par2|=str2int(TempStr)<<8;
                  if(Plugin_035_GetArgv(string,TempStr,6,'.'))
                    {
                      event->Par2|=str2int(TempStr);
                      event->Type    = NODO_TYPE_PLUGIN_COMMAND;
                      event->Command = PLUGIN_ID; // nummer van deze plugin
                      success=true;
                    }
                  }
                }
              }
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME);               // Commando 
      strcat(string," ");
      strcat(string,int2str(event->Par1));
      strcat(string,",");
      strcat(string,int2str((event->Par2 >>24)&0xff));
      strcat(string,".");
      strcat(string,int2str((event->Par2 >>16)&0xff));
      strcat(string,".");
      strcat(string,int2str((event->Par2 >>8)&0xff));
      strcat(string,".");
      strcat(string,int2str((event->Par2 )&0xff));

      break;
      }
#endif //NODO_MEGA
  }      
  return success;
}

#ifdef PLUGIN_035_CORE
byte Plugin_035_GetArgv(char *string, char *argv, int argc, char delimiter)
{
  int string_pos=0,argv_pos=0,argc_pos=0; 
  char c,d;

  while(string_pos<strlen(string))
  {
    c=string[string_pos];
    d=string[string_pos+1];

    if       (c==' ' && d==' '){}
    else if  (c==' ' && d==','){}
    else if  (c==',' && d==' '){}
    else if  (c==' ' && d>=33 && d<=126){}
    else if  (c==',' && d>=33 && d<=126){}
    else 
      {
      if(c!=' ' && c!=delimiter && c!=',')
        {
        argv[argv_pos++]=c;
        argv[argv_pos]=0;
        }          

      if(d==' ' || d==delimiter || d==0 || d==',')
        {
        // Bezig met toevoegen van tekens aan een argument, maar er kwam een scheidingsteken.
        argv[argv_pos]=0;
        argc_pos++;

        if(argc_pos==argc)
          return string_pos+1;
          
        argv[0]=0;
        argv_pos=0;
        string_pos++;
      }
    }
    string_pos++;
  }
  return 0;
}
#endif
