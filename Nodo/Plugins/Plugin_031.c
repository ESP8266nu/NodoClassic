//#######################################################################################################
//#################################### Plugin-031: SysLog ###############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor uitgaande communicatie via het UDP protocol op poort 514 t.b.v. syslog server
 * 
 * Auteur             : Martinus van den Broek
 * Support            : www.nodo-domotica.nl
 * Datum              : 12 Jul 2014
 * Versie             : BETA 0.3
 * Nodo productnummer : SWACDE-31-V10
 * Compatibiliteit    : Vanaf Nodo build nummer 744 (LET OP: DEZE PLUGIN WERKT ALLEEN OP EEN NODO MEGA!!!
 \*********************************************************************************************/

#define PLUGIN_ID 31
#define PLUGIN_NAME_031 "SysLog"

#include <Udp.h>

class EthernetUDP : public UDP {
private:
  uint8_t _sock;  // socket ID for Wiz5100
  uint16_t _port; // local port to listen on
  IPAddress _remoteIP; // remote IP address for the incoming packet whilst it's being processed
  uint16_t _remotePort; // remote port for the incoming packet whilst it's being processed
  uint16_t _offset; // offset into the packet being sent

public:
  EthernetUDP();  // Constructor
  virtual uint8_t begin(uint16_t);	// initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
  virtual void stop();  // Finish with the UDP socket
  virtual int beginPacket(IPAddress ip, uint16_t port);
  virtual int beginPacket(const char *host, uint16_t port);
  virtual int endPacket();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);
  using Print::write;
  virtual int parsePacket();
  virtual int available();
  virtual int read();
  virtual int read(unsigned char* buffer, size_t len);
  virtual int read(char* buffer, size_t len) { return read((unsigned char*)buffer, len); };
  virtual int peek();
  virtual void flush();	// Finish reading the current packet
  virtual IPAddress remoteIP() { return _remoteIP; };
  virtual uint16_t remotePort() { return _remotePort; };
};

EthernetUDP Udp;
byte Plugin_031_HostIP=0;

boolean Plugin_031(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_031_CORE

  case PLUGIN_EVENT_IN:
  case PLUGIN_EVENT_OUT:
    {
      if ((Plugin_031_HostIP > 0) && (!TerminalClient.connected())) // Log only if telnet is not connected or we might run short on W5100 sockets!
        {
          char* StringToPrint=(char*)malloc(100);
          char* TmpStr=(char*)malloc(INPUT_COMMAND_SIZE+1);

          StringToPrint[0]=0; // als start een lege string
  
          if(event->Direction)
            {
            strcat(StringToPrint,"<7>");
            strcat(StringToPrint,cmd2str(event->Direction));
            strcat(StringToPrint,"=");
  
            strcat(StringToPrint, cmd2str(event->Port));
            if(event->Port==VALUE_SOURCE_HTTP || event->Port==VALUE_SOURCE_TELNET)
              {
              strcat(StringToPrint, "(");
              strcat(StringToPrint, ip2str(ClientIPAddress));
              strcat(StringToPrint, ")");
              }
      
            if(event->Port==VALUE_SOURCE_EVENTLIST)
              {
              // print de nessting diepte van de eventlist en de regel uit de eventlist.
              strcat(StringToPrint, "(");
              strcat(StringToPrint, int2str(ExecutionDepth-1));
              strcat(StringToPrint, ".");
              strcat(StringToPrint, int2str(ExecutionLine));
              strcat(StringToPrint, ")");
              }
            strcat(StringToPrint, "; "); 
            }

          // Unit 
          strcat(StringToPrint, cmd2str(VALUE_UNIT));
          strcat(StringToPrint, "=");  
          if(event->Direction==VALUE_DIRECTION_OUTPUT && event->Port!=VALUE_SOURCE_HTTP)
            strcat(StringToPrint, int2str(event->DestinationUnit));
          else
            strcat(StringToPrint, int2str(event->SourceUnit)); 
 
          // Event
          strcat(StringToPrint, "; ");
          strcat(StringToPrint, ProgmemString(Text_14));
          Event2str(event,TmpStr);
          strcat(StringToPrint, TmpStr);

          IPAddress broadcastIP(EthernetNodo.localIP()[0],EthernetNodo.localIP()[1],EthernetNodo.localIP()[2], Plugin_031_HostIP);
          Udp.begin(514);
          Udp.beginPacket(broadcastIP, 514);
          Udp.write(StringToPrint);
          Udp.endPacket();
          Udp.stop();
          free(TmpStr);
          free(StringToPrint);
        }
      success=true;
      break;
    } // case

  case PLUGIN_COMMAND:
    {
      Plugin_031_HostIP=event->Par1;
      success=true;
      break;
    } // case

#endif // PLUGIN_031_CORE
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);
      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_031)==0)
          {
          if(GetArgv(string,TempStr,2)) 
            {
                event->Type = NODO_TYPE_PLUGIN_COMMAND;
                event->Command = PLUGIN_ID; // Plugin nummer  
                success=true;
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME_031);
      strcat(string," ");
      strcat(string,int2str(event->Par1));
      break;
      }
    #endif //MMI
  }      
  return success;
}

