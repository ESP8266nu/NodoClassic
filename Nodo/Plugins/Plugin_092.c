//#######################################################################################################
//#################################### Plugin-092: Wiznet 5100 UDP Link #################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor communicatie via het UDP protocol, Wiznet 5100
 * 
 * Auteur             : M vd Broek
 * Datum              : 21 jan 2016
 * Versie             : Beta 0.1
 * Compatibiliteit    : Vanaf Nodo build nummer 787 (NIET GESCHIKT VOOR Nodo 3.8 !)
 *
 *                      (LET OP: DEZE PLUGIN WERKT ALLEEN OP EEN NODO MEGA!!!
 *
 * Instructie         : Zet de volgende regels in het config bestand
 *
 * 				#define NODO_BETA_PLUGIN_SENDTO
 *				#define PLUGIN_092
 *				#define PLUGIN_092_CORE
 *                              #define PLUGIN_092_UDPPORT 65500
 *
 *                      We gaan uit van UDP port 65500 maar dan moet dit ook zo zijn ingesteld op de ESP!
 *			De plugin heeft verder geen configuratie nodig
 *                      dus alleen meecompileren is voldoende!
 *
 *                      Plugin geeft conflicten indien ook http en telnet wordt gebruikt!
 \*********************************************************************************************/

#define PLUGIN_ID 92
#define PLUGIN_NAME_092 "ESP"

#define UDP_TX_PACKET_MAX_SIZE 80
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
byte packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

struct NodeStruct
{
  byte ip[4];
  byte age;
} Nodes[UNIT_MAX+1];


boolean Plugin_092(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;
  static byte unicastTargetUnit = 0;
  static byte timer = 0;

  switch(function)
  {
#ifdef PLUGIN_092_CORE
    case PLUGIN_INIT:
      {
        for (byte x=0; x < 32; x++)
          Nodes[x].ip[0]=0;

        Udp.begin(PLUGIN_092_UDPPORT);
        break;
      }

    case PLUGIN_ONCE_A_SECOND:
      {
        timer++;
        if (timer > 30)
        {
          timer=0;
          byte data[20];
          data[0] = 255;
          data[1] = 1;
          for (byte x = 0; x < 4; x++)
            data[x + 8] = EthernetNodo.localIP()[x];
          data[12] = Settings.Unit;
          IPAddress broadcastIP(255, 255, 255, 255);
          Udp.beginPacket(broadcastIP, PLUGIN_092_UDPPORT);
          Udp.write(data, 20);
          Udp.endPacket();

          // refresh list
          for (byte counter = 0; counter < UNIT_MAX; counter++)
            {
              if (Nodes[counter].ip[0] != 0)
                {
                  Nodes[counter].age++;  // increment age counter
                  if (Nodes[counter].age > 10) // if entry to old, clear this node ip from the list.
                    for (byte x = 0; x < 4; x++)
                      Nodes[counter].ip[x] = 0;
                }
            }
        }
        break;
      }

  case PLUGIN_SCAN_EVENT:
    {
      int packetSize = Udp.parsePacket();
      if(packetSize)
      {
        IPAddress remoteIP = Udp.remoteIP();
        byte remoteOctet = remoteIP[3];
        Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
        if (packetBuffer[0] == 255 && packetBuffer[1] == 254)
          {
            memcpy((byte*)event, (byte*)&packetBuffer+2, sizeof(struct NodoEventStruct));
            if(Checksum(event))
              {

                if (event->SourceUnit < UNIT_MAX)
                {
                  for (byte x = 0; x < 4; x++)
                    Nodes[event->SourceUnit].ip[x] = remoteIP[x];
                  Nodes[event->SourceUnit].age = 0; // reset 'age counter'
                }

                if (event->Flags == 0) // normal traffic, Nodo flags are 0
                 {
                   unicastTargetUnit = 0;
                 }
               else
                 {
                   unicastTargetUnit = event->SourceUnit; // remote source Nodo unit will be unicast target
                 }

                event->Direction = VALUE_DIRECTION_INPUT;
                event->Port      = VALUE_SOURCE_RF;
                success = true;
              }
            else
              Serial.println("CRC!");
          }
      }
      break;
    } // case

  case PLUGIN_EVENT_OUT:
    {
    // Only send messages for port RF or ALL
    Checksum(event);
    if ((event->Port==VALUE_SOURCE_RF) || (event->Port==VALUE_ALL))
      {
        byte cmd[sizeof(struct NodoEventStruct)+2];
        cmd[0]=255;
        cmd[1]=254;
        memcpy((byte*)&cmd+2, (byte*)event,sizeof(struct NodoEventStruct));

        IPAddress sendIP(255, 255, 255, 255);

        if (event->Flags != 0) // flags set, set to unicast mode
          {
            if ((unicastTargetUnit != 0) && (Nodes[unicastTargetUnit].ip[0] != 0))
              for(byte x=0; x <4; x++)
                sendIP[x] = Nodes[unicastTargetUnit].ip[x];
          }

        Udp.beginPacket(sendIP, PLUGIN_092_UDPPORT);
        Udp.write(cmd,16);
        Udp.endPacket();
        delay(50); // wiznet must not send too fast, ESP (and serial buffer behind..) will not keep up!
      }
      success=true;
      break;
    }

    case PLUGIN_COMMAND:
      {
				  		// Par 1 = command
        byte Par2=event->Par2 & 0xff;    	// Par 2 =  
        byte Par3=event->Par2>>8 & 0xff;	// Par 3 =  
        int Par4=event->Par2>>16 & 0xffff;	// Par 4 =  

        if (event->Par1 == CMD_NODO_IP)
          {
           unicastTargetUnit = event->Par2;
          }

        if (event->Par1 == VALUE_UNIT)
          {
            for (byte x=0; x < UNIT_MAX+1; x++)
              {
                if (Nodes[x].ip[0] != 0)
                  {
                    Serial.print(x);
                    Serial.print(" - ");
                    for(byte y=0; y < 4; y++)
                      {
                        Serial.print(Nodes[x].ip[y]);
                        if (y < 3)
                          Serial.print(".");
                      }
                    Serial.println();
                  }
              } 
          }

        success=true;
        break;
      } // case COMMAND

#endif // PLUGIN_092_CORE
  
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char* TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_092)==0)
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
      strcpy(string,PLUGIN_NAME_092);
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
    #endif //MMI
  }

  return success;
}
