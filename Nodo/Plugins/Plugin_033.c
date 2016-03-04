//#######################################################################################################
//#################################### Plugin-033: NRF24L01 plugin ######################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor communicatie met de NRF24L01 2.4 GHz Transceiver
 * 
 * Auteur             : Martinus van den Broek
 * Support            : Beta !
 * Datum              : 28 Jan 2015 (learn mechanisme aangepast)
 * Versie             : 0.15
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 765 voor Sendto (!)
 *
 * 			en deze regel moet zijn opgenomen in de unit config file:
 *				#define NODO_BETA_PLUGIN_SENDTO
 *
 * Syntax             : De plugin kan worden gebruikt zonder commando's!
 *                      
 *                      De volgende commando's zijn nodig voor batterij gebruik
 *                      waarbij de NRF wordt uitgeschakeld tijdens Nodo sleep mode
 *                      Na het inschakelen:
 *                        nrf reset		init device
 *                      Voor het uitschakelen:
 *                        nrf off		turn off device
 *
 *                      Alleen op de Mega:
 *                        nrf rf,<channel>	set RF channel (1-62)
 *                        nrf status,<unit>	check connectivity/retries
 \*********************************************************************************************/
#ifndef NRF_DEBUG
  #define NRF_DEBUG false
#endif

#ifndef NRF_CSN_PIN
  #if NODO_MEGA
    #define NRF_CSN_PIN  A12
    #define NRF_MOSI_PIN A13
    #define NRF_MISO_PIN A14
    #define NRF_SCK_PIN  A15
  #else
    #define NRF_CSN_PIN  A0
    #define NRF_MOSI_PIN A1
    #define NRF_MISO_PIN A2
    #define NRF_SCK_PIN  A3
  #endif
#endif

#ifndef NRF_FEATURE_ONOFF
  #define NRF_FEATURE_ONOFF false
#endif

#ifndef NRF_FEATURE_ROUTER
  #define NRF_FEATURE_ROUTER false
#endif

#ifndef NRF_FEATURE_NODE_DISCOVERY
  #define NRF_FEATURE_NODE_DISCOVERY false
#endif

#ifndef NRF_FEATURE_FINDMASTER
  #define NRF_FEATURE_FINDMASTER false
#endif

#ifndef NRF_FEATURE_CHANNELBROADCAST
  #define NRF_FEATURE_CHANNELBROADCAST false
#endif

#ifndef NRF_ADDRESS
  #define NRF_ADDRESS               2,3,4,5 // last 4 bytes of address, 1st byte is controlled by plugin
#endif
#ifndef NRF_CHANNEL
  #define NRF_CHANNEL               36 // Mega defaults to channel 36
#endif

#ifdef NRF_HWSPI		// test purposes for std hardware, not officially supported !
  #if NODO_MEGA
    #define NRF_CSN_PIN  40
    #define NRF_CE_PIN   41
  #else
    #define NRF_CSN_PIN   7
    #define NRF_CE_PIN    8
  #endif
#endif

#define PLUGIN_ID 33
#define PLUGIN_NAME_033 "NRF"

#define NRF_PAYLOAD_SIZE	  32
#define NRF_UNIT_MAX              31
#define NRF_SEND_RETRIES           3
#define NRF_RETRY_DELAYMS         50
#define NRF_SEND_TIMEOUTMS       150

#define NRF_PAYLOAD_NODO           0
#define NRF_PAYLOAD_SENSOR         1
#define NRF_PAYLOAD_EVENT          2
#define NRF_PAYLOAD_TEXT           3
#define NRF_PAYLOAD_ICMP           4
#define NRF_PAYLOAD_SYSTEM       255

#define NRF_SYSTEMCMD_CHECKONLINE  1
#define NRF_SYSTEMCMD_CHANNEL      2
#define NRF_SYSTEMCMD_OFFLINE      3
#define NRF_SYSTEMCMD_FINDMASTER   4

#define NRF_BROADCAST_ADDRESS_ACK 254
#define NRF_BROADCAST_ADDRESS     255

#ifndef NRF_DEFAULT_DESTINATION
  #define NRF_DEFAULT_DESTINATION  0
#endif

#define NRF_EN_AA       0x01		// Register for Enable Auto Acknowledge configuration register
#define NRF_RF_SETUP    0x06		// Register for Set Data Rate
#define NRF_SETUP_RETR  0x04		// Register for Retry delay and count
#define NRF_EN_RXADDR   0x02		// Register for RX enable receive pipes
#define NRF_EN_RXADDR   0x02
#define NRF_FEATURE     0x1D
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14

#ifdef PLUGIN_033_CORE

#define NRF_ADDR_LEN	5
#define NRF_CONFIG_DATA ((1<<EN_CRC) | (0<<CRCO) )

// NRF function prototypes
void Nrf24_setChannel(byte channel);
void Nrf24_init();
void Nrf24_config();
byte Nrf24_send(uint8_t *value);
void Nrf24_setRADDR(uint8_t * adr);
void Nrf24_setTADDR(uint8_t * adr);
bool Nrf24_dataReady();
bool Nrf24_rxFifoEmpty();
bool Nrf24_txFifoEmpty();
void Nrf24_getData(uint8_t * data);
uint8_t Nrf24_getStatus();
void Nrf24_transmitSync(uint8_t *dataout,uint8_t len);
void Nrf24_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len);
void Nrf24_configRegister(uint8_t reg, uint8_t value);
void Nrf24_readRegister(uint8_t reg, uint8_t * value, uint8_t len);
void Nrf24_writeRegister(uint8_t reg, uint8_t * value, uint8_t len);
void Nrf24_powerUpRx();
void Nrf24_powerUpTx();
void Nrf24_powerDown();
void Nrf24_csnHi();
void Nrf24_csnLow();
void Nrf24_flushRx();
void Nrf24_flushTx();

// function prototypes
boolean NRF_init(void);
boolean NRF_receive(struct NodoEventStruct *event);
byte NRF_findMaster();
void NRF_changeChannel(byte channel);
void NRF_CheckOnline();
byte NRF_sendpacket(byte Source, byte Destination, byte ID, byte Size, byte Sequence, byte Flags, byte retries);
byte NRF_Random();
void SPI_begin();
unsigned char SPI_transfer(unsigned char Byte);

#endif // plugin core

#ifndef NRF_BARE_MINIMUM

byte NRFOnline[NRF_UNIT_MAX+1];
byte NRFSequences[NRF_UNIT_MAX+1];
byte NRFSequence=0;
byte NRF_single_target=0;
#if NRF_FEATURE_ROUTER
  byte NRF_forceSingleTarget=0;
#endif
#if NRF_DEBUG
  boolean NRF_debug = true;
#endif
uint8_t NRF_channel=NRF_CHANNEL;
byte NRF_address[5] = { 1,NRF_ADDRESS };
boolean NRF_live=false;

struct NRFPayloadStruct
{
  byte Source;
  byte Destination;
  byte ID;
  byte Size;
  byte Sequence;
  byte Flags;
  byte Gateway;
  byte Future;
  byte Data[24];
}
NRFPayload;

boolean Plugin_033(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_033_CORE
    case PLUGIN_INIT:
      {
        NRFSequence=NRF_Random();
        #if NRF_DEBUG
          if (NRF_debug)
            {
              Serial.print("S ");
              Serial.println(NRFSequence);
            }
        #endif
        NRF_live=NRF_init();
        #if NRF_FEATURE_FINDMASTER
          if (NRF_channel==0) NRF_findMaster();
        #endif
        #if NRF_DEFAULT_DESTINATION == 0
          NRF_CheckOnline();
        #endif
        break;
      }

#ifdef NODO_BETA_PLUGIN_SENDTO
  case PLUGIN_SCAN_EVENT:
    {
       success=NRF_receive(event);
      break;
    } // case PLUGIN_SCAN_EVENT
#else
  case PLUGIN_ONCE_A_SECOND:
    {
      while(NRF_live && Nrf24_dataReady())
        {
          Nrf24_getData((byte *)&NRFPayload);

          switch(NRFPayload.ID)
            {

            case NRF_PAYLOAD_NODO:
            {
              struct NodoEventStruct TempEvent;
              memcpy((byte*)&TempEvent, (byte*)&NRFPayload+8,sizeof(struct NodoEventStruct));
              TempEvent.Direction = VALUE_DIRECTION_INPUT;
              TempEvent.Port      = VALUE_SOURCE_RF;
              NRFOnline[event->SourceUnit]=event->SourceUnit;
              ProcessEvent(&TempEvent);
              success=true;
              break;
            }

          }
        }
      break;
    } // case PLUGIN_ONCE_A_SECOND
#endif

  case PLUGIN_COMMAND:
    {

      #if NRF_DEBUG // test stuff, not for production!
      if (event->Par1 == CMD_SUBNET)      
          Nrf24_configRegister(NRF_EN_AA, event->Par2);

      if (event->Par1 == CMD_DELAY)      
        Nrf24_configRegister(NRF_SETUP_RETR, event->Par2);

      if (event->Par1 == CMD_OUTPUT)
        Nrf24_configRegister(NRF_RF_SETUP, event->Par2);
      #endif

      #if NRF_FEATURE_ROUTER
      // manipulate routing table manually
      if (event->Par1 < 32)      
        {
          NRFOnline[event->Par1]=event->Par2;
          #if NRF_DEBUG
            if (NRF_debug)
              {
                Serial.println("route rule");
                for (byte y=1; y < 32; y++)
                  {
                    Serial.print((int)y);
                    Serial.print("-");
                    Serial.println(NRFOnline[y]);
                  }
              }
          #endif
        }
      #endif

      #if NRF_DEBUG
      if (event->Par1 == CMD_DEBUG)
        {
          if (event->Par2 == VALUE_OFF) NRF_debug = false;
          if (event->Par2 == VALUE_ON) NRF_debug = true;
          if (event->Par2 == 1)
            {
              unsigned long duur=micros();
              if(Nrf24_dataReady()) {}
              Serial.println(micros()-duur);
            }
          if (event->Par2 == 2)
            {
              unsigned long duur=micros();
              SPI_transfer(255);
              Serial.println(micros()-duur);
            }
        }
      #endif

      #if NRF_FEATURE_ONOFF
      if (event->Par1 == CMD_RESET)
        {
          NRF_live=NRF_init();
          #if NRF_FEATURE_FINDMASTER
            if (NRF_channel==0) NRF_findMaster();
          #endif
          #if NRF_DEFAULT_DESTINATION == 0
            NRF_CheckOnline();
          #endif
        }

      if (event->Par1 == VALUE_OFF)
        {
          NRFSequence++;
          NRFPayload.Data[0]=NRF_SYSTEMCMD_OFFLINE;
          NRF_sendpacket(Settings.Unit, 0, NRF_PAYLOAD_SYSTEM, 1 ,NRFSequence, 0, NRF_SEND_RETRIES);
          NRF_live=false;
        }
      #endif

      if (event->Par1 == VALUE_SOURCE_RF)
        {
          #if NRF_FEATURE_FINDMASTER
            if (event->Par2 == 0) NRF_findMaster();
          #endif
          if ((event->Par2 >= 1) && (event->Par2 <= 62))  // Select channel between 1 <> 62
            {
              NRF_channel = event->Par2;
              #if NODO_MEGA
                #if NRF_FEATURE_CHANNELBROADCAST
                  NRF_changeChannel(NRF_channel);     // broadcast change to all channels!
                #endif
              #endif 
              Nrf24_setChannel(NRF_channel);
              #if NRF_DEFAULT_DESTINATION == 0
                NRF_CheckOnline();                      // Send data on air to find out who's online
              #endif 
            }
        }

      #if NODO_MEGA
      if (event->Par1 == CMD_STATUS)
        {
           unsigned long NRF_roundtrip=millis();
           NRFOnline[0]=0;
           byte NRF_status= NRF_sendpacket(Settings.Unit, event->Par2, NRF_PAYLOAD_ICMP, 0,NRFSequence++,0,0);
           NRFOnline[0]=1;
           Serial.print("status ");
           Serial.println((int)NRF_status);
           Serial.print("roundtrip ");
           Serial.println(millis()-NRF_roundtrip);
           byte NRF_retries=0;
           Nrf24_readRegister(8, &NRF_retries, sizeof(NRF_retries) );
           Serial.print("retries ");
           Serial.println((int)(NRF_retries & 0x0f));
        }
      #endif

      #if NRF_FEATURE_ROUTER
      if (event->Par1 == CMD_SENDTO)
        {
              NRF_forceSingleTarget = event->Par2;
        }
      #endif

      success=true;
      break;
    } // case


  case PLUGIN_EVENT_OUT:
    {
    // Only send messages for port RF or ALL and only send Nodo messages (events)
    // This will process commands like "UserEventSend" and "VariableSend" 

#ifdef NODO_BETA_PLUGIN_SENDTO
    if(NRF_live && ((event->Port==VALUE_SOURCE_RF) || (event->Port==VALUE_ALL)))
#else
    if(NRF_live && ((event->Port==VALUE_SOURCE_RF) || (event->Port==VALUE_ALL)) && ((event->Type==NODO_TYPE_EVENT) || (event->Type==NODO_TYPE_PLUGIN_EVENT)))
#endif
      {
        // on the receiving end, this event may be passed through by NRFExtender on the I2C bus, so it needs a checksum
        Checksum(event);// bereken checksum: crc-8 uit alle bytes in de struct.
        memcpy((byte*)&NRFPayload+8, (byte*)event,sizeof(struct NodoEventStruct));

        #if NODO_MEGA  // Sendto optimalisatie
          if ((event->Type == NODO_TYPE_COMMAND) && (event->Command == CMD_EVENTLIST_SHOW))
              NRF_single_target = event->DestinationUnit;
          if ((event->Flags != 0) && (event->DestinationUnit != 0))
              NRF_single_target = event->DestinationUnit;
        #endif

        // reset single target if flags are 0
        if (event->Flags ==0)
          {
            NRF_single_target = 0;
            #if NRF_DEBUG
              if (NRF_debug)
                {
                  Serial.print("So ");
                  Serial.println((int)NRF_single_target);
                }
            #endif
          }

        #if NRF_DEBUG
          if (NRF_debug)
            {
              Serial.print(">T ");
              Serial.print((int)event->Type);
              Serial.print(" S ");
              Serial.print((int)event->SourceUnit);
              Serial.print(" D ");
              Serial.print((int)event->DestinationUnit);
              Serial.print(" C ");
              Serial.print((int)event->Command);
              Serial.print(" F ");
              Serial.print((int)event->Flags);
              Serial.print(" ST ");
              Serial.println((int)NRF_single_target);
            }
        #endif

        NRFSequence++;
        NRF_sendpacket(Settings.Unit, NRF_DEFAULT_DESTINATION, NRF_PAYLOAD_NODO, sizeof(struct NodoEventStruct),NRFSequence, 0, NRF_SEND_RETRIES);
      }
      success=true;
      break;
    }

#endif // PLUGIN_033_CORE
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(26);
      string[25]=0;

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_033)==0)
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
      strcpy(string,PLUGIN_NAME_033);
      strcat(string," ");
      strcat(string,cmd2str(event->Par1));
      strcat(string,",");
      strcat(string,int2str(event->Par2));
      break;
      }
    #endif //MMI
  }      
  return success;
}

#ifdef PLUGIN_033_CORE

boolean NRF_receive(struct NodoEventStruct *event)
{
  boolean success=false;

      if(NRF_live && Nrf24_dataReady())
        {
          Nrf24_getData((byte *)&NRFPayload);

          #if NRF_DEBUG
            if (NRF_debug)
              {
                Serial.print("NRF");
                Serial.print(" S ");
                Serial.print(NRFPayload.Source);
                Serial.print(" D ");
                Serial.print(NRFPayload.Destination);
                Serial.print(" Sq ");
                Serial.print(NRFPayload.Sequence);
                Serial.print(" F");
                Serial.print(NRFPayload.Flags);
                Serial.print(" G ");
                Serial.print(NRFPayload.Gateway);
                Serial.print(" Id ");
                Serial.println(NRFPayload.ID);
              }
          #endif

          #if NRF_FEATURE_ROUTER
          // static routing
          if (NRFPayload.Destination != NRF_BROADCAST_ADDRESS_ACK && NRFPayload.Destination != NRF_BROADCAST_ADDRESS && NRFPayload.Destination != Settings.Unit && (NRFPayload.Flags & 0x3) < 3)
            {
              #if NRF_DEBUG
                if (NRF_debug)
                  {
                    Serial.println("Routing!");
                  }
              #endif
              NRF_sendpacket(NRFPayload.Source, NRFPayload.Destination, NRFPayload.ID, NRFPayload.Size, NRFPayload.Sequence, ++NRFPayload.Flags, NRF_SEND_RETRIES);
            }
          else // not direct routing 
          #endif

          // check duplicate packets
          if (NRFPayload.Sequence == NRFSequences[NRFPayload.Source])
            {
              #if NRF_DEBUG
                if (NRF_debug)
                  Serial.println("Dup!");
              #endif
            }
          else 
            {

// testmode block, to simulate unreachable nodo's in testlab, remove before production!
#ifdef NRF_TESTBLOCK1
  boolean blocked=false;
  if ((NRFPayload.Source==NRF_TESTBLOCK1 || NRFPayload.Source==NRF_TESTBLOCK2 || NRFPayload.Source==NRF_TESTBLOCK3) && (NRFPayload.Flags & 0x3)==0)
    {
      Serial.println("In Blocked1");
      blocked=true;
      NRFPayload.ID=123;
      NRFPayload.Flags=1;
      NRFPayload.Source=0;
      NRFPayload.Destination=Settings.Unit;
    }
  if ((NRFPayload.Gateway==NRF_TESTBLOCK1 || NRFPayload.Gateway==NRF_TESTBLOCK2 || NRFPayload.Gateway==NRF_TESTBLOCK3))
    {
      Serial.println("In Blocked2");
      blocked=true;
      NRFPayload.ID=123;
      NRFPayload.Flags=1;
      NRFPayload.Source=0;
      NRFPayload.Destination=Settings.Unit;
    }
if (!blocked)
#endif
            if(NRFPayload.Source!=Settings.Unit && NRFOnline[NRFPayload.Source]==0)
            {
              #if NRF_FEATURE_NODE_DISCOVERY
                NRFOnline[NRFPayload.Source]=NRFPayload.Gateway;
                #if NRF_DEBUG
                if (NRF_debug)
                  {
                    Serial.print("Learned ");
                    Serial.print((int)NRFPayload.Source);
                    Serial.print(" using ");
                    Serial.println((int)NRFPayload.Gateway);
                  }
                #endif
              #else
                // accept only direct targets
                if (NRFPayload.Source==NRFPayload.Gateway)
                  {
                    NRFOnline[NRFPayload.Source]=NRFPayload.Gateway;
                    #if NRF_DEBUG
                      if (NRF_debug)
                        {
                          Serial.print("Learned ");
                          Serial.println((int)NRFPayload.Source);
                        }
                    #endif
                  }
              #endif
            }

            switch(NRFPayload.ID)
            {
              case NRF_PAYLOAD_NODO:
              {
                memcpy((byte*)event, (byte*)&NRFPayload+8,sizeof(struct NodoEventStruct));
                event->Direction = VALUE_DIRECTION_INPUT;
                event->Port      = VALUE_SOURCE_RF;
  
                #if NRF_DEBUG
                  if (NRF_debug)
                    {
                      Serial.print("<T ");
                      Serial.print((int)event->Type);
                      Serial.print(" S ");
                      Serial.print((int)event->SourceUnit);
                      Serial.print(" D ");
                      Serial.print((int)event->DestinationUnit);
                      Serial.print(" C ");
                      Serial.print((int)event->Command);
                      Serial.print(" F ");
                      Serial.print((int)event->Flags);
                      Serial.print(" ST ");
                      Serial.println((int)NRF_single_target);
                    }
                #endif
 
                // this code is to speed up sendto eventlistshow
                // remember peer node during sendto 
                if ((event->Type == NODO_TYPE_SYSTEM) && (event->Command == SYSTEM_COMMAND_QUEUE_SENDTO) && (event->DestinationUnit == Settings.Unit))
                  {
                    NRF_single_target = event->SourceUnit;
                    #if NRF_DEBUG
                      if (NRF_debug)
                        {
                          Serial.print("SQ to ");
                          Serial.println((int)NRF_single_target);
                        }
                    #endif
                  }
  
                success=true;
                break;
              }

              case NRF_PAYLOAD_SYSTEM:
              {
                #if NRF_FEATURE_CHANNELBROADCAST
                if (NRFPayload.Data[0] == NRF_SYSTEMCMD_CHANNEL)
                  {
                    NRF_channel = NRFPayload.Data[1];
                    Nrf24_setChannel(NRF_channel);
                    #if NRF_DEBUG
                      if (NRF_debug)
                        {
                          Serial.println(NRF_channel);
                        }
                    #endif
                    for (byte x=1; x < Settings.Unit+2; x++)
                      {
                        #if NRF_DEBUG
                          if (NRF_debug)
                            {
                              Serial.print("Wait:");
                              Serial.print(x);
                              Serial.print('/');
                              Serial.println(Settings.Unit+2);
                             }
                        #endif
                        delay(3000);
                        Nrf24_flushRx();
                      }
                    #if NRF_DEFAULT_DESTINATION == 0
                      NRF_CheckOnline();
                    #endif
                  }
                #endif

                #if NRF_FEATURE_ONOFF
                if (NRFPayload.Data[0] == NRF_SYSTEMCMD_OFFLINE)
                  {
                    #if NRF_DEBUG
                    if (NRF_debug)
                      Serial.println("Offline");
                    #endif
                    NRFOnline[NRFPayload.Source]=0;
                  }
                #endif

                break;
              }

              default:
              {
                // must be some other packet, just create log event within Nodo
                // other plugins can check this event and use NRFPayload for processing
                ClearEvent(event); 
                event->Type=NODO_TYPE_PLUGIN_EVENT;
                event->Command=PLUGIN_ID;
                event->Par1=71;
                event->Par2=NRFPayload.ID;
                success=true;
              }

            } // switch payload id

            #if NRF_FEATURE_NODE_DISCOVERY
            // re-broadcast routing message (system message only)
            if(NRFPayload.ID==NRF_PAYLOAD_SYSTEM && NRFPayload.Destination==NRF_BROADCAST_ADDRESS && NRFPayload.Source != Settings.Unit && (NRFPayload.Flags & 0x3) < 3)
              {
                delay(100+10*Settings.Unit);
                #if NRF_DEBUG
                if (NRF_debug)
                  Serial.println("Repeat BRC!");
                #endif
                byte gw=NRFPayload.Gateway; // store this, it will be changed after next send, need it later...
                NRF_sendpacket(NRFPayload.Source, NRFPayload.Destination, NRFPayload.ID, NRFPayload.Size, NRFPayload.Sequence, ++NRFPayload.Flags, 0);

                // should reply to source if this is a indirect node. source does not know remote nodo's at boot time...
                // save to do it here ? sequence at source for this unit at boot is 0.
                // maybe safe because this is the first packet and cannot be a duplicate
                if (NRFPayload.Source != gw)
                  {
                    #if NRF_DEBUG
                    if (NRF_debug)
                      Serial.println("Reply to boot!");
                    #endif
                    NRF_sendpacket(Settings.Unit, NRFPayload.Source, 123, 0, NRFPayload.Sequence, 0, 0);
                  }
              } // re-broadcast
              #endif

// testmode block, to simulate unreachable nodo's in testlab, remove before production!
#ifdef NRF_TESTBLOCK1
  if (!blocked)
#endif
//
          NRFSequences[NRFPayload.Source]=NRFPayload.Sequence;

          } // if not duplicate
        }
  return success;
}


boolean NRF_init(void)
{
  SPI_begin();
  Nrf24_init();

  Nrf24_configRegister(NRF_RF_SETUP, 0x27);  // 250 kbps (bit 0 set for SI24R1 clone)

  byte check=0;
  Nrf24_readRegister(NRF_RF_SETUP,&check,1);
  #if NRF_DEBUG
    if (NRF_debug)
      {
        Serial.println((int)check);
      }
  #endif
  if (check != 0x27)
    {
      #if NRF_DEBUG
        if (NRF_debug)
          {
            for(byte x=0; x < 10; x++)
              {
                digitalWrite(PIN_LED_RGB_R,HIGH);
                delay(50);
                digitalWrite(PIN_LED_RGB_R,LOW);
                delay(50);
              } 
          }
      #endif
      return false;
    }

  NRF_address[0]=Settings.Unit;
  Nrf24_setRADDR((byte *)NRF_address);

  Nrf24_configRegister(NRF_EN_AA, 0x7);       // Auto-Ack Enabled on pipes 0,1,2
  Nrf24_configRegister(NRF_SETUP_RETR, 0xff); // Delay 4000uS, 15 retries
  Nrf24_configRegister(NRF_EN_RXADDR, 0xf);   // Enable RX pipes 0,1,2,3

  Nrf24_configRegister(RX_PW_P0, NRF_PAYLOAD_SIZE);
  Nrf24_configRegister(RX_PW_P1, NRF_PAYLOAD_SIZE);
  Nrf24_configRegister(RX_PW_P2, NRF_PAYLOAD_SIZE);
  Nrf24_configRegister(RX_PW_P3, NRF_PAYLOAD_SIZE);

  Nrf24_setChannel(NRF_channel);

  // Start receiver 
  Nrf24_powerUpRx();
  Nrf24_flushRx();

  return true;
}


#if NRF_DEFAULT_DESTINATION==0
void NRF_CheckOnline()
{
  NRFOnline[0]=0;
  Nrf24_flushRx();
  #if NRF_DEBUG
    if (NRF_debug)
      {
        Serial.println("Online:");
      }
  #endif
  NRFPayload.Data[0]=NRF_SYSTEMCMD_CHECKONLINE;
  NRFSequence++;
  for(int y=1;y<=NRF_UNIT_MAX;y++)
    {
      if ((NRF_sendpacket(Settings.Unit, y, NRF_PAYLOAD_SYSTEM, 1,NRFSequence,0,0) & 0x20) == 0x20)
        {
          #if NRF_DEBUG
            if (NRF_debug)
              {
                Serial.print((int)y);
                Serial.println(" is Online");
              }
          #endif

          NRFOnline[y]=y;
          NodoOnline(y,VALUE_SOURCE_RF);
        }
      else
          NRFOnline[y]=0;
    }
  NRFOnline[0]=1;

  #if NRF_FEATURE_NODE_DISCOVERY
  // broadcast something, so routed node's will learn this unit...
  NRFSequence++;
  NRF_sendpacket(Settings.Unit, NRF_BROADCAST_ADDRESS, NRF_PAYLOAD_SYSTEM, 2,NRFSequence,0,0);
  unsigned long timer=millis()+1000;
  while (millis() < timer)
    NRF_receive(&LastReceived);
  #endif
}
#endif


#if NRF_FEATURE_FINDMASTER
byte NRF_findMaster()
{
  byte channel=2;
  byte found=0;
  Nrf24_flushRx();
  NRFPayload.Data[0]=NRF_SYSTEMCMD_FINDMASTER;
  NRFSequence++;
  while(found < 5 && channel <63)
    {
      #if NRF_DEBUG
        if (NRF_debug)
          {
            Serial.print((int)channel);
            Serial.print(" - ");
          }
      #endif
      Nrf24_setChannel(channel);
      if ((NRF_sendpacket(Settings.Unit, NRF_BROADCAST_ADDRESS_ACK, NRF_PAYLOAD_SYSTEM, 1,NRFSequence,0,0) & 0x20) == 0x20)
        found++;
      else
        {
          found=0;
          channel++;
        }
   }
  return channel;
}
#endif


#if NRF_FEATURE_CHANNELBROADCAST
void NRF_changeChannel(byte channel)
{
  NRFPayload.Data[0]=NRF_SYSTEMCMD_CHANNEL;
  NRFPayload.Data[1]=channel;
  NRFSequence++;
  for (byte x=1; x <63; x++)
    {
      #if NRF_DEBUG
        if (NRF_debug)
          {
            Serial.println((int)x);
          }
      #endif
      Nrf24_setChannel(x);
      NRF_sendpacket(Settings.Unit, NRF_BROADCAST_ADDRESS, NRF_PAYLOAD_SYSTEM, 2,NRFSequence,0,0);
   }
}
#endif


byte NRF_sendpacket(byte Source, byte Destination, byte ID, byte Size, byte Sequence, byte Flags, byte retries)
{
  if (!NRF_live) return 0;

#if NRF_DEFAULT_DESTINATION==0
  byte first=1;
  byte last=NRF_UNIT_MAX;

  if (Destination > 0)
    {
      first=Destination;
      last=Destination;
    }

  if (NRFOnline[0]!=0)
    {
      if (NRF_single_target > 0)
        {
          first=NRF_single_target;
          last=NRF_single_target;
        }

      #if NRF_FEATURE_ROUTER
      if (NRF_forceSingleTarget > 0)
        {
          first=NRF_forceSingleTarget;
          last=NRF_forceSingleTarget;
        }
      #endif
    }
#else
  byte first=NRF_DEFAULT_DESTINATION;
  byte last=NRF_DEFAULT_DESTINATION;
#endif

  NRFPayload.Source=Source;
  NRFPayload.ID=ID;
  NRFPayload.Size=Size;
  NRFPayload.Sequence=Sequence;
  NRFPayload.Flags=Flags;
  NRFPayload.Gateway=Settings.Unit;
  NRFPayload.Future=0;

  boolean ack=true;
  if (first == NRF_BROADCAST_ADDRESS &&  !(NRFPayload.ID == NRF_PAYLOAD_SYSTEM && NRFPayload.Data[0] == NRF_SYSTEMCMD_FINDMASTER))
    ack=false;

  byte status=0;
  for(int y=first;y<=last;y++)
    {
      status=0;
      if((NRFOnline[y] !=0 || NRFOnline[0]==0 || y==NRF_BROADCAST_ADDRESS_ACK ||y==NRF_BROADCAST_ADDRESS) && y != NRFPayload.Source)
        {
          NRFPayload.Destination=y;
          if (y==NRF_BROADCAST_ADDRESS_ACK || y==NRF_BROADCAST_ADDRESS || NRFOnline[0]==0)
            NRF_address[0]=y;
          else
            NRF_address[0]=NRFOnline[y];

          Nrf24_setTADDR((byte *)NRF_address);
          for (byte retry=0; retry <= retries; retry++)
            {
              #if NRF_DEBUG
              if (retry > 0)
                {
                  Serial.print("R");
                  Serial.println(retry);
                }
              #endif

              if (NRF_address[0]==NRF_BROADCAST_ADDRESS)
                Nrf24_configRegister(NRF_EN_AA, 0x6);
              else
                Nrf24_configRegister(NRF_EN_AA, 0x7);

// testmode block, to simulate unreachable nodo's in testlab, remove before production!
#ifdef NRF_TESTBLOCK1
// do not send directly to this unit!
if(NRF_address[0]==NRF_TESTBLOCK1 || NRF_address[0]==NRF_TESTBLOCK2 || NRF_address[0]==NRF_TESTBLOCK3)
  {
    Serial.println("Out Blocked");
    status=0;
  }
else
#endif
// end testblock
              status = Nrf24_send((byte *)&NRFPayload);

              if ((status & 0x20) == 0x20)
                break;
              else
                delay(NRF_RETRY_DELAYMS + Settings.Unit);
            }

          #if NRF_DEBUG
            if (NRF_debug)
              {
                Serial.print("NRF Send:");
                Serial.print(NRFPayload.Destination);
                Serial.print(" to ");
                Serial.print((int)NRF_address[0]);
                Serial.print(" - ");
                Serial.println((int)status);
              }
          #endif

          // set auto-ack receive pipe to null
          NRF_address[0]=0;
          Nrf24_setTADDR((byte *)NRF_address);
        }  // online
    } // for

  return status;
}
#endif // core

#else // BARE_MINIMUM

#define NRF_PAYLOAD_SIZE sizeof(struct NodoEventStruct)
boolean Plugin_033(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_033_CORE

  case PLUGIN_INIT:
      SPI_begin();
      Nrf24_init();
      Nrf24_configRegister(NRF_RF_SETUP, 0x27);
      Nrf24_configRegister(RX_PW_P0, NRF_PAYLOAD_SIZE);
      Nrf24_configRegister(RX_PW_P1, NRF_PAYLOAD_SIZE);
      Nrf24_configRegister(NRF_EN_AA, 0x0);             // Auto-Ack Disabled
      Nrf24_setChannel(NRF_CHANNEL);
      Nrf24_powerUpRx();
      break;

  case PLUGIN_SCAN_EVENT:
      if(Nrf24_dataReady())
        {
          Nrf24_getData((byte *)event);
          success=true;
        }
      break;

  case PLUGIN_EVENT_OUT:
      if(((event->Port==VALUE_SOURCE_RF) || (event->Port==VALUE_ALL)) && ((event->Type==NODO_TYPE_EVENT) || (event->Type==NODO_TYPE_PLUGIN_EVENT)))
        Nrf24_send((byte *)event);
      break;
  }
#endif // PLUGIN_033_CORE
  return success;
}
#endif // BARE_MINIMUM

#ifdef PLUGIN_033_CORE
// ***********************************************************************************************************
// NRF24 specific code
// ***********************************************************************************************************

/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
    Nathan Isburgh <nathan@mrroot.net>
    An Ardunio port of http://www.tinkerer.eu/AVRLib/nRF24L01
    Significant changes to remove depencence on interupts and auto ack support: Aaron Shrimpton <aaronds@gmail.com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

/* Memory Map */
#define CONFIG      0x00
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define RF_CH       0x05
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_TX_PAYLOAD_NOACK  0xB0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

void Nrf24_setChannel(byte channel)
{
  Nrf24_configRegister(RF_CH,channel*2);
  delayMicroseconds(256);
}

void Nrf24_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len)
{
	uint8_t i;
	for(i = 0;i < len;i++){
		datain[i] = SPI_transfer(dataout[i]);
	}
}

void Nrf24_transmitSync(uint8_t *dataout,uint8_t len)
{
	uint8_t i;
	for(i = 0;i < len;i++){
		SPI_transfer(dataout[i]);
	}
}


void Nrf24_init() 
{
    #ifndef NRF_CSN_TRICK   
      pinMode(NRF_CSN_PIN,OUTPUT);
      Nrf24_csnHi();
    #endif
}

void Nrf24_setRADDR(uint8_t * adr) 
{
	Nrf24_writeRegister(RX_ADDR_P1,adr,NRF_ADDR_LEN);
	Nrf24_configRegister(RX_ADDR_P2,NRF_BROADCAST_ADDRESS_ACK);
	Nrf24_configRegister(RX_ADDR_P3,NRF_BROADCAST_ADDRESS);
}

void Nrf24_setTADDR(uint8_t * adr)
{
	Nrf24_writeRegister(RX_ADDR_P0,adr,NRF_ADDR_LEN);
	Nrf24_writeRegister(TX_ADDR,adr,NRF_ADDR_LEN);
}

bool Nrf24_dataReady() 
{
    return !Nrf24_rxFifoEmpty();
}

bool Nrf24_rxFifoEmpty(){
	uint8_t fifoStatus;

	Nrf24_readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
	return (fifoStatus & (1 << RX_EMPTY));
}



void Nrf24_getData(uint8_t * data) 
{
    Nrf24_csnLow();                               // Pull down chip select
    SPI_transfer( R_RX_PAYLOAD );            // Send cmd to read rx payload
    Nrf24_transferSync(data,data,NRF_PAYLOAD_SIZE); // Read payload
    Nrf24_csnHi();                               // Pull up chip select
    Nrf24_configRegister(STATUS,(1<<RX_DR));   // Reset status register
}

void Nrf24_configRegister(uint8_t reg, uint8_t value)
{
    Nrf24_csnLow();
    SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
    SPI_transfer(value);
    Nrf24_csnHi();
}

void Nrf24_readRegister(uint8_t reg, uint8_t * value, uint8_t len)
{
    Nrf24_csnLow();
    SPI_transfer(R_REGISTER | (REGISTER_MASK & reg));
    Nrf24_transferSync(value,value,len);
    Nrf24_csnHi();
}

void Nrf24_writeRegister(uint8_t reg, uint8_t * value, uint8_t len) 
{
    Nrf24_csnLow();
    SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
    Nrf24_transmitSync(value,len);
    Nrf24_csnHi();
}

byte Nrf24_send(uint8_t * value) 
{
    uint8_t status=0;

    Nrf24_configRegister(CONFIG, NRF_CONFIG_DATA); // power down trick to workaround CE pin tied high!
    Nrf24_powerUpTx();       // Set to transmitter mode , Power up
       
    Nrf24_csnLow();                    // Pull down chip select

    SPI_transfer( W_TX_PAYLOAD ); // Write cmd to write payload

    Nrf24_transmitSync(value,NRF_PAYLOAD_SIZE);   // Write payload
    Nrf24_csnHi();                    // Pull up chip select

    unsigned long timer=millis()+NRF_SEND_TIMEOUTMS;
    while(millis()<timer)
      {
        status = Nrf24_getStatus();
        if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
          {
            break;
          }
      }
  Nrf24_flushTx();
  Nrf24_powerUpRx();
  return status;
}

uint8_t Nrf24_getStatus()
{
	uint8_t rv;
	Nrf24_readRegister(STATUS,&rv,1);
	return rv;
}

void Nrf24_powerUpRx()
{
	Nrf24_configRegister(CONFIG, NRF_CONFIG_DATA | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
	Nrf24_configRegister(STATUS,(1 << TX_DS) | (1 << MAX_RT)); 
}

void Nrf24_flushRx()
{
    Nrf24_csnLow();
    SPI_transfer( FLUSH_RX );
    Nrf24_csnHi();
}

void Nrf24_flushTx()
{
    Nrf24_csnLow();
    SPI_transfer( FLUSH_TX );
    Nrf24_csnHi();
}

void Nrf24_powerUpTx()
{
	Nrf24_configRegister(CONFIG, NRF_CONFIG_DATA| ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void Nrf24_csnHi()
{
    #ifdef NRF_CSN_TRICK
      digitalWrite(NRF_SCK_PIN,HIGH);
      delayMicroseconds(64);  // allow csn to settle
    #else 
	digitalWrite(NRF_CSN_PIN,HIGH);
	#ifdef NRF_HWSPI
	  #if NODO_MEGA
	    digitalWrite(EthernetShield_CS_W5100, 0);
	    digitalWrite(EthernetShield_CS_SDCard, 1);
	  #endif
	#endif
    #endif
}

void Nrf24_csnLow()
{
    #ifdef NRF_CSN_TRICK
      digitalWrite(NRF_SCK_PIN,LOW);
      delayMicroseconds(8);  // allow csn to settle
    #else
	#ifdef NRF_HWSPI
	  #if NODO_MEGA
	    digitalWrite(EthernetShield_CS_W5100, 1);
	    digitalWrite(EthernetShield_CS_SDCard, 1);
	  #endif
	#endif
	digitalWrite(NRF_CSN_PIN,LOW);
    #endif
}

void Nrf24_powerDown()
{
	Nrf24_configRegister(CONFIG, NRF_CONFIG_DATA);
}

#ifdef NRF_HWSPI
void SPI_begin()
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(NRF_CE_PIN,OUTPUT);
  digitalWrite(NRF_CE_PIN,HIGH);
  Serial.println("HW SPI");
}

unsigned char SPI_transfer(unsigned char Byte)
{
  return SPI.transfer(Byte);
}

#else
// Software SPI routines

void SPI_begin()
{
  pinMode(NRF_SCK_PIN,OUTPUT);
  pinMode(NRF_MOSI_PIN,OUTPUT);
  pinMode(NRF_MISO_PIN,INPUT);
  pinMode(NRF_MISO_PIN,INPUT_PULLUP);
  digitalWrite(NRF_SCK_PIN,LOW);
  digitalWrite(NRF_MOSI_PIN,HIGH);
}

unsigned char SPI_transfer(unsigned char Byte)
{
  uint8_t mosibit = digitalPinToBitMask(NRF_MOSI_PIN);
  uint8_t mosiport = digitalPinToPort(NRF_MOSI_PIN);
  uint8_t sckbit = digitalPinToBitMask(NRF_SCK_PIN);
  uint8_t sckport = digitalPinToPort(NRF_SCK_PIN);
  uint8_t misobit = digitalPinToBitMask(NRF_MISO_PIN);
  uint8_t misoport = digitalPinToPort(NRF_MISO_PIN);
  volatile uint8_t *mosiout = portOutputRegister(mosiport);
  volatile uint8_t *sckout = portOutputRegister(sckport);
  volatile uint8_t *misoin = portInputRegister(misoport);
  uint8_t oldSREG = SREG;

  cli();

  for(unsigned char i=0;i<8;i++)
  {
    if(Byte & 0x80)
	*mosiout |= mosibit;
    else
	*mosiout &= ~mosibit;

    *sckout |= sckbit;

    Byte <<= 1;
    if (*misoin & misobit)
      Byte |= 1;

    *sckout &= ~sckbit;
  }
  SREG = oldSREG;
  return(Byte);
}
#endif // hw/sw spi

byte NRF_Random() {
  long result=0;
  for (byte x=1; x<250;x++)
  {
    analogRead(0);

    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
       ADMUX = _BV(MUX5) | _BV(MUX0) ;
    #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif  

    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
 
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
 
    result += (high<<8) | low;
  }
  return result &0xff;
}

#endif // plugin core
