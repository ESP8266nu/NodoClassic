//#######################################################################################################
//####################### Plugin-093: Domoticz Sensor using ESP Easy sketch #############################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor communicatie met Domoticz via een ESP met ESP Easy sketch
 * communicatie tussen Nodo en ESP is soft serial op twee IO pinnen naar keuze
 * Indien de Nodo Event Bridge op de ESP Easy is geladen, dan worden ook Nodo events tussen units uitgewisseld
 * 
 * Auteur             : M vd Broek
 * Datum              : 21 jan 2016
 * Versie             : Beta 0.1
 * Compatibiliteit    : Vanaf Nodo build nummer 787 (NIET GESCHIKT VOOR Nodo 3.8 !)
 *
 * Syntax             : Domoticz VariableSend <var>,<sensortype>,<idx>
 *                      Domoticz VariableGet <var>,<sensortype>,<idx>
 *
 * Instructie         : Zet de volgende regels in het config bestand
 *
 *                              #define NODO_BETA_PLUGIN_SENDTO
 *				#define PLUGIN_093
 *				#define PLUGIN_093_CORE
 *				#define PLUGIN_093_DEBUG          true
 * 				#define PLUGIN_093_SERIAL_RX_PIN  A0 
 * 				#define PLUGIN_093_SERIAL_TX_PIN  A1
 *
 *                      Als Plugin 019, 028 of 030 wordt gebruikt, neem dan digital 3 als RX pin !
 *
 *			De plugin gebruikt software serial voor communicatie met de ESP8266 op 9600 BAUD
 *                      
\*********************************************************************************************/

#define PLUGIN_ID   93
#define PLUGIN_NAME_093 "Domoticz"

#ifndef ESP_HWSERIAL
  #define ESP_HWSERIAL false
#endif

#ifndef PLUGIN_093_DEBUG
  #define PLUGIN_093_DEBUG false
#endif

#ifdef PLUGIN_093_CORE

// function prototypes for plugin
boolean Plugin_093_softSerialSend(char *cmd, int intdelay);
boolean Plugin_093_init();

byte Serial_available(void);
void Serial_flush(void);
void Serial_write(char data);
void Serial_print(char* string);
void Serial_println(char* string);
int Serial_read(void);
int Serial_peek(void);
void Serial_debugwrite(char character);
void Serial_debugprint(char* string);
void Serial_debugprintln(char* string);
char* Plugin_093_int2str(unsigned long x);

// function prototypes software serial
void SoftwareSerial_print(char* string);
void SoftwareSerial_println(char* string);
void SoftwareSerial_init(byte receivePin_093, byte transmitPin);
inline void SoftwareSerial_tunedDelay(uint16_t delay);
void SoftwareSerial_recv();
void SoftwareSerial_tx_pin_write(uint8_t pin_state);
uint8_t SoftwareSerial_rx_pin_read();
void SoftwareSerial_setTX(uint8_t tx);
void SoftwareSerial_setRX(uint8_t rx);
void SoftwareSerial_end();
int SoftwareSerial_read();
int SoftwareSerial_available();
size_t SoftwareSerial_write(uint8_t b);
int SoftwareSerial_peek();
void SoftwareSerial_flush();

void Plugin_093_ISR();

boolean ESP_live = true;

#if PLUGIN_093_DEBUG
  boolean ESP_debug = true;
#endif

#endif // CORE

boolean Plugin_093(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  switch(function)
    {    
    #ifdef PLUGIN_093_CORE

    case PLUGIN_INIT:
      {
        SoftwareSerial_init(PLUGIN_093_SERIAL_RX_PIN,PLUGIN_093_SERIAL_TX_PIN);         // RX, TX
        break;
      }

    case PLUGIN_SCAN_EVENT:
      {
        if (Serial_available())
        {
          byte test = Serial_peek();
          if (test == 255)
          {
            delay(20);
            if (Serial_read() == 255 && Serial_read() == 254)
            {
                byte data[14];
                byte count = 0;
                while (Serial_available() && count < 14)
                  data[count++] = Serial_read();
                memcpy((byte*)event, (byte*)&data, sizeof(struct NodoEventStruct));
                if(Checksum(event))
                  {
                    event->Direction = VALUE_DIRECTION_INPUT;
                    event->Port      = VALUE_SOURCE_RF;
                    success = true;
                  }
                else
                  Serial_debugprintln("CRC!");
             }
          } // peek
         else
           Serial_flush();
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
          for (byte x = 0; x < 16; x++)
            Serial_write(cmd[x]);
          delay(50);
        }
        success=true;
        break;
      }

    case PLUGIN_COMMAND:
      {
				  		// Par 1 ='command/type'
        byte Par2=event->Par2 & 0xff;    	// Par 2 = Variabel nr 
        byte Par3=event->Par2>>8 & 0xff;	// Par 3 = Sensor type 
        int Par4=event->Par2>>16 & 0xffff;	// Par 4 = Domoticz device IDX

        if (event->Par1 == CMD_VARIABLE_SEND)
          {
            char str[40];
            strcpy(str,"DomoticzSend ");
            strcat(str,Plugin_093_int2str(Par3));
            strcat(str,",");
            strcat(str,Plugin_093_int2str(Par4));
            strcat(str,",");
            dtostrf(UserVar[Par2-1], 0, 2,str+strlen(str));
            Plugin_093_softSerialSend(str,300);
          }

        if (event->Par1 == CMD_VARIABLE_GET)
          {
            char str[40];
            strcpy(str,"DomoticzGet ");
            strcat(str,Plugin_093_int2str(Par3));
            strcat(str,",");
            strcat(str,Plugin_093_int2str(Par4));
            Plugin_093_softSerialSend(str,0);

            unsigned long timer=millis() + 2000;
            char buffer[80];
            byte pos=0;
            char currentchar;
            while (millis() < timer)
              {
                while (Serial_available())
                {
                  currentchar = Serial_read();
                  if (currentchar != 13)
                    {
                      if (pos < (80-1) && currentchar != 10)
                        buffer[pos++]=currentchar;
                    }
                  else
                    {
                      buffer[pos]=0;
                      Serial.println(buffer);
                      // check reply here
                      char* TempStr=(char*)malloc(80);
                      if (GetArgv(buffer,TempStr,1))
                      {
                        if(strcasecmp(TempStr,"DomoticzGet")==0)
                        {
                          if(GetArgv(buffer,TempStr,2))
                            UserVar[Par2-1]=atof(TempStr);
                          if(GetArgv(buffer,TempStr,3))
                            UserVar[Par2]=atof(TempStr);
                        }
                      }
                      free(TempStr);
                      buffer[0]=0;
                      pos=0;
                    }
                }
              }

          }

        success=true;
        break;
      } // case COMMAND

    #endif // CORE

    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char* TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_093)==0)
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
      strcpy(string,PLUGIN_NAME_093);
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

#ifdef PLUGIN_093_CORE
boolean Plugin_093_softSerialSend(char *cmd, int intdelay)
{
  char currentchar=0;
  char lastchar=0;
  unsigned long timer=millis()+ intdelay;

  Serial_flush();

  #if PLUGIN_093_DEBUG
    if (ESP_debug)
      Serial_debugprintln(cmd);
  #endif

  Serial_println(cmd);

  while (millis() < timer)
    {
      while (Serial_available())
        {
          currentchar = Serial_read();
          Serial_debugwrite(currentchar);
        }
    }
  return false;
}

char* Plugin_093_int2str(unsigned long x)
{
  static char OutputLine[12];
  char* OutputLinePosPtr=&OutputLine[10];
  int y;

  *OutputLinePosPtr=0;

  if(x==0)
    {
    *--OutputLinePosPtr='0';
    }
  else
    {  
    while(x>0)
      {
      *--OutputLinePosPtr='0'+(x%10);
      x/=10;
      }
    }    
  return OutputLinePosPtr;
  }

#if !NODO_MEGA
byte GetArgv(char *string, char *argv, int argc)
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
      if(c!=' ' && c!=',')
        {
        argv[argv_pos++]=c;
        argv[argv_pos]=0;
        }          

      if(d==' ' || d==',' || d==0)
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


// ================================================================================
// Serial functions to provide ease change between hw/sw serial

byte Serial_available(void)
{
#if ESP_HWSERIAL
   return Serial.available();
#else
   return SoftwareSerial_available();
#endif
}

int Serial_read(void)
{
#if ESP_HWSERIAL
  return Serial.read();
#else
  return SoftwareSerial_read();
#endif
}

int Serial_peek(void)
{
#if ESP_HWSERIAL
  return Serial.peek();
#else
  return SoftwareSerial_peek();
#endif
}

void Serial_flush()
{
#if ESP_HWSERIAL
  while(Serial_available())
     Serial_read();
#else
   SoftwareSerial_flush();
#endif
}

void Serial_write(char data)
{
#if ESP_HWSERIAL
  Serial.print(data);
#else
  SoftwareSerial_write(data);
#endif
}

void Serial_print(char* string)
{
#if ESP_HWSERIAL
  Serial.print(string);
#else
  SoftwareSerial_print(string);
#endif
}

void Serial_println(char* string)
{
#if ESP_HWSERIAL
  Serial.println(string);
#else
  SoftwareSerial_println(string);
#endif
}

void Serial_debugwrite(char character)
{
#if ESP_HWSERIAL
  SoftwareSerial_write(character);
#else
  Serial.write(character);
#endif
}

void Serial_debugprint(char* string)
{
#if ESP_HWSERIAL
  SoftwareSerial_print(string);
#else
  Serial.print(string);
#endif
}

void Serial_debugprintln(char* string)
{
#if ESP_HWSERIAL
  SoftwareSerial_println(string);
#else
  Serial.println(string);
#endif
}

char* PLUGIN_093_int2str(unsigned long x)
{
  static char OutputLine[12];
  char* OutputLinePosPtr=&OutputLine[10];
  int y;

  *OutputLinePosPtr=0;

  if(x==0)
    {
    *--OutputLinePosPtr='0';
    }
  else
    {  
    while(x>0)
      {
      *--OutputLinePosPtr='0'+(x%10);
      x/=10;
      }
    }    
  return OutputLinePosPtr;
  }

// ================================================================================

/*
  Software serial gebaseerd op originele softwareserial lib
  Waarom deze code?
   - Include lib vanuit plugin file werkt niet
   - Code vereenvoudig en gereduceerd ivm Nodo plugin gebruik op Atmel 328 platform
*/

// settings for 16 MHz CPU, 9600 BAUD
#define _SS_MAX_RX_BUFF         128 // RX buffer size
// Intrabit timing slightly adjusted, because program code is reduced
#define RX_DELAY_CENTERING      114
#define RX_DELAY_INTRABIT       236
#define RX_DELAY_STOPBIT        236
#define TX_DELAY                233
#define XMIT_START_ADJUSTMENT     5

uint8_t _receivePin_093;
uint8_t _receiveBitMask_093;
volatile uint8_t *_receivePortRegister_093;
char _receive_buffer_093[_SS_MAX_RX_BUFF]; 
volatile uint8_t _receive_buffer_093_tail = 0;
volatile uint8_t _receive_buffer_093_head = 0;

uint8_t _transmitBitMask;
volatile uint8_t *_transmitPortRegister;

void SoftwareSerial_print(char* string)
{
  byte x=0;
  while (string[x] != 0)
  {
    SoftwareSerial_write(string[x]);
    x++;
  }
}

void SoftwareSerial_println(char* string)
{
  SoftwareSerial_print(string);
  SoftwareSerial_write(13);
  SoftwareSerial_write(10);
}

void SoftwareSerial_init(byte receivePin_093, byte transmitPin)
{
  SoftwareSerial_setTX(transmitPin);
  SoftwareSerial_setRX(receivePin_093);

  #if (PLUGIN_093_SERIAL_RX_PIN == 2 || PLUGIN_093_SERIAL_RX_PIN == 3)
    attachInterrupt(PLUGIN_093_SERIAL_RX_PIN-2,Plugin_093_ISR,FALLING);
  #else
  if (digitalPinToPCICR(_receivePin_093))
  {
    *digitalPinToPCICR(_receivePin_093) |= _BV(digitalPinToPCICRbit(_receivePin_093));
    *digitalPinToPCMSK(_receivePin_093) |= _BV(digitalPinToPCMSKbit(_receivePin_093));
  }
  #endif

  SoftwareSerial_tunedDelay(TX_DELAY); // if we were low this establishes the end
}

inline void SoftwareSerial_tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+w" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

void SoftwareSerial_recv()
{
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (!SoftwareSerial_rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    SoftwareSerial_tunedDelay(RX_DELAY_CENTERING);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      SoftwareSerial_tunedDelay(RX_DELAY_INTRABIT);
      uint8_t noti = ~i;
      if (SoftwareSerial_rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    SoftwareSerial_tunedDelay(RX_DELAY_STOPBIT);

    if ((_receive_buffer_093_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_093_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer_093[_receive_buffer_093_tail] = d; // save new byte
      _receive_buffer_093_tail = (_receive_buffer_093_tail + 1) % _SS_MAX_RX_BUFF;
    } 
  }
}

void SoftwareSerial_tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}

uint8_t SoftwareSerial_rx_pin_read()
{
  return *_receivePortRegister_093 & _receiveBitMask_093;
}

#if (PLUGIN_093_SERIAL_RX_PIN == 2 || PLUGIN_093_SERIAL_RX_PIN == 3)
void Plugin_093_ISR()
{
  SoftwareSerial_recv();
}

#else
ISR(PCINT0_vect)
{
  SoftwareSerial_recv();
}

ISR(PCINT1_vect)
{
  SoftwareSerial_recv();
}

ISR(PCINT2_vect)
{
  SoftwareSerial_recv();
}
#endif

void SoftwareSerial_setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial_setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin_093 = rx;
  _receiveBitMask_093 = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister_093 = portInputRegister(port);
}

void SoftwareSerial_end()
{
  if (digitalPinToPCMSK(_receivePin_093))
    *digitalPinToPCMSK(_receivePin_093) &= ~_BV(digitalPinToPCMSKbit(_receivePin_093));
}

int SoftwareSerial_read()
{
  // Empty buffer?
  if (_receive_buffer_093_head == _receive_buffer_093_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer_093[_receive_buffer_093_head]; // grab next byte
  _receive_buffer_093_head = (_receive_buffer_093_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial_available()
{
  return (_receive_buffer_093_tail + _SS_MAX_RX_BUFF - _receive_buffer_093_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial_write(uint8_t b)
{
  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  SoftwareSerial_tx_pin_write(LOW);
  SoftwareSerial_tunedDelay(TX_DELAY + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      SoftwareSerial_tx_pin_write(HIGH); // send 1
    else
      SoftwareSerial_tx_pin_write(LOW); // send 0
  
    SoftwareSerial_tunedDelay(TX_DELAY);
  }
  SoftwareSerial_tx_pin_write(HIGH); // restore pin to natural state

  SREG = oldSREG; // turn interrupts back on
  SoftwareSerial_tunedDelay(TX_DELAY);
  
  return 1;
}

int SoftwareSerial_peek()
{
  // Empty buffer?
  if (_receive_buffer_093_head == _receive_buffer_093_tail)
    return -1;

  // Read from "head"
  return _receive_buffer_093[_receive_buffer_093_head];
}

void SoftwareSerial_flush()
{
  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_093_head = _receive_buffer_093_tail = 0;
  SREG = oldSREG;
}
#endif
