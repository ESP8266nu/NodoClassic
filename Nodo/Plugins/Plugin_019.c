//#######################################################################################################
//#################################### Plugin-19: ID-12 RFID Tag Reader #################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor verwerking van ID-12 RFID Tag Readers
 * 
 * Auteur             : Martinus van den Broek
 * Support            : www.nodo-domotica.nl
 * Datum              : 22 Okt 2013
 * Versie             : 1.2
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 596
 * Syntax             : "RFID <customer ID>,<tag ID>
 ***********************************************************************************************
 * Technische beschrijving:
 *
 * De ID-12 Tag Reader is een device die 125KHz EM4001 compatible tags kan uitlezen. De data wordt serieel aangeboden
 * baudrate is 9600 BAUD
 * We maken gebruik van een analoge input die we als digitale input gebruiken
 * Op de ATMEGA328P platformen gebruiken we poort A3
 * Op de ATMEGA2560 platformen gebruiken we poort A15
 * Elke EM4001 tag levert een unieke code, bestaande uit een 8 bits customer of version ID en een 32 bits serienummer
 * Par1 bevat het customer ID
 * Par2 bevat het 32 bits serienummer
 * Dit device genereert een event, zodat actie kan worden ondernomen via de Nodo eventlist indien een bekende tag wordt gebruikt
 \*********************************************************************************************/
#define PLUGIN_ID       19
#define PLUGIN_NAME "RFID"

#if NODO_MEGA
  #define RFID_PIN         A15 // A0-A7 are not PCINT capable on a ATMega2560!
#else
  #define RFID_PIN         A3
#endif

#include <avr/interrupt.h>
#define _SS_MAX_RX_BUFF       16 // RX buffer size
#define RX_DELAY_CENTERING   114
#define RX_DELAY_INTRABIT    236
#define RX_DELAY_STOPBIT     236

//prototypes
inline void rfidDelay(uint16_t delay);
void rfid_recv();
uint8_t rfid_rx_pin_read();
int rfid_available();
int rfid_read();
inline void rfid_handle_interrupt();

#ifdef PLUGIN_019_CORE
uint16_t _buffer_overflow;
uint8_t _receivePin=RFID_PIN;
uint8_t _receiveBitMask;
volatile uint8_t *_receivePortRegister;
char _receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t _receive_buffer_tail = 0;
volatile uint8_t _receive_buffer_head = 0;
#endif

boolean Plugin_019(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_019_CORE
  case PLUGIN_INIT:
    {
      // Init IO pin
      pinMode(_receivePin, INPUT);
      digitalWrite(_receivePin, HIGH);
      _receivePin = _receivePin;
      _receiveBitMask = digitalPinToBitMask(_receivePin);
      uint8_t port = digitalPinToPort(_receivePin);
      _receivePortRegister = portInputRegister(port);
  
      if (digitalPinToPCICR(_receivePin))
        {
          *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
          *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
        }
      break;
    }

  case PLUGIN_ONCE_A_SECOND:
    {
      if(rfid_available() > 0)
        {
          byte val = 0;
          byte code[6];
          byte checksum = 0;
          byte bytesread = 0;
          byte tempbyte = 0;

          if((val = rfid_read()) == 2)
            {                  // check for header 
              bytesread = 0; 
              while (bytesread < 12) {                        // read 10 digit code + 2 digit checksum
              if( rfid_available() > 0) { 
                val = rfid_read();
                if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) {
                  // if header or stop bytes before the 10 digit reading 
                  break;
                 }

               // Do Ascii/Hex conversion:
              if ((val >= '0') && (val <= '9')) {
                val = val - '0';
              } 
              else if ((val >= 'A') && (val <= 'F')) {
                val = 10 + val - 'A';
              }

              // Every two hex-digits, add byte to code:
              if (bytesread & 1 == 1) {
                // make some space for this hex-digit by
                // shifting the previous hex-digit with 4 bits to the left:
                code[bytesread >> 1] = (val | (tempbyte << 4));

                if (bytesread >> 1 != 5) {                // If we're at the checksum byte,
                  checksum ^= code[bytesread >> 1];       // Calculate the checksum... (XOR)
                  };
                } 
                else {
                  tempbyte = val;                           // Store the first hex digit first...
                };
              bytesread++;                                // ready to read next digit
          } 
        }
      }

      // Output to Serial:
      if (bytesread == 12)
        {
          if (code[5]==checksum)
            {
              struct NodoEventStruct TempEvent;
              ClearEvent(&TempEvent);
              TempEvent.SourceUnit = 0;
              TempEvent.Direction  = VALUE_DIRECTION_OUTPUT;
              TempEvent.Command    = PLUGIN_ID;
              TempEvent.Port       = VALUE_ALL;
              TempEvent.Par1       = code[0];
              TempEvent.Par2       = 0;
              TempEvent.Type       = NODO_TYPE_PLUGIN_EVENT;
              for (byte i=1; i<5; i++) TempEvent.Par2 = TempEvent.Par2 | (((unsigned long) code[i] << ((4-i)*8)));
              ProcessEvent(&TempEvent);
            }
        }

      } // available 
      break;
    }
    
#endif // PLUGIN_019_CORE

#if NODO_MEGA
  case PLUGIN_MMI_IN:
    {
    char* str=(char*)malloc(INPUT_COMMAND_SIZE);
  
    if(GetArgv(string,str,1))
      {
      if(strcasecmp(str,PLUGIN_NAME)==0)
        {
        if(GetArgv(string,str,2))
          {
          event->Par1=str2int(str);    
          if(GetArgv(string,str,3))
            {
              event->Par2=str2int(str);
              event->Type = NODO_TYPE_PLUGIN_EVENT;
              event->Command = 19; // Plugin nummer  
              success=true;
            }
          }
        }
      }
    free(str);
    break;
    }

  case PLUGIN_MMI_OUT:
    {
    strcpy(string,PLUGIN_NAME);            // Eerste argument=het commando deel
    strcat(string," ");
    strcat(string,int2str(event->Par1)); 
    strcat(string,",");
    strcat(string,int2strhex(event->Par2));

    break;
    }
#endif //NODO_MEGA
  }      
  return success;
}
#ifdef PLUGIN_019_CORE
/*********************************************************************/
inline void rfidDelay(uint16_t delay) { 
/*********************************************************************/
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

/*********************************************************************/
void rfid_recv()
/*********************************************************************/
{
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit, so interrupt is probably not for us
  if (!rfid_rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    rfidDelay(RX_DELAY_CENTERING);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      rfidDelay(RX_DELAY_INTRABIT);
      uint8_t noti = ~i;
      if (rfid_rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    rfidDelay(RX_DELAY_STOPBIT);

    // if buffer full, set the overflow flag and return
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else 
    {
      _buffer_overflow = true;
    }
  }
}

/*********************************************************************/
uint8_t rfid_rx_pin_read()
/*********************************************************************/
{
  return *_receivePortRegister & _receiveBitMask;
}

/*********************************************************************/
int rfid_read()
/*********************************************************************/
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

/*********************************************************************/
int rfid_available()
/*********************************************************************/
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

/*********************************************************************/
inline void rfid_handle_interrupt()
/*********************************************************************/
{
  rfid_recv();
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect) { rfid_handle_interrupt(); }
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect) { rfid_handle_interrupt(); }
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect) { rfid_handle_interrupt(); }
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect) { rfid_handle_interrupt(); }
#endif

#endif //PLUGIN_019_CORE
