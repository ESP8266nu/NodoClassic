//#######################################################################################################
//#################################### Plugin-028 4-voudige PulseCounter #################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit device zorgt voor ondersteuning van een 4-voudige pulsteller met 1 enkele Nodo Mega of Small.
 * Is vooral bedoeld als "Meterkast Slave" unit voor het meten van zowel Gas, Elektra en Water met 1 Nodo.
 *
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 10 Sep 2013
 * Versie             : 1.0
 * Nodo productnummer : 
 * Compatibiliteit    : 
 * Syntax             : Pulse <Counter Number>, <Basis Variabele>
 * LET OP             : om allerlei WiredIn meldingen te vermijden bij het tellen van pulsen, geef je de volgende commando's op de Nodo:
 *                        WiredThreshold <poort>,0
 *                        (Doe dit voor elke poort die je als pulsteller gaat gebruiken)
 ***********************************************************************************************
 * Technische beschrijving:
 * We gebruiken de Nodo WiredIn aansluitingen als digitale pulstellers.
 * Het is n.l. mogelijk om de diverse Atmel pinnen voor alternatieve functies te gebruiken.
 * De WiredIn pinnen zijn standaard analoog, maar ook digitaal te gebruiken en daar maken we gebruik van.
 * Met dit device is het mogelijk om ook zeer korte pulsen te tellen, hetgeen met de normale Nodo WiredIn functie niet lukt.
 * Dit device gebruikt twee variabelen voor elke pulsteller, 1 voor de pulsecount en 1 voor de pulsetime
 * Deze variabelen worden alleen gebruikt na het geven van het commando pulse x,y
 \*********************************************************************************************/
#define PLUGIN_ID       28
#define PLUGIN_NAME "Pulse"

#include "Arduino.h"
#include <avr/interrupt.h>

void SetPinIRQ(byte _receivePin);
inline void pulse_handle_interrupt();

#ifdef PLUGIN_028_CORE
volatile uint8_t *_receivePortRegister;
volatile unsigned long PulseCounter[4]={0,0,0,0};
volatile unsigned long PulseTimer[4]={0,0,0,0};
volatile unsigned long PulseTimerPrevious[4]={0,0,0,0};
volatile byte PulseState[4]={0,0,0,0};
#endif

boolean Plugin_028(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_028_CORE
  case PLUGIN_INIT:
    {
      // Init IO pins
      #if NODO_MEGA
        SetPinIRQ(A8);
        SetPinIRQ(A9);
        SetPinIRQ(A10);
        SetPinIRQ(A11);
      #else
        SetPinIRQ(A0);
        SetPinIRQ(A1);
        SetPinIRQ(A2);
        SetPinIRQ(A3);
      #endif
      break;
    }

  case PLUGIN_ONCE_A_SECOND:
    {
      break;
    }
    
  case PLUGIN_RAWSIGNAL_IN:
    {
      break;
    }

  case PLUGIN_COMMAND:
    {
      // Store countervalue into uservar and reset internal counter
      UserVar[event->Par2-1] = float(PulseCounter[event->Par1-1]);
      PulseCounter[event->Par1-1]=0;

      // If actual time difference > last known pulstime, update pulstimer now.
      if ((millis() - PulseTimerPrevious[event->Par1-1]) > PulseTimer[event->Par1-1])
        PulseTimer[event->Par1-1] = millis() - PulseTimerPrevious[event->Par1-1];

      if (PulseTimer[event->Par1-1] > 999999) PulseTimer[event->Par1-1] = 999999; // WebApp cannot handle larger values! (bug or by design?)
      UserVar[event->Par2] = float(PulseTimer[event->Par1-1]);
      success=true;
      break;
    }
#endif // PLUGIN_028_CORE
#if NODO_MEGA
  case PLUGIN_MMI_IN:
    {
      char* str=(char*)malloc(INPUT_COMMAND_SIZE);
    
      if(GetArgv(string,str,1))
        {
        if(strcasecmp(str,PLUGIN_NAME)==0)
          {
          event->Command=PLUGIN_ID;
          if(GetArgv(string,str,2))
            {
            event->Par1=str2int(str);    
            if(GetArgv(string,str,3))
              {
               if(event->Par1>0 && event->Par1<5 && event->Par2>0 && event->Par2<=USER_VARIABLES_MAX-1)            
                 {
                  event->Par2=str2int(str);    
                  event->Type = NODO_TYPE_PLUGIN_COMMAND;
                  event->Command = 28; // Plugin nummer
                  success=true;
                }
              }
            }
          }
        }
      free(str);
      break;
      }

  case PLUGIN_MMI_OUT:
    {
      if(event->Command==PLUGIN_ID)
      {
        strcpy(string,PLUGIN_NAME);
        strcat(string," ");
        strcat(string,int2str(event->Par1)); 
        strcat(string,",");
        strcat(string,int2str(event->Par2));
        success=true;
      }
      break;
    }
#endif //NODO_MEGA
  }      
  return success;
}
#ifdef PLUGIN_028_CORE
void SetPinIRQ(byte _receivePin)
{
  uint8_t _receiveBitMask;
  pinMode(_receivePin, INPUT);
  digitalWrite(_receivePin, HIGH);
  _receiveBitMask = digitalPinToBitMask(_receivePin);
  uint8_t port = digitalPinToPort(_receivePin);
  _receivePortRegister = portInputRegister(port);
  if (digitalPinToPCICR(_receivePin))
    {
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
      *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
    }
}

/*********************************************************************/
inline void pulse_handle_interrupt()
/*********************************************************************/
{
  byte portstate= *_receivePortRegister & 0xf;
  for (byte x=0; x<4; x++)
    {
      if ((portstate & (1 << x)) == 0)
        {
          if (PulseState[x] == 1)
            {
              PulseCounter[x]++;
              PulseTimer[x]=millis()-PulseTimerPrevious[x];
              PulseTimerPrevious[x]=millis();
            }
          PulseState[x]=0;
        }
      else
        PulseState[x]=1;
    }
}

#if defined(PCINT1_vect)
ISR(PCINT1_vect) { pulse_handle_interrupt(); }
#endif
#if defined(PCINT2_vect)
ISR(PCINT2_vect) { pulse_handle_interrupt(); }
#endif

#endif //PLUGIN_028_CORE
