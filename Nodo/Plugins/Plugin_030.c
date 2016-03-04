//#######################################################################################################
//################################# Plugin-30: Wiegand RFID Tag Reader ##################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor verwerking van RFID Tag Readers die werken met het Wiegand-26 protocol
 * 
 * Auteur             : Martinus van den Broek / Richard Ten Klooster
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Mrt 2014
 * Versie             : 0.6
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 596
 * Syntax             : RFIDWG <reader unit>,<tag ID>
 ***********************************************************************************************
 * Technische beschrijving:
 *
 * Een Wiegand-26 Tag Reader is een device die 125KHz EM4001 compatible tags kan uitlezen. De data wordt via het Wiegand 26 protocol aangeboden
 * Dit betekent dat er twee datalijnen zijn welke korte pulsen genereren voor respectievelijk 0 en 1. Pulsen zijn zeer kort (50uSec) dus IRQ is nodig.
 * We gebruiken de PIN-CHANGE interrupt voorziening van de Atmel.
 * We gebuiken hiervoor twee (of vier) van de WiredIn aansluitingen van de Nodo
 * We maken dus gebruik van een analoge input die we als digitale input met IRQ support gaan gebruiken
 * Op de ATMEGA328P platformen gebruiken we poort A0 en A1
 * Op de ATMEGA2560 platformen gebruiken we poort A8 en A9
 * Elke EM4001 tag levert een unieke code, bestaande uit een 26 bits serienummer
 * Par1 bevat het nummer van de reader (1 of 2)
 * Par2 bevat het 26 bits serienummer
 * Dit device genereert een event, zodat actie kan worden ondernomen via de Nodo eventlist indien een bekende tag wordt gebruikt
 * In de config file dient te worden ingesteld of er 1 of 2 units worden ondersteund
 \*********************************************************************************************/

#define PLUGIN_NAME "RFIDWG"
#define PLUGIN_ID   30

#if PLUGIN_030_CORE == 1
  #define PLUGIN_030_BITMASK 0x3
#else
  #define PLUGIN_030_BITMASK 0xf
#endif
#define PLUGIN_030_WGSIZE 26

#include "Arduino.h"
#include <avr/interrupt.h>

void SetPinIRQ(byte _receivePin);
inline void pulse_handle_interrupt();

volatile uint8_t *_receivePortRegister;

volatile byte Plugin_030_bitCount = 0;	           // Count the number of bits received.
volatile unsigned long Plugin_030_keyBuffer = 0;   // A 32-bit-long keyBuffer into which the number is stored.
byte Plugin_030_bitCountPrev = 0;                  // to detect noise
byte Plugin_030_Unit=0;

// In de funktienaam zit het pluginnummer verwerkt zodat deze eenduidig kan worden geidentificeerd en aangeroepen. De Nodo
boolean Plugin_030(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  switch(function)
    {    
    #ifdef PLUGIN_030_CORE

    case PLUGIN_INIT:
      {
        // Init IO pins
        #if NODO_MEGA
          SetPinIRQ(A8);
          SetPinIRQ(A9);
          #if PLUGIN_030_CORE == 2
            SetPinIRQ(A10);
            SetPinIRQ(A11);
          #endif
        #else
          SetPinIRQ(A0);
          SetPinIRQ(A1);
          #if PLUGIN_030_CORE == 2
            SetPinIRQ(A2);
            SetPinIRQ(A3);
          #endif
        #endif
        break;
      }

    case PLUGIN_ONCE_A_SECOND:
      {
        if ((Plugin_030_bitCount != PLUGIN_030_WGSIZE) && (Plugin_030_bitCount == Plugin_030_bitCountPrev))
          {
            // must be noise
            Plugin_030_bitCount = 0;
            Plugin_030_keyBuffer = 0;
          }

        if (Plugin_030_bitCount == PLUGIN_030_WGSIZE)
          {
            Plugin_030_bitCount = 0;          // Read in the current key and reset everything so that the interrupts can

            Plugin_030_keyBuffer = Plugin_030_keyBuffer >> 1;          // Strip leading and trailing parity bits from the keyBuffer
            Plugin_030_keyBuffer &= 0xFFFFFF;

            struct NodoEventStruct TempEvent;
            ClearEvent(&TempEvent);
            TempEvent.SourceUnit = Settings.Unit;
            TempEvent.Direction  = VALUE_DIRECTION_INPUT;
            TempEvent.Command    = PLUGIN_ID;
            TempEvent.Port       = VALUE_SOURCE_WIRED;
            TempEvent.Par1       = Plugin_030_Unit;
            TempEvent.Par2       = Plugin_030_keyBuffer;
            TempEvent.Type       = NODO_TYPE_PLUGIN_EVENT;
            ProcessEvent(&TempEvent);
            Plugin_030_keyBuffer = 0;          // Clear the buffer for the next iteration.
          }

      Plugin_030_bitCountPrev = Plugin_030_bitCount; // store this value for next check, detect noise
      break;
      }
    #endif // CORE
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
     case PLUGIN_MMI_IN:
    {
    char* str=(char*)malloc(40);
    string[25]=0; // kap voor de zekerheid de string af.
  
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
              event->Command = PLUGIN_ID;   
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
        strcpy(string,PLUGIN_NAME);               // Commando 
        strcat(string," ");
        strcat(string,int2str(event->Par1));      // Parameter-1 (8-bit)
        strcat(string,",");
        strcat(string,int2strhex(event->Par2));
        break;
      }
    #endif //MMI
    }
    
  return success;
  }

#ifdef PLUGIN_030_CORE
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
  byte portstate= *_receivePortRegister & PLUGIN_030_BITMASK;

  if((portstate & 0x3) != 0x3) // first unit!
      Plugin_030_Unit=1;

  #if PLUGIN_030_CORE == 2
    if((portstate & 0xC) != 0xC) // second unit!
      {
        portstate = portstate >> 2;
        Plugin_030_Unit=2;
      }
  #endif

  portstate = portstate & 0x3;

  switch(portstate)
    {
       case 0:			         // Should never happen with Wiegand protocol...
       break;
       case 1: 			         // We've received a 1 bit. (bit 0 = high, bit 1 = low)
         Plugin_030_keyBuffer = Plugin_030_keyBuffer << 1;     // Left shift the number (effectively multiplying by 2)
         Plugin_030_keyBuffer += 1;    		 // Add the 1 (not necessary for the zeroes)
         Plugin_030_bitCount++;   			 // Increment the bit count
       break;
       case 2:				 // We've received a 0 bit. (bit 0 = low, bit 1 = high)
         Plugin_030_keyBuffer = Plugin_030_keyBuffer << 1;     // Left shift the number (effectively multiplying by 2)
         Plugin_030_bitCount++;       		 // Increment the bit count
         break;
       case 3:				 // Back to "idle state"
       break;
     }
}

#if defined(PCINT1_vect)
ISR(PCINT1_vect) { pulse_handle_interrupt(); }
#endif
#if defined(PCINT2_vect)
ISR(PCINT2_vect) { pulse_handle_interrupt(); }
#endif

#endif // CORE
