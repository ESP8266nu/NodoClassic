//#######################################################################################################
//############################## Plugin-095: SleepExternal ##############################################
//#######################################################################################################

/*********************************************************************************************\
 * Auteur             : M vd Broek
 * Datum              : 28 feb 2016
 * Versie             : Beta 0.1
 * Compatibiliteit    : R787
 * Syntax             : "SleepExternal <mode>"
 \*********************************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>

void Plugin_095_SetPinIRQ(byte _receivePin);
void plugin_095_sleep();

#define PLUGIN_ID 095
#define PLUGIN_NAME "PluginSleep"

boolean Plugin_095(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_095_CORE

  case PLUGIN_ONCE_A_SECOND:
    {
      digitalWrite(13,HIGH);
      delay(10);
      digitalWrite(13,LOW);
      break;
    }

  case PLUGIN_COMMAND:
    {
      switch(event->Par1)
      {
        case 0:
          {
            plugin_095_sleep();
            break;
          }
        case 1:
          {
            if (event->Par2 == 0)
              Plugin_095_SetPinIRQ(A0);
            if (event->Par2 == 1)
              Plugin_095_SetPinIRQ(A1);
            break;
          }
      }
      success=true;
      break;
     }

#endif // PLUGIN_095_CORE

    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(26);
      string[25]=0;
      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
          if(GetArgv(string,TempStr,2)) 
            {
              event->Type = NODO_TYPE_PLUGIN_COMMAND;
              event->Command = 095;
              success=true;
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME);            // Commando / event 
      strcat(string," ");                
      strcat(string,int2str(event->Par1));
      break;
      }
    #endif

  }

  return success;
}

#ifdef PLUGIN_095_CORE

// ************************************************************************************************************************
// Sleep stuff from here...
// ************************************************************************************************************************

volatile uint8_t *_receivePortRegister;

void Plugin_095_SetPinIRQ(byte _receivePin)
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
}

#if defined(PCINT1_vect)
ISR(PCINT1_vect) { pulse_handle_interrupt(); }
#endif
#if defined(PCINT2_vect)
ISR(PCINT2_vect) { pulse_handle_interrupt(); }
#endif


void plugin_095_sleep()
{
  delay(100);
  digitalWrite(13,0);
  // disable ADC
  byte ad= ADCSRA;
  ADCSRA = 0;  

  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = 0;
  // set interrupt mode and an interval 
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = _BV (BODS) | _BV (BODSE);
  MCUCR = _BV (BODS); 
  sleep_cpu ();  
  
  // cancel sleep as a precaution
  sleep_disable();
  ADCSRA = ad;  
}

#endif
