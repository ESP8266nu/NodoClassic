//#######################################################################################################
//#################################### Plugin-22: HCSR04_Read  ##########################################
//#######################################################################################################

/*********************************************************************************************\
 * Funktionele beschrijving:
 * 
 * Deze device code zorgt voor uitlezing van een ultrasone afstandsmeter HC-SR04. Dit is een low-cost
 * sensor die met behulp van ultrasone pulsen de afstand tot een object kan meten met een bereik van
 * 2 tot 400 centimeter. Kan worden gebruikt als bijvoorbeeld parkeerhulp of als beveiliging van een  
 * ruimte of object. Na aanroep van dit device wordt de afstand tot het object in centimeters geplaatst
 * in de opgegeven variabele.
 *
 * Iedere keer als [HCSR04_Read <variabele>] wordt aangeroepen, zal de opgegeven variabele worden gevuld
 * met de gemeten afstand. De variabele kan vervolgens worden gebruikt voor eigen toepassing.
 * 
 * Auteur             : Paul Tonkes, p.k.tonkes@gmail.com
 * Support            : www.nodo-domotica.nl
 * Datum              : 23-04-2013
 * Versie             : 1.0
 * Nodo productnummer : Plugin-22 HC-SR04 Distance sensor (SWACDE-22-V10)
 * Compatibiliteit    : Nodo release 3.5.0
 * Syntax             : "HCSR04_Read <variabele>"
 *
 ***********************************************************************************************
 * Technische beschrijving:
 *
 * Compiled size      : 675 bytes voor een Mega en 660 voor een Small.
 * Externe funkties   : float2ul(), GetArgv()
 *
 * De sensor heeft vier aansluitingen:
 * VCC  => +5
 * TRIG => Aansluiten op WiredOut-1  (ATMega2560=D30, ATMega328=D7)
 * ECHO => Aansluiten op WiredOut-2  (ATMega2560=D31, ATMega328=D8)
 * GND  => Massa
 *
 * (sluit uit veiligheid TRIG en ECHO aan via een weerstand van 1K Ohm)
 \*********************************************************************************************/
 
#define PLUGIN_NAME_22 "HCSR04_Read"

boolean Plugin_022(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  switch(function)
    {        
    #ifdef PLUGIN_022_CORE
    case PLUGIN_COMMAND:
      {
      // start de meting en zend een trigger puls van 10mSec.
      noInterrupts();
      digitalWrite(PIN_WIRED_OUT_1, LOW);
      delayMicroseconds(2);
      digitalWrite(PIN_WIRED_OUT_1, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_WIRED_OUT_1, LOW);
      
      // meet de tijd van de echo puls. Uit dit gegeven berekenen we de afstand.
      float distance=pulseIn(PIN_WIRED_OUT_1+1,HIGH);
      interrupts();
      
      distance=distance/58;

      event->Type         = NODO_TYPE_COMMAND;
      event->Command      = CMD_VARIABLE_SET;                 // Commando "VariableSet"
      event->Par2         = float2ul(distance);   // Waarde terugstoppen in de variabele
      success=true;
      break;
      }
      
    case PLUGIN_INIT:
      {
      pinMode(PIN_WIRED_OUT_1  , OUTPUT); // TRIG
      pinMode(PIN_WIRED_OUT_1+1, INPUT ); // ECHO
      break;
      }
    #endif // CORE
    
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_22)==0)
          {
          if(event->Par1>0 && event->Par1<=USER_VARIABLES_MAX)
            {
            success=true;
            event->Type = NODO_TYPE_PLUGIN_COMMAND;
            event->Command = 22; // Plugin nummer  
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME_22);            // Eerste argument=het commando deel
      strcat(string," ");
      strcat(string,int2str(event->Par1));
      strcat(string,",");
      strcat(string,int2str(event->Par2));
      break;
      }
    #endif //CORE_22
    }
  return success;
  }
