//#######################################################################################################
//#################################### Plugin-026: ExtWiredIn ###########################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor uitlezen van de PCF8574 I2C IO-Expander (fabrikant NXP/Philips)
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.Plugin
 * Support            : www.nodo-domotica.Plugin
 * Datum              : 25 jan 2014
 * Versie             : 1.1
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 691
 * Syntax             : "ExtWiredIn <Par1:Poort>, <Par2:On/Off>"
 *********************************************************************************************
 * Technische beschrijving:
 *
 * Compiled size      : <grootte> bytes voor een Mega en <grootte> voor een Small.
 * Externe funkties   : <geef hier aan welke funkties worden gebruikt. 
 *
 * De PCF8574 is een IO Expander chip die via de I2C bus moet worden aangesloten
 * Het basis I2C adres = 0x20
 * Elke chip heeft 8 digitale pinnen die we hier als input gebruiken.
 *   i.v.m. technische beperkingen (performance) lezen we op dit moment maar 1 chip uit op het basis adres
 *   daarom worden nu maximaal 8 inputs ondersteund
 \*********************************************************************************************/

#define PLUGIN_NAME_026 "ExtWiredIn"
#define PLUGIN_ID 26

boolean Plugin_026(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  #ifdef PLUGIN_026_CORE
  static byte portValue=0;

  switch(function)
    {
    case PLUGIN_INIT:
    {
      // get the current pin status
      uint8_t address = 0x20;
      WireNodo.requestFrom(address, (uint8_t)0x1);
      if(WireNodo.available()) portValue = WireNodo.read();
      break;
    }
    
    case PLUGIN_ONCE_A_SECOND:
      {
      byte tmpPortValue=0;
      uint8_t address = 0x20;

      // get the current pin status
      WireNodo.requestFrom(address, (uint8_t)0x1);
      if(WireNodo.available())
      {
        tmpPortValue = WireNodo.read();
        if (tmpPortValue != portValue)
          {
            for (byte x=0; x<8; x++)
              {
                if ((portValue & (1 << x)) != (tmpPortValue & (1 << x)))
                  {
                    struct NodoEventStruct tmpEvent;
                    ClearEvent(&tmpEvent);
                    tmpEvent.Type         = NODO_TYPE_PLUGIN_EVENT;
                    tmpEvent.Command      = PLUGIN_ID;
                    tmpEvent.Par1         = x+1;
                    tmpEvent.Par2         = VALUE_ON;
                    if (tmpPortValue & (1 << x)) tmpEvent.Par2 = VALUE_OFF;
                    ProcessEvent(&tmpEvent);
                    //QueueAdd(&tmpEvent);
                  }
              }
            portValue = tmpPortValue;
          }
        success=true;
      }
      break;
      }
    #endif // CORE
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_026)==0)
          {
          if(GetArgv(string,TempStr,2)) 
            {
            if(GetArgv(string,TempStr,3))
              {
              if(event->Par1>0 && event->Par1<65 && (event->Par2==VALUE_ON || event->Par2==VALUE_OFF))            
                {
                  event->Type = NODO_TYPE_PLUGIN_EVENT;
                  event->Command = 26; // Plugin nummer  
                  success=true;
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
      strcpy(string,PLUGIN_NAME_026);
      strcat(string," ");
      strcat(string,int2str(event->Par1));
      strcat(string,",");
      if(event->Par2==VALUE_ON)
        strcat(string,"On");  
      else strcat(string,"Off");
      break;
      }
    #endif //MMI
    }
    
  return success;
  }
