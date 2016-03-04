//#######################################################################################################
//##################################### Plugin-13 Flamingo FA20RF Rookmelder ############################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van Flamingo FA20RF rookmelder
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 27 Dec 2013
 * Versie             : 1.3
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 596
 * Syntax             : "SmokeAlert 0, <Par2: rookmelder ID>"
 *********************************************************************************************
 * Technische informatie:
 * De Flamingo FA20RF rookmelder bevat een RF zender en ontvanger. Standaard heeft elke unit een uniek ID
 * De rookmelder heeft een learn knop waardoor hij het ID van een andere unit kan overnemen
 * Daardoor kunnen ze onderling worden gekoppeld.
 * Na het koppelen hebben ze dus allemaal hetzelfde ID!
 * Je gebruikt 1 unit als master, waarvan de je code aanleert aan de andere units (slaves)
 \*********************************************************************************************/
#define PLUGIN_ID 13
#define PLUGIN_NAME "SmokeAlert"

boolean Plugin_013(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_013_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      if (RawSignal.Number != 52) return false;

      unsigned long bitstream=0L;
      for(byte x=4;x<=50;x=x+2)
      {
        if (RawSignal.Pulses[x-1]*RawSignal.Multiply > 1000) return false; // every preceding puls must be < 1000!
        if (RawSignal.Pulses[x]*RawSignal.Multiply > 1800) bitstream = (bitstream << 1) | 0x1; 
        else bitstream = bitstream << 1;
      }

      if (bitstream == 0) return false;

      event->Par1=0;
      event->Par2=bitstream;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 13; // Nummer van dit device

      return true;
      break;
    }
  case PLUGIN_COMMAND:
    {
      break;
    }
#endif // PLUGIN_013_CORE

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
              event->Command = 13; // Plugin nummer  
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
    strcat(string,int2str(event->Par2));
    break;
    }
#endif //NODO_MEGA
  }      
  return success;
}
