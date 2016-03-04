//#######################################################################################################
//#################################### Plugin-15: HomeEasy EU ###########################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst HomeEasy EU zenders
 * die werken volgens de automatische codering (Ontvangers met leer-knop)
 *
 * LET OP: GEEN SUPPORT VOOR DIRECTE DIMWAARDES!!!
 *
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotca.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Mrt 2013
 * Versie             : 1.3
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "HomeEasy <Adres>,<On|Off>
 *********************************************************************************************
 * Technische informatie:
 * Analyses Home Easy Messages and convert these into an eventcode
 * Only new EU devices with automatic code system are supported
 * Only  On / Off status is decoded, no DIM values
 * Only tested with Home Easy HE300WEU transmitter, doorsensor and PIR sensor
 * Home Easy message structure, by analyzing bitpatterns so far ...
 * AAAAAAAAAAABBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBCCCCDDEEFFFFFFG
 *       A = startbits/preamble, B = address, C = ?, D = Command, E = ?, F = Channel, G = Stopbit
 \*********************************************************************************************/
 
#define HomeEasy_LongLow    0x490    // us
#define HomeEasy_ShortHigh  200      // us
#define HomeEasy_ShortLow   150      // us

#define PLUGIN_ID 15
#define PLUGIN_NAME "HomeEasy"

boolean Plugin_015(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
  #ifdef PLUGIN_015_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      unsigned long address = 0;
      unsigned long bitstream = 0;
      int counter = 0;
      byte rfbit =0;
      byte state = 0;
      unsigned long channel = 0;

      // valid messages are 116 pulses          
      if (RawSignal.Number != 116) return false;

      for(byte x=1;x<=RawSignal.Number;x=x+2)
        {
        if ((RawSignal.Pulses[x]*RawSignal.Multiply < 500) & (RawSignal.Pulses[x+1]*RawSignal.Multiply > 500)) 
          rfbit = 1;
        else
          rfbit = 0;

        if ((x>=23) && (x<=86)) address = (address << 1) | rfbit;
        if ((x>=87) && (x<=114)) bitstream = (bitstream << 1) | rfbit;

      }
      state = ((bitstream >> 8) & 0x3)-1;
      channel = (bitstream) & 0x3f;

      // Add channel info to base address, first shift channel info 6 positions, so it can't interfere with bit 5
      channel = channel << 6;
      address = address + channel;

      RawSignal.Repeats    = true; // het is een herhalend signaal. Bij ontvangst herhalingen onderdrukken.
      event->Par1=state?VALUE_ON:VALUE_OFF; // On/Off bit omzetten naar een Nodo waarde. 
      event->Par2=address &0xFFFFFFCF; 
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 15; // Nummer van dit device
      success=true;
      break;
    }

#endif // PLUGIN_015_CORE

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
          event->Par2=str2int(str);    
          if(GetArgv(string,str,3))
            {
            // Vul Par1 met het HomeEasy commando. Dit kan zijn: VALUE_ON, VALUE_OFF, Andere waarden zijn ongeldig.

            // haal uit de tweede parameter een 'On' of een 'Off'.
            if(event->Par1=str2cmd(str))
              {
              event->Type  = NODO_TYPE_PLUGIN_EVENT;
              event->Command = 15; // Plugin nummer                
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
    strcpy(string,PLUGIN_NAME);            // Eerste argument=het commando deel
    strcat(string," ");

    // In Par3 twee mogelijkheden: Het bevat een door gebruiker ingegeven adres 0..255 of een volledig HomeEasy adres.
    if(event->Par2>=0x0ff)
      strcat(string,int2strhex(event->Par2)); 
    else
      strcat(string,int2str(event->Par2)); 

    strcat(string,",");

    if(event->Par1==VALUE_ON)
      strcat(string,"On");  
    else if(event->Par1==VALUE_OFF)
      strcat(string,"Off");
    else
      strcat(string,int2str(event->Par1));

    break;
    }
#endif //NODO_MEGA
  }      
  return success;
}
