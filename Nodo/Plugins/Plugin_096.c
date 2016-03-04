//#######################################################################################################
//##################################### Plugin-96 Xiron  ################################################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides reception/decoding of Xiron weatherstation outdoorsensors
 * 
 * Author             : M vd Broek
 * Date               : 29 Sep 2014 (debugging code removed)
 * Version            : Beta 0.2
 * Compatibility      : R787
 * Syntax             : "XironV1 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technical data:
 * Decodes signals from Xiron Weatherstation outdoor unit
 * Message Format: (84 pulses, 40 bits of data)
 * AAAAAAAA BB CC DDDDDDDDDDDD EEEE FFFFFFFF GGGG
 * ID       ?? Ch Temperature  ?    Humidity ?
 * A = ID (code changes after battery replacement)
 * B = ?
 * C = Channel (1,2,3)
 * D = Temperature (12 bit value)
 * E = ?
 * F = Humidity
 * G = ?
 *
 * RF Message is repeated 5 times, delay between messages is approx 3600 uSec
 * Current Nodo release (R744) cannot receive this because this delay is too short!!!
 * Two possible workarounds:
 *   1) Ajdust SIGNAL_TIMEOUT from default 5 mSec to 3 mSec (side effects unknown !!)
 *   2) Adjust RAW_BUFFER_SIZE from 256 to 512 so entire message burst can be processed
 * The current plugin version can handle both workarounds.
 * Workaround (1) seems to receive better in case of longer distance between Nodo and sensor.
 *
 \*********************************************************************************************/
 
#define PLUGIN_ID 96
#define PLUGIN_NAME "XironV1"

#define XIRON_PULSECOUNT_SINGLE 84 
#define XIRON_PULSECOUNT_REPEATED 508 

byte Plugin_096_ProtocolXironCheckID(byte checkID);

byte Plugin_096_ProtocolXironValidID[5];
byte Plugin_096_ProtocolXironVar[5];

boolean Plugin_096(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_096_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      if ((RawSignal.Number != XIRON_PULSECOUNT_SINGLE) && (RawSignal.Number != XIRON_PULSECOUNT_REPEATED)) return false;

      unsigned long bitstream1=0;
      unsigned long bitstream2=0;
      byte rc=0;
      int temperature=0;
      byte humidity=0;
      byte basevar=0;
      byte offset=2;
      if (RawSignal.Number == XIRON_PULSECOUNT_SINGLE)
        offset=0;

      // get first 28 bits
      for(byte x=(2+offset); x<=(56+offset); x=x+2) if(RawSignal.Pulses[x]*RawSignal.Multiply > 1250) bitstream1 = (bitstream1 << 1) | 0x1; 
      else bitstream1 = (bitstream1 << 1);
      // get remaining 12 bits
      for(byte x=(58+offset); x<=(80+offset); x=x+2) if(RawSignal.Pulses[x]*RawSignal.Multiply > 1250) bitstream2 = (bitstream2 << 1) | 0x1; 
      else bitstream2 = (bitstream2 << 1);

      rc = (bitstream1 >> 20) & 0xff;

      basevar = Plugin_096_ProtocolXironCheckID(rc);

      event->Par1=rc;
      event->Par2=basevar;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = PLUGIN_ID; // Nummer van dit device

      if (basevar == 0) return true;

      temperature = ((bitstream1 >> 4) & 0x3ff);
      UserVar[basevar-1] = (float)temperature / 10;
      humidity = ((bitstream2 >> 4) & 0xff);
      UserVar[basevar+1 -1] = (float)humidity;

      RawSignal.Number=0;
      success = true;
      break;
    }
  case PLUGIN_COMMAND:
    {
    if ((event->Par2 > 0) && (Plugin_096_ProtocolXironCheckID(event->Par1) == 0))
      {
      for (byte x=0; x<5; x++)
        {
        if (Plugin_096_ProtocolXironValidID[x] == 0)
          {
          Plugin_096_ProtocolXironValidID[x] = event->Par1;
          Plugin_096_ProtocolXironVar[x] = event->Par2;
          success=true;
          break;
          }
        }
      }
    break;
    }
#endif // PLUGIN_096_CORE

#if NODO_MEGA
  case PLUGIN_MMI_IN:
    {
    char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

    if(GetArgv(string,TempStr,1))
      {
      if(strcasecmp(TempStr,PLUGIN_NAME)==0)
        {
        if(event->Par1>0 && event->Par1<255 && event->Par2>0 && event->Par2<=USER_VARIABLES_MAX)
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

#ifdef PLUGIN_096_CORE

/*********************************************************************************************\
 * Check for valid sensor ID
 \*********************************************************************************************/
byte Plugin_096_ProtocolXironCheckID(byte checkID)
{
  for (byte x=0; x<5; x++) if (Plugin_096_ProtocolXironValidID[x] == checkID) return Plugin_096_ProtocolXironVar[x];
  return 0;
}
#endif //CORE