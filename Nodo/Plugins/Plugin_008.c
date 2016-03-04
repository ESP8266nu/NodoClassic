//#######################################################################################################
//##################################### Plugin-08 AlectoV1  #############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van Alecto weerstation buitensensoren met protocol V1
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Jan 2014
 * Versie             : 1.3
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "AlectoV1 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technische informatie:
 * Message Format: (9 nibbles, 36 bits):
 *
 * Format for Temperature Humidity
 *   AAAAAAAA BBBB CCCC CCCC CCCC DDDDDDDD EEEE
 *   RC       Type Temperature___ Humidity Checksum
 *   A = Rolling Code
 *   B = Message type (xyyx = temp/humidity if yy <> '11')
 *   C = Temperature (two's complement)
 *   D = Humidity BCD format
 *   E = Checksum
 *
 * Format for Rain
 *   AAAAAAAA BBBB CCCC DDDD DDDD DDDD DDDD EEEE
 *   RC       Type      Rain                Checksum
 *   A = Rolling Code
 *   B = Message type (xyyx = NON temp/humidity data if yy = '11')
 *   C = fixed to 1100
 *   D = Rain (bitvalue * 0.25 mm)
 *   E = Checksum
 *
 * Format for Windspeed
 *   AAAAAAAA BBBB CCCC CCCC CCCC DDDDDDDD EEEE
 *   RC       Type                Windspd  Checksum
 *   A = Rolling Code
 *   B = Message type (xyyx = NON temp/humidity data if yy = '11')
 *   C = Fixed to 1000 0000 0000
 *   D = Windspeed  (bitvalue * 0.2 m/s, correction for webapp = 3600/1000 * 0.2 * 100 = 72)
 *   E = Checksum
 *
 * Format for Winddirection & Windgust
 *   AAAAAAAA BBBB CCCD DDDD DDDD EEEEEEEE FFFF
 *   RC       Type      Winddir   Windgust Checksum
 *   A = Rolling Code
 *   B = Message type (xyyx = NON temp/humidity data if yy = '11')
 *   C = Fixed to 111
 *   D = Wind direction
 *   E = Windgust (bitvalue * 0.2 m/s, correction for webapp = 3600/1000 * 0.2 * 100 = 72)
 *   F = Checksum
 \*********************************************************************************************/
 
#define PLUGIN_ID 8
#define PLUGIN_NAME "AlectoV1"

#define WS3500_PULSECOUNT 74

byte Plugin_008_ProtocolAlectoCheckID(byte checkID);

byte Plugin_008_ProtocolAlectoValidID[5];
byte Plugin_008_ProtocolAlectoVar[5];
unsigned int Plugin_008_ProtocolAlectoRainBase=0;

boolean Plugin_008(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_008_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      if (RawSignal.Number != WS3500_PULSECOUNT) return false;

      RawSignal.Repeats    = false;                                            // het is een herhalend signaal. Bij ontvangst herhalingen NIET onderdrukken.

      unsigned long bitstream=0;
      byte nibble0=0;
      byte nibble1=0;
      byte nibble2=0;
      byte nibble3=0;
      byte nibble4=0;
      byte nibble5=0;
      byte nibble6=0;
      byte nibble7=0;
      byte checksum=0;
      int temperature=0;
      byte humidity=0;
      unsigned int rain=0;
      byte windspeed=0;
      byte windgust=0;
      int winddirection=0;
      byte checksumcalc = 0;
      byte rc=0;
      byte basevar=0;

      for(byte x=2; x<=64; x=x+2)
        {
        if(RawSignal.Pulses[x]*RawSignal.Multiply > 0xA00) bitstream = ((bitstream >> 1) |(0x1L << 31)); 
        else bitstream = (bitstream >> 1);
        }

      for(byte x=66; x<=72; x=x+2)
        {
        if(RawSignal.Pulses[x]*RawSignal.Multiply > 0xA00) checksum = ((checksum >> 1) |(0x1L << 3)); 
        else checksum = (checksum >> 1);
        }

      nibble7 = (bitstream >> 28) & 0xf;
      nibble6 = (bitstream >> 24) & 0xf;
      nibble5 = (bitstream >> 20) & 0xf;
      nibble4 = (bitstream >> 16) & 0xf;
      nibble3 = (bitstream >> 12) & 0xf;
      nibble2 = (bitstream >> 8) & 0xf;
      nibble1 = (bitstream >> 4) & 0xf;
      nibble0 = bitstream & 0xf;

      // checksum calculations
      if ((nibble2 & 0x6) != 6) {
        checksumcalc = (0xf - nibble0 - nibble1 - nibble2 - nibble3 - nibble4 - nibble5 - nibble6 - nibble7) & 0xf;
      }
      else
        {
        // Alecto checksums are Rollover Checksums by design!
        if (nibble3 == 3)
          checksumcalc = (0x7 + nibble0 + nibble1 + nibble2 + nibble3 + nibble4 + nibble5 + nibble6 + nibble7) & 0xf;
        else
          checksumcalc = (0xf - nibble0 - nibble1 - nibble2 - nibble3 - nibble4 - nibble5 - nibble6 - nibble7) & 0xf;
        }

      if (checksum != checksumcalc) return false;
      rc = bitstream & 0xff;

      basevar = Plugin_008_ProtocolAlectoCheckID(rc);

      event->Par1=rc;
      event->Par2=basevar;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 8; // Nummer van dit device

      if (basevar == 0) return true;

      if ((nibble2 & 0x6) != 6) {

        temperature = (bitstream >> 12) & 0xfff;
        //fix 12 bit signed number conversion
        if ((temperature & 0x800) == 0x800) temperature = temperature - 0x1000;
        UserVar[basevar -1] = (float)temperature/10;

        humidity = (10 * nibble7) + nibble6;
        UserVar[basevar +1 -1] = (float)humidity;
        RawSignal.Number=0;
        return true;
      }
      else
      {
        if (nibble3 == 3)
        {
          rain = ((bitstream >> 16) & 0xffff);
          // check if rain unit has been reset!
          if (rain < Plugin_008_ProtocolAlectoRainBase) Plugin_008_ProtocolAlectoRainBase=rain;
          if (Plugin_008_ProtocolAlectoRainBase > 0)
          {
            UserVar[basevar +2 -1] += ((float)rain - Plugin_008_ProtocolAlectoRainBase) * 0.25;
          }
          Plugin_008_ProtocolAlectoRainBase = rain;
          return true;
        }

        if (nibble3 == 1)
        {
          windspeed = ((bitstream >> 24) & 0xff);
          UserVar[basevar +3 -1] = (float)windspeed * 0.72;
          RawSignal.Number=0;
          return true;
        }

        if ((nibble3 & 0x7) == 7)
        {
          winddirection = ((bitstream >> 15) & 0x1ff) / 45;
          UserVar[basevar +4 -1] = (float)winddirection;

          windgust = ((bitstream >> 24) & 0xff);
          UserVar[basevar +5 -1] = (float)windgust * 0.72;

          RawSignal.Number=0;
          return true;
        }
      }
      success = true;
      break;
    }
  case PLUGIN_COMMAND:
    {
    if ((event->Par2 > 0) && (Plugin_008_ProtocolAlectoCheckID(event->Par1) == 0))
      {
      for (byte x=0; x<5; x++)
        {
        if (Plugin_008_ProtocolAlectoValidID[x] == 0)
          {
          Plugin_008_ProtocolAlectoValidID[x] = event->Par1;
          Plugin_008_ProtocolAlectoVar[x] = event->Par2;
          success = true;
          break;
          }
        }
      }
    break;
    }
#endif // PLUGIN_008_CORE

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
          event->Command = 8; // Plugin nummer  
          event->Type = NODO_TYPE_PLUGIN_COMMAND;
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

#ifdef PLUGIN_008_CORE
/*********************************************************************************************\
 * Check for valid sensor ID
 \*********************************************************************************************/
byte Plugin_008_ProtocolAlectoCheckID(byte checkID)
{
  for (byte x=0; x<5; x++) if (Plugin_008_ProtocolAlectoValidID[x] == checkID) return Plugin_008_ProtocolAlectoVar[x];
  return 0;
}
#endif //CORE