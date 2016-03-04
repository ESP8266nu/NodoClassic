//#######################################################################################################
//##################################### Plugin-09 AlectoV2  #############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van Alecto weerstation buitensensoren
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 *                      Support ACH2010 en code optimalisatie door forumlid: Arendst
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Jan 2013
 * Versie             : 1.3
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "AlectoV2 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technische informatie:
 * DKW2012 Message Format: (11 Bytes, 88 bits):
 * AAAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD EEEEEEEE FFFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH IIIIIIII
 *                         Temperature Humidity Windspd_ Windgust Rain____ ________ Winddir  Checksum
 * A = start/unknown, first 8 bits are always 11111111
 * B = Rolling code
 * C = Temperature (10 bit value with -400 base)
 * D = Humidity
 * E = windspeed (* 0.3 m/s, correction for webapp = 3600/1000 * 0.3 * 100 = 108))
 * F = windgust (* 0.3 m/s, correction for webapp = 3600/1000 * 0.3 * 100 = 108))
 * G = Rain ( * 0.3 mm)
 * H = winddirection (0 = north, 4 = east, 8 = south 12 = west)
 * I = Checksum, calculation is still under investigation
 *
 * WS3000 and ACH2010 systems have no winddirection, message format is 8 bit shorter
 * Message Format: (10 Bytes, 80 bits):
 * AAAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD EEEEEEEE FFFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH
 *                         Temperature Humidity Windspd_ Windgust Rain____ ________ Checksum
 * 
 * DCF Time Message Format: (NOT DECODED!, we already have time sync through webapp)
 * AAAAAAAA BBBBCCCC DDDDDDDD EFFFFFFF GGGGGGGG HHHHHHHH IIIIIIII JJJJJJJJ KKKKKKKK LLLLLLLL MMMMMMMM
 * 	    11                 Hours   Minutes  Seconds  Year     Month    Day      ?        Checksum
 * B = 11 = DCF
 * C = ?
 * D = ?
 * E = ?
 * F = Hours BCD format (7 bits only for this byte, MSB could be '1')
 * G = Minutes BCD format
 * H = Seconds BCD format
 * I = Year BCD format (only two digits!)
 * J = Month BCD format
 * K = Day BCD format
 * L = ?
 * M = Checksum
 \*********************************************************************************************/

#define PLUGIN_ID 9
#define PLUGIN_NAME "AlectoV2"

#define DKW2012_PULSECOUNT 176
#define ACH2010_MIN_PULSECOUNT 160 // reduce this value (144?) in case of bad reception
#define ACH2010_MAX_PULSECOUNT 160

byte Plugin_009_ProtocolAlectoCheckID(byte checkID);
uint8_t Plugin_009_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len);

byte Plugin_009_ProtocolAlectoValidID[5];
byte Plugin_009_ProtocolAlectoVar[5];
unsigned int Plugin_009_ProtocolAlectoRainBase=0;

boolean Plugin_009(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_009_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      if (!(((RawSignal.Number >= ACH2010_MIN_PULSECOUNT) && (RawSignal.Number <= ACH2010_MAX_PULSECOUNT)) || (RawSignal.Number == DKW2012_PULSECOUNT))) return false;

      byte c=0;
      byte rfbit;
      byte data[9]; 
      byte msgtype=0;
      byte rc=0;
      unsigned int rain=0;
      byte checksum=0;
      byte checksumcalc=0;
      byte basevar;
      byte maxidx = 8;

      if(RawSignal.Number > ACH2010_MAX_PULSECOUNT) maxidx = 9;  
      // Get message back to front as the header is almost never received complete for ACH2010
      byte idx = maxidx;
      for(byte x=RawSignal.Number; x>0; x=x-2)
        {
          if(RawSignal.Pulses[x-1]*RawSignal.Multiply < 0x300) rfbit = 0x80; else rfbit = 0;  
          data[idx] = (data[idx] >> 1) | rfbit;
          c++;
          if (c == 8) 
          {
            if (idx == 0) break;
            c = 0;
            idx--;
          }   
        }

      checksum = data[maxidx];
      checksumcalc = Plugin_009_ProtocolAlectoCRC8(data, maxidx);
  
      msgtype = (data[0] >> 4) & 0xf;
      rc = (data[0] << 4) | (data[1] >> 4);

      if (checksum != checksumcalc) return false;
  
      basevar = Plugin_009_ProtocolAlectoCheckID(rc);

      event->Par1=rc;
      event->Par2=basevar;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 9; // Nummer van dit device

      if (basevar == 0) return true;
      if ((msgtype != 10) && (msgtype != 5)) return true;
  
      UserVar[basevar -1] = (float)(((data[1] & 0x3) * 256 + data[2]) - 400) / 10;
      UserVar[basevar +1 -1] = (float)data[3];

      rain = (data[6] * 256) + data[7];
      // check if rain unit has been reset!
      if (rain < Plugin_009_ProtocolAlectoRainBase) Plugin_009_ProtocolAlectoRainBase=rain;
      if (Plugin_009_ProtocolAlectoRainBase > 0)
      {
        UserVar[basevar +2 -1] += ((float)rain - Plugin_009_ProtocolAlectoRainBase) * 0.30;
      }
      Plugin_009_ProtocolAlectoRainBase = rain;

      UserVar[basevar +3 -1] = (float)data[4] * 1.08;
      UserVar[basevar +4 -1] = (float)data[5] * 1.08;
      if (RawSignal.Number == DKW2012_PULSECOUNT) UserVar[basevar +5 -1] = (float)(data[8] & 0xf);

      RawSignal.Number=0;
      success = true;
      break;
    }
  case PLUGIN_COMMAND:
    {
    if ((event->Par2 > 0) && (Plugin_009_ProtocolAlectoCheckID(event->Par1) == 0))
      {
      for (byte x=0; x<5; x++)
        {
        if (Plugin_009_ProtocolAlectoValidID[x] == 0)
          {
            Plugin_009_ProtocolAlectoValidID[x] = event->Par1;
            Plugin_009_ProtocolAlectoVar[x] = event->Par2;
            success = true;
            break;
          }
        }
      }
    break;
    }
#endif // PLUGIN_009_CORE

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
          event->Command = 9; // Plugin nummer  
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

#ifdef PLUGIN_009_CORE
/*********************************************************************************************\
 * Calculates CRC-8 checksum
 * reference http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
 *           http://lucsmall.com/2012/04/30/weather-station-hacking-part-3/
 *           https://github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino/blob/master/WeatherSensorWH2.cpp
 \*********************************************************************************************/
uint8_t Plugin_009_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  // Indicated changes are from reference CRC-8 function in OneWire library
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}

/*********************************************************************************************\
 * Check for valid sensor ID
 \*********************************************************************************************/
byte Plugin_009_ProtocolAlectoCheckID(byte checkID)
{
  for (byte x=0; x<5; x++) if (Plugin_009_ProtocolAlectoValidID[x] == checkID) return Plugin_009_ProtocolAlectoVar[x];
  return 0;
}
#endif //CORE