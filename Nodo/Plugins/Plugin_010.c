//#######################################################################################################
//##################################### Plugin-10 AlectoV3  #############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van Alecto weerstation buitensensoren
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 *                      Support WS1100 door forumlid: Arendst
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Jan 2014
 * Versie             : 1.3
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "AlectoV3 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technische informatie:
 * Decodes signals from Alecto Weatherstation outdoor unit, type 3 (94/126 pulses, 47/63 bits, 433 MHz).
 * WS1100 Message Format: (7 bits preamble, 5 Bytes, 40 bits):
 * AAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD EEEEEEEE
 *                        Temperature Humidity Checksum
 * A = start/unknown, first 8 bits are always 11111111
 * B = Rolling code
 * C = Temperature (10 bit value with -400 base)
 * D = Checksum
 * E = Humidity
 *
 * WS1200 Message Format: (7 bits preamble, 7 Bytes, 56 bits):
 * AAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD DDDDDDDD EEEEEEEE FFFFFFFF 
 *                        Temperature Rain LSB Rain MSB ???????? Checksum
 * A = start/unknown, first 8 bits are always 11111111
 * B = Rolling code
 * C = Temperature (10 bit value with -400 base)
 * D = Rain ( * 0.3 mm)
 * E = ?
 * F = Checksum
 \*********************************************************************************************/
 
#define PLUGIN_ID 10
#define PLUGIN_NAME "AlectoV3"

#define WS1100_PULSECOUNT 94
#define WS1200_PULSECOUNT 126

uint8_t Plugin_010_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len);
byte Plugin_010_ProtocolAlectoCheckID(byte checkID);

byte Plugin_010_ProtocolAlectoValidID[5];
byte Plugin_010_ProtocolAlectoVar[5];
unsigned int Plugin_010_ProtocolAlectoRainBase=0;

boolean Plugin_010(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_010_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      if ((RawSignal.Number != WS1100_PULSECOUNT) && (RawSignal.Number != WS1200_PULSECOUNT)) return false;

      unsigned long bitstream1=0;
      unsigned long bitstream2=0;
      byte rc=0;
      int temperature=0;
      byte humidity=0;
      unsigned int rain=0;
      byte checksum=0;
      byte checksumcalc=0;
      byte basevar=0;
      byte data[6];

      // get first 32 relevant bits
      for(byte x=15; x<=77; x=x+2) if(RawSignal.Pulses[x]*RawSignal.Multiply < 0x300) bitstream1 = (bitstream1 << 1) | 0x1; 
      else bitstream1 = (bitstream1 << 1);
      // get second 32 relevant bits
      for(byte x=79; x<=141; x=x+2) if(RawSignal.Pulses[x]*RawSignal.Multiply < 0x300) bitstream2 = (bitstream2 << 1) | 0x1; 
      else bitstream2 = (bitstream2 << 1);

      data[0] = (bitstream1 >> 24) & 0xff;
      data[1] = (bitstream1 >> 16) & 0xff;
      data[2] = (bitstream1 >>  8) & 0xff;
      data[3] = (bitstream1 >>  0) & 0xff;
      data[4] = (bitstream2 >> 24) & 0xff;
      data[5] = (bitstream2 >> 16) & 0xff;

      if (RawSignal.Number == WS1200_PULSECOUNT)
      {
        checksum = (bitstream2 >> 8) & 0xff;
        checksumcalc = Plugin_010_ProtocolAlectoCRC8(data, 6);
      }
      else
      {
        checksum = (bitstream2 >> 24) & 0xff;
        checksumcalc = Plugin_010_ProtocolAlectoCRC8(data, 4);
      }

      rc = (bitstream1 >> 20) & 0xff;

      if (checksum != checksumcalc) return false;
      basevar = Plugin_010_ProtocolAlectoCheckID(rc);

      event->Par1=rc;
      event->Par2=basevar;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 10; // Nummer van dit device

      if (basevar == 0) return true;

      temperature = ((bitstream1 >> 8) & 0x3ff) - 400;
      UserVar[basevar-1] = (float)temperature / 10;

      if (RawSignal.Number == WS1200_PULSECOUNT)
      {
        rain = (((bitstream2 >> 24) & 0xff) * 256) + ((bitstream1 >> 0) & 0xff);
        // check if rain unit has been reset!
        if (rain < Plugin_010_ProtocolAlectoRainBase) Plugin_010_ProtocolAlectoRainBase=rain;
        if (Plugin_010_ProtocolAlectoRainBase > 0)
        {
          UserVar[basevar+1 -1] += ((float)rain - Plugin_010_ProtocolAlectoRainBase) * 0.30;
        }
        Plugin_010_ProtocolAlectoRainBase = rain;
      }
      else
      {
        humidity = bitstream1 & 0xff;
        UserVar[basevar+1 -1] = (float)humidity / 10;
      }

      RawSignal.Number=0;
      success = true;
      break;
    }
  case PLUGIN_COMMAND:
    {
    if ((event->Par2 > 0) && (Plugin_010_ProtocolAlectoCheckID(event->Par1) == 0))
      {
      for (byte x=0; x<5; x++)
        {
        if (Plugin_010_ProtocolAlectoValidID[x] == 0)
          {
          Plugin_010_ProtocolAlectoValidID[x] = event->Par1;
          Plugin_010_ProtocolAlectoVar[x] = event->Par2;
          success=true;
          break;
          }
        }
      }
    break;
    }
#endif // PLUGIN_010_CORE

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
          event->Command = 10; // Plugin nummer  
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

#ifdef PLUGIN_010_CORE
/*********************************************************************************************\
 * Calculates CRC-8 checksum
 * reference http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
 *           http://lucsmall.com/2012/04/30/weather-station-hacking-part-3/
 *           https://github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino/blob/master/WeatherSensorWH2.cpp
 \*********************************************************************************************/
uint8_t Plugin_010_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len)
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
byte Plugin_010_ProtocolAlectoCheckID(byte checkID)
{
  for (byte x=0; x<5; x++) if (Plugin_010_ProtocolAlectoValidID[x] == checkID) return Plugin_010_ProtocolAlectoVar[x];
  return 0;
}
#endif //CORE