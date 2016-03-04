//#######################################################################################################
//##################################### Plugin-12 OregonV2  #############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst van Oregon buitensensoren
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 *                      Support THGN132N en code optimalisatie door forumlid: Arendst
 * Support            : www.nodo-domotica.nl
 * Datum              : 5 Jul 2014
 * Versie             : 1.6
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "OregonV2 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technische informatie:
 * Only supports Oregon V2 protocol messages, message format consists of 18 or 21 nibbles:
 * THN132
 * AAAA AAAA AAAA AAAA BBBB CCCC CCCC CCCC CCCC DDDD EEEE EEEE FFFF GGGG GGGG GGGG HHHH IIII IIII
 *                          0    1    2    3    4    5    6    7    8    9    10   11   12   13
 *   A = preamble, B = sync bits, C = ID, D = Channel, E = RC, F = Flags,
 *   G = Measured value, 3 digits BCD encoded, H = sign, bit3 set if negative temperature
 *   I = Checksum, sum of nibbles C,D,E,F,G,H
 *
 * THGN132
 * AAAA AAAA AAAA AAAA BBBB CCCC CCCC CCCC CCCC DDDD EEEE EEEE FFFF GGGG GGGG GGGG HHHH IIII IIII JJJJ KKKK KKKK
 *                          0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
 *   A = preamble, B = sync bits, C = ID, D = Channel, E = RC, F = Flags,
 *   G = Measured value, 3 digits BCD encoded, H = sign, bit3 set if negative temperature
 *   I = Humidity value
 *   J = Unknown
 *   K = Checksum, sum of nibbles C,D,E,F,G,H,I,J
 **********************************************************************************************/
 
#define PLUGIN_ID 12
#define PLUGIN_NAME   "OregonV2"

#define THN132N_ID              1230
#define THGN123N_ID              721
#define THGR810_ID             17039
#define THN132N_MIN_PULSECOUNT   196
#define THN132N_MAX_PULSECOUNT   206
#define THGN123N_MIN_PULSECOUNT  225
#define THGN123N_MAX_PULSECOUNT  240

byte ProtocolOregonCheckID(byte checkID);

byte ProtocolOregonValidID[5];
byte ProtocolOregonVar[5];

boolean Plugin_012(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  switch(function)
  {
#ifdef PLUGIN_012_CORE
  case PLUGIN_RAWSIGNAL_IN:
    {
      byte nibble[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      byte y = 1;
      byte c = 1;
      byte rfbit = 1;
      byte sync = 0; 
      int id = 0;
      byte checksum = 0;
      byte checksumcalc = 0;
      int datavalue = 0;
      byte basevar=0;

      if (!((RawSignal.Number >= THN132N_MIN_PULSECOUNT && RawSignal.Number <= THN132N_MAX_PULSECOUNT) || (RawSignal.Number >= THGN123N_MIN_PULSECOUNT && RawSignal.Number <= THGN123N_MAX_PULSECOUNT))) return false;
  
      for(byte x=1;x<=RawSignal.Number;x++)
      {
        if(RawSignal.Pulses[x]*RawSignal.Multiply < 600)
        {
          rfbit = (RawSignal.Pulses[x]*RawSignal.Multiply < RawSignal.Pulses[x+1]*RawSignal.Multiply);
          x++;
          y = 2;
        }
        if (y%2 == 1)
        {
          // Find sync pattern as THN132N and THGN123N have different preamble length
          if (c == 1)
          {
            sync = (sync >> 1) | (rfbit << 3);
            sync = sync & 0xf;
            if (sync == 0xA) 
            {
              c = 2;
              if (x < 40) return false;
            }
          }
          else
          {
            if (c < 70) nibble[(c-2)/4] = (nibble[(c-2)/4] >> 1) | rfbit << 3;
            c++;
          }
        }
        y++;
      }
      // if no sync pattern match found, return
      if (c == 1) return false;
      
      // calculate sensor ID
      id = (nibble[3] << 16) |(nibble[2] << 8) | (nibble[1] << 4) | nibble[0];
 
      // calculate and verify checksum
      for(byte x=0; x<12;x++) checksumcalc += nibble[x];
      checksum = (nibble[13] << 4) | nibble[12];

      if ((id == THGN123N_ID) || (id == THGR810_ID))                           // Units with humidity sensor have extra data
        {
          for(byte x=12; x<15;x++) checksumcalc += nibble[x];
          checksum = (nibble[16] << 4) | nibble[15];
        }

      if (checksum != checksumcalc) return false;
  
      basevar = ProtocolOregonCheckID((nibble[6] << 4) | nibble[5]);

      event->Par1          = (nibble[6] << 4) | nibble[5];
      event->Par2          = basevar;
      event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
      event->Port          = VALUE_SOURCE_RF;
      event->Type          = NODO_TYPE_PLUGIN_EVENT;
      event->Command       = 12; // Nummer van dit device

      if (basevar == 0) return true;
  
      // if valid sensor type, update user variable and process event
      if ((id == THGN123N_ID) || (id == THGR810_ID) || (id == THN132N_ID))
        {
        datavalue = ((1000 * nibble[10]) + (100 * nibble[9]) + (10 * nibble[8]));
        if ((nibble[11] & 0x8) == 8) datavalue = -1 * datavalue;
        UserVar[basevar -1] = (float)datavalue/100;
        if ((id == THGN123N_ID) || (id == THGR810_ID))
          {
          datavalue = ((1000 * nibble[13]) + (100 * nibble[12]));
          UserVar[basevar] = (float)datavalue/100;
          }
        }
      success = true;
      break;
    }
 
  case PLUGIN_COMMAND:
    {
    if ((event->Par2 > 0) && (ProtocolOregonCheckID(event->Par1) == 0))
      {
      for (byte x=0; x<5; x++)
        {
        if (ProtocolOregonValidID[x] == 0)
          {
          ProtocolOregonValidID[x] = event->Par1;
          ProtocolOregonVar[x] = event->Par2;
          success=true;
          break;
          }
        }
      success=true;
      }
    break;
    }
#endif // PLUGIN_CORE_012

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
          event->Command = 12; // Plugin nummer  
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

#ifdef PLUGIN_012_CORE
/*********************************************************************************************\
 * Check for valid sensor ID
 \*********************************************************************************************/
byte ProtocolOregonCheckID(byte checkID)
{
  for (byte x=0; x<5; x++) if (ProtocolOregonValidID[x] == checkID) return ProtocolOregonVar[x];
  return 0;
}
#endif //CORE
