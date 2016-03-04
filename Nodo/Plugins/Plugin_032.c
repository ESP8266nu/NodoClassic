//#######################################################################################################
//#################################### Plugin-032: RFScanner ############################################
//#######################################################################################################

/*********************************************************************************************\
 * Deze plugin geeft toegang tot een dedicated RF scanner voor bekende RF berichten
 * 
 * Auteur             : Martinus van den Broek
 * Support            : www.nodo-domotica.nl
 * Datum              : 5 Jul 2013
 * Versie             : 1.4
 * Nodo productnummer : SWACDE-32-V10
 * Compatibiliteit    : Vanaf Nodo build nummer 596
 \*********************************************************************************************/

void Plugin_032_DisplayStats();
void Plugin_032_bandwidthUsage(void);
boolean Plugin_032_AnalyzeRawSignal(byte mode);
boolean Plugin_032_Nodo();
boolean Plugin_032_ClassicNodo();
boolean Plugin_032_kaku();
boolean Plugin_032_newkaku();
boolean Plugin_032_alectov1();
boolean Plugin_032_alectov2();
boolean Plugin_032_alectov3();
uint8_t Plugin_032_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len);
boolean Plugin_032_oregonv2();
boolean Plugin_032_flamengofa20rf();
boolean Plugin_032_homeeasy();

#define PLUGIN_NAME "RFScanner"
#define PLUGIN_ID   32

prog_char PROGMEM Plugin_032_Text_01[] = "RF Scanner Plugin V1.0";
prog_char PROGMEM Plugin_032_Text_02[] = "Licensed under GNU General Public License.";
prog_char PROGMEM Plugin_032_Text_03[] = "n - Normal scan mode";
prog_char PROGMEM Plugin_032_Text_04[] = "p - Normal scan mode with packet details";
prog_char PROGMEM Plugin_032_Text_05[] = "r - Show Signal Ratio";
prog_char PROGMEM Plugin_032_Text_06[] = "c - Show Packet Count";
prog_char PROGMEM Plugin_032_Text_07[] = "q - Quit";
prog_char PROGMEM Plugin_032_Text_08[] = "";
prog_char PROGMEM Plugin_032_Text_09[] = "";
prog_char PROGMEM Plugin_032_Text_10[] = "";
prog_char PROGMEM Plugin_032_Text_11[] = "Signal Ratio:";
prog_char PROGMEM Plugin_032_Text_12[] = "%, Pulsecount:";
prog_char PROGMEM Plugin_032_Text_13[] = ", Shortest:";
prog_char PROGMEM Plugin_032_Text_14[] = ", Longest:";
prog_char PROGMEM Plugin_032_Text_15[] = "**********************************************************************";

prog_char PROGMEM Plugin_032_Protocol_01[] = "Nodo V2";
prog_char PROGMEM Plugin_032_Protocol_02[] = "Nodo V1";
prog_char PROGMEM Plugin_032_Protocol_03[] = "KAKU V1";
prog_char PROGMEM Plugin_032_Protocol_04[] = "KAKU V2";
prog_char PROGMEM Plugin_032_Protocol_05[] = "Alecto V1-Temp";
prog_char PROGMEM Plugin_032_Protocol_06[] = "Alecto V1-Rain";
prog_char PROGMEM Plugin_032_Protocol_07[] = "Alecto V1-Wspeed";
prog_char PROGMEM Plugin_032_Protocol_08[] = "Alecto V1-Wdir";
prog_char PROGMEM Plugin_032_Protocol_09[] = "Alecto V2";
prog_char PROGMEM Plugin_032_Protocol_10[] = "Alecto V3";
prog_char PROGMEM Plugin_032_Protocol_11[] = "Oregon V2";
prog_char PROGMEM Plugin_032_Protocol_12[] = "Flamengo FA20RF";
prog_char PROGMEM Plugin_032_Protocol_13[] = "Home Easy 300EU";
prog_char PROGMEM Plugin_032_Protocol_14[] = "Unknown";

PROGMEM const char *Plugin_032_ProtocolText_tabel[]={Plugin_032_Protocol_01,Plugin_032_Protocol_02,Plugin_032_Protocol_03,Plugin_032_Protocol_04,Plugin_032_Protocol_05,Plugin_032_Protocol_06,Plugin_032_Protocol_07,Plugin_032_Protocol_08,Plugin_032_Protocol_09,Plugin_032_Protocol_10,Plugin_032_Protocol_11,Plugin_032_Protocol_12,Plugin_032_Protocol_13,Plugin_032_Protocol_14};
int Plugin_032_count_protocol[14];

boolean Plugin_032_active=false;
unsigned long Plugin_032_timer=millis();
byte Plugin_032_mode=1;
unsigned long Plugin_032_scantimer=0;

boolean Plugin_032(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  
  switch(function)
    {    
    #ifdef PLUGIN_032_CORE

    case PLUGIN_INIT:
      {
        break;
      }

    case PLUGIN_ONCE_A_SECOND:
      {
      #if !NODO_MEGA // Small has no MMI
        if(Serial.available()) if (Serial.read() =='r') Plugin_032_active=true;
      #endif

      if(Plugin_032_active)
        {
          Serial.println(ProgmemString(Plugin_032_Text_15));
          Serial.println(ProgmemString(Plugin_032_Text_01));
          Serial.println(ProgmemString(Plugin_032_Text_02));
          Serial.println();
          Serial.println("Commands:");
          Serial.println(ProgmemString(Plugin_032_Text_03));
          Serial.println(ProgmemString(Plugin_032_Text_04));
          Serial.println(ProgmemString(Plugin_032_Text_05));
          Serial.println(ProgmemString(Plugin_032_Text_06));
          Serial.println(ProgmemString(Plugin_032_Text_15));

          Plugin_032_mode=1;
          char Plugin_032_command=0;
          for (byte x=0;x<14;x++) Plugin_032_count_protocol[x]=0;

          Plugin_032_scantimer=millis();

          while(Plugin_032_command != 'q')
            {
              // Dedicated Sniffer loop!
              if (Plugin_032_command == 'n') Plugin_032_mode=1;
              if (Plugin_032_command == 'p') Plugin_032_mode=2;
              if (Plugin_032_command == 'r') Plugin_032_mode=3;
              if (Plugin_032_command == 'c') Plugin_032_DisplayStats();
              if (Plugin_032_mode <= 2) if((*portInputRegister(RFport)&RFbit)==RFbit) if(FetchSignal(PIN_RF_RX_DATA,HIGH)) Plugin_032_AnalyzeRawSignal(Plugin_032_mode);
              if (Plugin_032_mode == 3) Plugin_032_bandwidthUsage();
              if(Serial.available()) Plugin_032_command=Serial.read();
            }
          Serial.println("RFScanner disabled!");
          Plugin_032_active=false;
        }
      break;
      }
      
    case PLUGIN_COMMAND:
      {
        Plugin_032_active=true;
        success=true;
        break;
      }      
    #endif // CORE
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {

      char *TempStr=(char*)malloc(26);
      string[25]=0;

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
            event->Type    = NODO_TYPE_PLUGIN_COMMAND;
            event->Command = 32; // nummer van dit plugin
            success=true;
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME);               // Commando 
      break;
      }
    #endif //MMI
    }
    
  return success;
  }

#ifdef PLUGIN_032_CORE
/*********************************************************************************************\
 * Display counters
\*********************************************************************************************/

void Plugin_032_DisplayStats()
{
  char* str=(char*)malloc(80);
  Serial.println();
  Serial.println(ProgmemString(Plugin_032_Text_15));
  Serial.println("Counters:");
  
  for (byte x=0; x < 14; x++)
    {
      strcpy_P(str,(char*)pgm_read_word(&(Plugin_032_ProtocolText_tabel[x])));
      Serial.print(str);
      Serial.print(":");
      Serial.println(Plugin_032_count_protocol[x]);
    }
    Serial.print("Scanning for ");
    Serial.print(round((millis()-Plugin_032_scantimer)/1000));
    Serial.println(" Seconds");

  Serial.println(ProgmemString(Plugin_032_Text_15));
  free(str);
}

/*********************************************************************************************\
 * Display signal ratio
\*********************************************************************************************/

void Plugin_032_bandwidthUsage(void)
{
  unsigned long lowcounter=0;
  unsigned long highcounter=0;  
  unsigned long pulscounter=0;
  unsigned long microtimer;
  unsigned long duration;
  unsigned long shortest=999999;
  unsigned long longest=0;
  byte state = 0;
  byte prevstate = 0;

  // wait for signal to go high
  while (((*portInputRegister(RFport)&RFbit) != RFbit)) {
  }

  microtimer = micros();
  for (unsigned long x=0; x < 500000; x++)
  {
    state=0;
    if((*portInputRegister(RFport)&RFbit)==RFbit) state=1;
    if (prevstate != state)
    {
      if (state==0)
      {
        pulscounter++;
        duration = micros()-microtimer;
        microtimer=micros();
        if (duration > longest) longest = duration;
        if (duration < shortest) shortest = duration;
      }
      prevstate = state;
    }
    if (state==1) highcounter++; 
    else lowcounter++;
  }
  Serial.print(ProgmemString(Plugin_032_Text_11));
  Serial.print((100 * highcounter) / (highcounter+lowcounter));
  Serial.print(ProgmemString(Plugin_032_Text_12));
  Serial.print(pulscounter);
  Serial.print(ProgmemString(Plugin_032_Text_13));
  Serial.print(shortest);
  Serial.print("uS");
  Serial.print(ProgmemString(Plugin_032_Text_14));
  Serial.print(longest);
  Serial.println("uS");
}

/*********************************************************************************************\
 * Analyze signal for known protcols
\*********************************************************************************************/

boolean Plugin_032_AnalyzeRawSignal(byte mode)
{
  if(RawSignal.Number==RAW_BUFFER_SIZE)return false;

  Serial.write('+');
  Serial.print(millis()-Plugin_032_timer);
  Plugin_032_timer = millis();

  Serial.print(" Pulses:");
  Serial.print((int)RawSignal.Number);

  if (mode == 1)
    {
      Serial.print(" Protocol: ");

      if(Plugin_032_Nodo()) return true;
      if(Plugin_032_ClassicNodo()) return true;
      if(Plugin_032_kaku()) return true;
      if(Plugin_032_newkaku()) return true;
      if(Plugin_032_alectov1()) return true;
      if(Plugin_032_alectov2()) return true;
      if(Plugin_032_alectov3()) return true;
      if(Plugin_032_oregonv2()) return true;
      if(Plugin_032_flamengofa20rf()) return true;
      if(Plugin_032_homeeasy()) return true;
      Serial.println("?");
      Plugin_032_count_protocol[13]++;
    }

  if (mode == 2)
    {
      Serial.print(" Details: ");
      for (byte x=1; x <= RawSignal.Number; x++)
        {
          Serial.print((int)RawSignal.Pulses[x]*RawSignal.Multiply);
          Serial.print(',');
        }
      Serial.println();
    }
  return false;   
}

/*********************************************************************************************\
 * NODO 96 bit protocol R596 (V3.6)
\*********************************************************************************************/
#define Plugin_032_NODO_PULSE_MID            1000

boolean Plugin_032_Nodo()
  {

  struct DataBlockStruct
    {
      byte Version;
      byte SourceUnit;
      byte DestinationUnit;
      byte Flags;
      byte Type;
      byte Command;
      byte Par1;
      unsigned long Par2;
      byte Checksum;
    };  

  byte b,x,y,z;

  if(RawSignal.Number!=16*sizeof(struct DataBlockStruct)+2)
    return false;
    
  struct DataBlockStruct DataBlock;
  byte *B=(byte*)&DataBlock;
  z=3;

  for(x=0;x<sizeof(struct DataBlockStruct);x++)
    {
    b=0;
    for(y=0;y<=7;y++)
      {
      if((RawSignal.Pulses[z]*RawSignal.Multiply)>Plugin_032_NODO_PULSE_MID)      
        b|=1<<y;
      z+=2;
      }
    *(B+x)=b;
    }
    
    Serial.print(ProgmemString(Plugin_032_Protocol_01));
    Serial.print(", V:");
    Serial.print((int)DataBlock.Version);
    Serial.print(", C:");
    Serial.print((int)DataBlock.Checksum);
    Serial.print(", Home:");
    Serial.print(DataBlock.SourceUnit>>5);
    Serial.print(", Unit:");
    Serial.print(DataBlock.SourceUnit&0x1F);
    Serial.print(", Type:");
    Serial.print(DataBlock.Type);
    Serial.print(", Flags:");
    Serial.print((int)DataBlock.Flags);
    Serial.print(", Cmd:");
    Serial.print((int)DataBlock.Command);
    Serial.print(", Par1:");
    Serial.print((int)DataBlock.Par1);
    Serial.print(", Par2:");
    Serial.print(DataBlock.Par2);
    Serial.println("");
    Plugin_032_count_protocol[0]++;
    return true;
  }

/*********************************************************************************************\
 * NODO 32 bit protocol
\*********************************************************************************************/
boolean Plugin_032_ClassicNodo()
{
  if(RawSignal.Number !=66) return false;

  unsigned long bitstream=0L;
  int x,y,z;

  z=0;
  for(x=3;x<=RawSignal.Number;x+=2)
    {
    if((RawSignal.Pulses[x]*RawSignal.Multiply)>Plugin_032_NODO_PULSE_MID)      
      bitstream|=(long)(1L<<z);
    z++;
    }
    Serial.print(ProgmemString(Plugin_032_Protocol_02));
    Serial.print(", Unit:");
    Serial.print((bitstream>>24)&0xf);
    Serial.print(", Cmd:");
    Serial.print((int)(bitstream>>16)&0xff);
    Serial.print(", Par1:");
    Serial.print((int)(bitstream>>8)&0xff);
    Serial.print(", Par2:");
    Serial.print((int)bitstream&0xff);
    Serial.println("");
    Plugin_032_count_protocol[1]++;
    return true;
}

/*********************************************************************************************\
 * KAKU Classic protocol
\*********************************************************************************************/
#define Plugin_032_VALUE_OFF         150 
#define Plugin_032_VALUE_ON          151
#define Plugin_032_KAKU_CodeLength    12
#define Plugin_032_KAKU_T            350

boolean Plugin_032_kaku()
{
      int i,j;
      unsigned long bitstream=0;
      byte Par1=0;
      byte Par2=0;
      
      if (RawSignal.Number!=(Plugin_032_KAKU_CodeLength*4)+2)return false;
    
      for (i=0; i<Plugin_032_KAKU_CodeLength; i++)
        {
        j=(Plugin_032_KAKU_T*2)/RawSignal.Multiply;
        
        if      (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]<j && RawSignal.Pulses[4*i+4]>j) {bitstream=(bitstream >> 1);} // 0
        else if (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]>j && RawSignal.Pulses[4*i+4]<j) {bitstream=(bitstream >> 1 | (1 << (Plugin_032_KAKU_CodeLength-1))); }// 1
        else if (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]<j && RawSignal.Pulses[4*i+4]<j) {bitstream=(bitstream >> 1); Par1=2;}
        else {return false;} // error
        }
     
      if ((bitstream&0x600)==0x600)
        {
        Par2  = bitstream & 0xFF;
        Par1 |= (bitstream >> 11) & 0x01;
        Serial.print(ProgmemString(Plugin_032_Protocol_03));
        Serial.print(" Address:");
        Serial.write('A' + (Par2 & 0xf));
        if(Par1 & 2) // als 2e bit in commando staat, dan groep.
          Serial.print((int)0);                // Als Groep, dan adres 0       
        else
          Serial.print( (int) ((Par2 & 0xf0) >> 4 ) +1 ); // Anders adres toevoegen             
      
        if(Par1 & 0x01)
          Serial.println(", State:On");  
        else
          Serial.println(", State:Off");  

        Plugin_032_count_protocol[2]++;
          
        return true;
        }
        
  return false;
}

/*********************************************************************************************\
 * KAKU protocol with automatic addressing
\*********************************************************************************************/
#define Plugin_032_NewKAKU_RawSignalLength      132
#define Plugin_032_NewKAKUdim_RawSignalLength   148
#define Plugin_032_NewKAKU_1T                   275        // us
#define Plugin_032_NewKAKU_mT                   500        // us
#define Plugin_032_NewKAKU_4T                  1100        // us
#define Plugin_032_NewKAKU_8T                  2200        // us

boolean Plugin_032_newkaku()
{
      unsigned long bitstream=0L;
      unsigned long address=0;
      boolean Bit;
      int i;
      int P0,P1,P2,P3;
      byte Par1=0;
      
      if (RawSignal.Number==Plugin_032_NewKAKU_RawSignalLength || RawSignal.Number==Plugin_032_NewKAKUdim_RawSignalLength)
        {
        i=3;
        do 
          {
          P0=RawSignal.Pulses[i]    * RawSignal.Multiply;
          P1=RawSignal.Pulses[i+1]  * RawSignal.Multiply;
          P2=RawSignal.Pulses[i+2]  * RawSignal.Multiply;
          P3=RawSignal.Pulses[i+3]  * RawSignal.Multiply;
          
          if     (P0<Plugin_032_NewKAKU_mT && P1<Plugin_032_NewKAKU_mT && P2<Plugin_032_NewKAKU_mT && P3>Plugin_032_NewKAKU_mT)Bit=0; // T,T,T,4T
          else if(P0<Plugin_032_NewKAKU_mT && P1>Plugin_032_NewKAKU_mT && P2<Plugin_032_NewKAKU_mT && P3<Plugin_032_NewKAKU_mT)Bit=1; // T,4T,T,T
          else if(P0<Plugin_032_NewKAKU_mT && P1<Plugin_032_NewKAKU_mT && P2<Plugin_032_NewKAKU_mT && P3<Plugin_032_NewKAKU_mT)       // T,T,T,T
            {
            if(RawSignal.Number!=Plugin_032_NewKAKUdim_RawSignalLength) // no dim-bits
              return false;
            }
          else
            return false; // not valid
            
          if(i<130)
            bitstream=(bitstream<<1) | Bit;
          else
            Par1=(Par1<<1) | Bit;
       
          i+=4;
          }while(i<RawSignal.Number-2);
            
        // Address
        if(bitstream>0xffff)
          address=bitstream &0xFFFFFFCF;
        else
          address=(bitstream>>6)&0xff;
          
        // Command and Dim
        if(i>140)
          Par1++;
        else
          Par1=((bitstream>>4)&0x01)?VALUE_ON:VALUE_OFF;
        Serial.print(ProgmemString(Plugin_032_Protocol_04));
        Serial.print(" Address:");
        Serial.print(address,HEX);
        if(Par1==VALUE_ON)
          Serial.println(", State:On");  
        else if(Par1==VALUE_OFF)
          Serial.println(", State: Off");
        else
          {
          Serial.print(", Dim:");
          Serial.println((int)Par1);
          }
        Plugin_032_count_protocol[3]++;
        return true;
        }
  return false;
}

/*********************************************************************************************\
 * Alecto V1 protocol
\*********************************************************************************************/
#define Plugin_032_WS3500_PULSECOUNT 74
boolean Plugin_032_alectov1()
{
      if (RawSignal.Number != Plugin_032_WS3500_PULSECOUNT) return false;

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

      Serial.print("Alecto V1");
      Serial.print(", ID:");
      Serial.print(rc);

      if ((nibble2 & 0x6) != 6) {

        temperature = (bitstream >> 12) & 0xfff;
        //fix 12 bit signed number conversion
        if ((temperature & 0x800) == 0x800) temperature = temperature - 0x1000;
        Serial.print(", Temp:");
        Serial.print(temperature);

        humidity = (10 * nibble7) + nibble6;
        Serial.print(", Hum:");
        Serial.println(humidity);
        Plugin_032_count_protocol[4]++;
        return true;
      }
      else
      {
        if (nibble3 == 3)
        {
          rain = ((bitstream >> 16) & 0xffff);
          Serial.print(", Rain:");
          Serial.println(rain);
          Plugin_032_count_protocol[5]++;
          return true;
        }

        if (nibble3 == 1)
        {
          windspeed = ((bitstream >> 24) & 0xff);
          Serial.print(", WindSpeed:");
          Serial.println(windspeed);
          Plugin_032_count_protocol[6]++;
          return true;
        }

        if ((nibble3 & 0x7) == 7)
        {
          winddirection = ((bitstream >> 15) & 0x1ff) / 45;
          Serial.print(", WindDir:");
          Serial.print(winddirection);
          windgust = ((bitstream >> 24) & 0xff);
          Serial.print(", WindGust:");
          Serial.println(windgust);
          Plugin_032_count_protocol[7]++;
          return true;
        }
      }
      return true;
}


/*********************************************************************************************\
 * Alecto V2 protocol
\*********************************************************************************************/
#define Plugin_032_DKW2012_PULSECOUNT 176
#define Plugin_032_ACH2010_MIN_PULSECOUNT 160 // reduce this value (144?) in case of bad reception
#define Plugin_032_ACH2010_MAX_PULSECOUNT 160
boolean Plugin_032_alectov2()
{
      if (!(((RawSignal.Number >= Plugin_032_ACH2010_MIN_PULSECOUNT) && (RawSignal.Number <= Plugin_032_ACH2010_MAX_PULSECOUNT)) || (RawSignal.Number == Plugin_032_DKW2012_PULSECOUNT))) return false;

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

      if(RawSignal.Number > Plugin_032_ACH2010_MAX_PULSECOUNT) maxidx = 9;  
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
      checksumcalc = Plugin_032_ProtocolAlectoCRC8(data, maxidx);
  
      msgtype = (data[0] >> 4) & 0xf;
      rc = (data[0] << 4) | (data[1] >> 4);

      if (checksum != checksumcalc) return false;
  
      if ((msgtype != 10) && (msgtype != 5)) return true;
      Serial.print(ProgmemString(Plugin_032_Protocol_09));
      Serial.print(", ID:");
      Serial.print(rc);

      Serial.print(", Temp:");
      Serial.print((float)(((data[1] & 0x3) * 256 + data[2]) - 400) / 10);
      Serial.print(", Hum:");
      Serial.print((float)data[3]);
      rain = (data[6] * 256) + data[7];
      Serial.print(", Rain:");
      Serial.print(rain);

      Serial.print(", WindSpeed:");
      Serial.print((float)data[4] * 1.08);
      Serial.print(", WindGust:");
      Serial.print((float)data[5] * 1.08);

      if (RawSignal.Number == Plugin_032_DKW2012_PULSECOUNT)
        {
          Serial.print(", WindDir:");
          Serial.print((float)(data[8] & 0xf));
        }
   Serial.println("");        
   Plugin_032_count_protocol[8]++;
   return true;
}


/*********************************************************************************************\
 * Alecto V3 protocol
\*********************************************************************************************/
#define Plugin_032_WS1100_PULSECOUNT 94
#define Plugin_032_WS1200_PULSECOUNT 126
boolean Plugin_032_alectov3()
{
      if ((RawSignal.Number != Plugin_032_WS1100_PULSECOUNT) && (RawSignal.Number != Plugin_032_WS1200_PULSECOUNT)) return false;

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

      if (RawSignal.Number == Plugin_032_WS1200_PULSECOUNT)
      {
        checksum = (bitstream2 >> 8) & 0xff;
        checksumcalc = Plugin_032_ProtocolAlectoCRC8(data, 6);
      }
      else
      {
        checksum = (bitstream2 >> 24) & 0xff;
        checksumcalc = Plugin_032_ProtocolAlectoCRC8(data, 4);
      }

      rc = (bitstream1 >> 20) & 0xff;

      if (checksum != checksumcalc) return false;

      Serial.print(ProgmemString(Plugin_032_Protocol_10));
      Serial.print(", ID:");
      Serial.print(rc);

      temperature = ((bitstream1 >> 8) & 0x3ff) - 400;
      Serial.print(", Temp:");
      Serial.print((float)temperature/10);

      if (RawSignal.Number == Plugin_032_WS1200_PULSECOUNT)
      {
        rain = (((bitstream2 >> 24) & 0xff) * 256) + ((bitstream1 >> 0) & 0xff);
        Serial.print(", Rain:");
        Serial.print((float)rain * 0.30);
      }
      else
      {
        humidity = bitstream1 & 0xff;
        Serial.print(", Hum:");
        Serial.print((float)humidity/10);
      }
    Serial.println("");
    Plugin_032_count_protocol[9]++;
    return true;
}

uint8_t Plugin_032_ProtocolAlectoCRC8( uint8_t *addr, uint8_t len)
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
 * Oregon V2 protocol
\*********************************************************************************************/
#define Plugin_032_THN132N_ID              1230
#define Plugin_032_THGN123N_ID              721
#define Plugin_032_THGR810_ID             17039
#define Plugin_032_THN132N_MIN_PULSECOUNT   196
#define Plugin_032_THN132N_MAX_PULSECOUNT   206
#define Plugin_032_THGN123N_MIN_PULSECOUNT  228
#define Plugin_032_THGN123N_MAX_PULSECOUNT  238
boolean Plugin_032_oregonv2()
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

      if (!((RawSignal.Number >= Plugin_032_THN132N_MIN_PULSECOUNT && RawSignal.Number <= Plugin_032_THN132N_MAX_PULSECOUNT) || (RawSignal.Number >= Plugin_032_THGN123N_MIN_PULSECOUNT && RawSignal.Number <= Plugin_032_THGN123N_MAX_PULSECOUNT))) return false;

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
          // Find sync pattern as THN132N and THGN132N have different preamble length
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
      if ((id == Plugin_032_THGN123N_ID) || (id == Plugin_032_THGR810_ID))                           // Units with humidity sensor have extra data
        {
          for(byte x=12; x<15;x++) checksumcalc += nibble[x];
          checksum = (nibble[16] << 4) | nibble[15];
        }
      if (checksum != checksumcalc) return false;

      Serial.print(ProgmemString(Plugin_032_Protocol_11));
      Serial.print(", ID:");
      Serial.print((nibble[6] << 4) | nibble[5]);

      // if valid sensor type, update user variable and process event
      if ((id == Plugin_032_THGN123N_ID) || (id == Plugin_032_THGR810_ID) || (id == Plugin_032_THN132N_ID))
      {
        datavalue = ((1000 * nibble[10]) + (100 * nibble[9]) + (10 * nibble[8]));
        if ((nibble[11] & 0x8) == 8) datavalue = -1 * datavalue;
        Serial.print(", Temp:");
        Serial.print((float)datavalue/100);
        if ((id == Plugin_032_THGN123N_ID) || (id == Plugin_032_THGR810_ID))
        {
          datavalue = ((1000 * nibble[13]) + (100 * nibble[12]));
          Serial.print(", Hum:");
          Serial.print((float)datavalue/100);
        }
      }
  Serial.println("");
  Plugin_032_count_protocol[10]++;
  return true;
}


/*********************************************************************************************\
 * Flamengo FA20RF protocol
\*********************************************************************************************/
boolean Plugin_032_flamengofa20rf()
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

    Serial.print(ProgmemString(Plugin_032_Protocol_12));
    Serial.print(", ID:");
    Serial.println(bitstream,HEX);
    Plugin_032_count_protocol[11]++;
    return true;
}


/*********************************************************************************************\
 * Home Easy EU protocol
\*********************************************************************************************/
boolean Plugin_032_homeeasy()
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
      state = (bitstream >> 8) & 0x3;
      channel = (bitstream) & 0x3f;

      // Add channel info to base address, first shift channel info 6 positions, so it can't interfere with bit 5
      channel = channel << 6;
      address = address + channel;

      // Set bit 5 based on command information in the Home Easy protocol
      if (state == 1) address = address & 0xFFFFFEF;
      else address = address | 0x00000010;

      Serial.print(ProgmemString(Plugin_032_Protocol_13));
      Serial.print(", Address:");
      Serial.print(address);
      if (state == 0)
        Serial.println(", State:On");
      else
        Serial.println(", State:Off");

      Plugin_032_count_protocol[12]++;
      return true;      
}

#if !NODO_MEGA
/**********************************************************************************************\
 * Deze functie haalt een tekst op uit PROGMEM en geeft als string terug
 \*********************************************************************************************/
char* ProgmemString(prog_char* text)
  {
  byte x=0;
  static char buffer[90]; 

  buffer[0]=0;
  do
    {
    buffer[x]=pgm_read_byte_near(text+x);
    }while(buffer[x++]!=0);
    
  return buffer;
  }
#endif

#endif //core
