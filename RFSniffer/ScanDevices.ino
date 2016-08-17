/*********************************************************************************************\
 * NODO 96 bit protocol R555 (V3.5)
\*********************************************************************************************/
#define NODO_PULSE_0               500
#define NODO_PULSE_MID            1000
#define NODO_PULSE_1              1500
#define NODO_SPACE                 500

boolean RawSignal_2_Nodo()
  {
  
  struct DataBlockStruct
    {
      byte SourceUnit;
      byte DestinationUnit;
      byte Flags;
      byte Type;
      byte Command;
      byte Par1;
      unsigned long Par2;
      byte Checksum;
    };  

    byte b,c,x,y,z;

  if(RawSignal.Number!=16*sizeof(struct DataBlockStruct)+2)
    return false;
    
  struct DataBlockStruct DataBlock;
  byte *B=(byte*)&DataBlock;
  z=3;

  c=0;
  for(x=0;x<sizeof(struct DataBlockStruct);x++)
    {
    b=0;
    for(y=0;y<=7;y++)
      {
      if((RawSignal.Pulses[z]*RawSignal.Multiply)>NODO_PULSE_MID)      
        b|=1<<y;
      z+=2;
      }
    *(B+x)=b;
    c^=b;  // bereken checksum: crc-8 uit alle bytes. 
    }
    
    Serial.print(ProgmemString(Protocol_01));
    Serial.print(", C:");
    Serial.print((int)c);
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
    count_protocol[0]++;
    return true;
  }

/*********************************************************************************************\
 * NODO 96 bit protocol R596 (V3.6)
\*********************************************************************************************/
boolean RawSignal_2_NodoNew()
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
      if((RawSignal.Pulses[z]*RawSignal.Multiply)>NODO_PULSE_MID)      
        b|=1<<y;
      z+=2;
      }
    *(B+x)=b;
    }
    
    Serial.print(ProgmemString(Protocol_01));
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
    count_protocol[0]++;
    return true;
  }

/*********************************************************************************************\
 * NODO 32 bit protocol
\*********************************************************************************************/
boolean RawSignal_2_ClassicNodo()
{
  if(RawSignal.Number !=66) return false;

  unsigned long bitstream=0L;
  int x,y,z;

  z=0;
  for(x=3;x<=RawSignal.Number;x+=2)
    {
    if((RawSignal.Pulses[x]*RawSignal.Multiply)>NODO_PULSE_MID)      
      bitstream|=(long)(1L<<z);
    z++;
    }
    Serial.print(ProgmemString(Protocol_02));
    Serial.print(", Unit:");
    Serial.print((bitstream>>24)&0xf);
    Serial.print(", Cmd:");
    Serial.print((int)(bitstream>>16)&0xff);
    Serial.print(", Par1:");
    Serial.print((int)(bitstream>>8)&0xff);
    Serial.print(", Par2:");
    Serial.print((int)bitstream&0xff);
    Serial.println("");
    count_protocol[1]++;
    return true;
}

/*********************************************************************************************\
 * KAKU Classic protocol
\*********************************************************************************************/
#define VALUE_OFF         150 
#define VALUE_ON          151
#define KAKU_CodeLength    12
#define KAKU_T            350

boolean kaku()
{
      int i,j;
      unsigned long bitstream=0;
      byte Par1=0;
      byte Par2=0;
      
      if (RawSignal.Number!=(KAKU_CodeLength*4)+2)return false;
    
      for (i=0; i<KAKU_CodeLength; i++)
        {
        j=(KAKU_T*2)/RawSignal.Multiply;
        
        if      (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]<j && RawSignal.Pulses[4*i+4]>j) {bitstream=(bitstream >> 1);} // 0
        else if (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]>j && RawSignal.Pulses[4*i+4]<j) {bitstream=(bitstream >> 1 | (1 << (KAKU_CodeLength-1))); }// 1
        else if (RawSignal.Pulses[4*i+1]<j && RawSignal.Pulses[4*i+2]>j && RawSignal.Pulses[4*i+3]<j && RawSignal.Pulses[4*i+4]<j) {bitstream=(bitstream >> 1); Par1=2;}
        else {return false;} // error
        }
     
      if ((bitstream&0x600)==0x600)
        {
        Par2  = bitstream & 0xFF;
        Par1 |= (bitstream >> 11) & 0x01;
        Serial.print(ProgmemString(Protocol_03));
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

        count_protocol[2]++;
          
        return true;
        }
        
  return false;
}


/*********************************************************************************************\
 * KAKU protocol with automatic addressing
\*********************************************************************************************/
#define NewKAKU_RawSignalLength      132
#define NewKAKUdim_RawSignalLength   148
#define NewKAKU_1T                   275        // us
#define NewKAKU_mT                   500        // us
#define NewKAKU_4T                  1100        // us
#define NewKAKU_8T                  2200        // us
boolean newkaku()
{
      unsigned long bitstream=0L;
      unsigned long address=0;
      boolean Bit;
      int i;
      int P0,P1,P2,P3;
      byte Par1=0;
      
      if (RawSignal.Number==NewKAKU_RawSignalLength || RawSignal.Number==NewKAKUdim_RawSignalLength)
        {
        i=3;
        do 
          {
          P0=RawSignal.Pulses[i]    * RawSignal.Multiply;
          P1=RawSignal.Pulses[i+1]  * RawSignal.Multiply;
          P2=RawSignal.Pulses[i+2]  * RawSignal.Multiply;
          P3=RawSignal.Pulses[i+3]  * RawSignal.Multiply;
          
          if     (P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3>NewKAKU_mT)Bit=0; // T,T,T,4T
          else if(P0<NewKAKU_mT && P1>NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)Bit=1; // T,4T,T,T
          else if(P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)       // T,T,T,T
            {
            if(RawSignal.Number!=NewKAKUdim_RawSignalLength) // no dim-bits
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
        Serial.print(ProgmemString(Protocol_04));
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
        count_protocol[3]++;
        return true;
        }
  return false;
}


/*********************************************************************************************\
 * Alecto V1 protocol
\*********************************************************************************************/
#define WS3500_PULSECOUNT 74
boolean alectov1()
{
      if (RawSignal.Number != WS3500_PULSECOUNT) return false;

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

      Serial.print(ProgmemString(Protocol_05));
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
        count_protocol[4]++;
        return true;
      }
      else
      {
        if (nibble3 == 3)
        {
          rain = ((bitstream >> 16) & 0xffff);
          Serial.print(", Rain:");
          Serial.println(rain);
          count_protocol[4]++;
          return true;
        }

        if (nibble3 == 1)
        {
          windspeed = ((bitstream >> 24) & 0xff);
          Serial.print(", WindSpeed:");
          Serial.println(windspeed);
          count_protocol[4]++;
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
          count_protocol[4]++;
          return true;
        }
      }
      return true;
}


/*********************************************************************************************\
 * Alecto V2 protocol
\*********************************************************************************************/
#define DKW2012_PULSECOUNT 176
#define ACH2010_MIN_PULSECOUNT 160 // reduce this value (144?) in case of bad reception
#define ACH2010_MAX_PULSECOUNT 160
boolean alectov2()
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
      checksumcalc = ProtocolAlectoCRC8(data, maxidx);
  
      msgtype = (data[0] >> 4) & 0xf;
      rc = (data[0] << 4) | (data[1] >> 4);

      if (checksum != checksumcalc) return false;
  
      if ((msgtype != 10) && (msgtype != 5)) return true;
      Serial.print(ProgmemString(Protocol_06));
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

      if (RawSignal.Number == DKW2012_PULSECOUNT)
        {
          Serial.print(", WindDir:");
          Serial.print((float)(data[8] & 0xf));
        }
   Serial.println("");        
   count_protocol[5]++;
   return true;
}


/*********************************************************************************************\
 * Alecto V3 protocol
\*********************************************************************************************/
#define WS1100_PULSECOUNT 94
#define WS1200_PULSECOUNT 126
boolean alectov3()
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
        checksumcalc = ProtocolAlectoCRC8(data, 6);
      }
      else
      {
        checksum = (bitstream2 >> 24) & 0xff;
        checksumcalc = ProtocolAlectoCRC8(data, 4);
      }

      rc = (bitstream1 >> 20) & 0xff;

      if (checksum != checksumcalc) return false;

      Serial.print(ProgmemString(Protocol_07));
      Serial.print(", ID:");
      Serial.print(rc);

      temperature = ((bitstream1 >> 8) & 0x3ff) - 400;
      Serial.print(", Temp:");
      Serial.print((float)temperature/10);

      if (RawSignal.Number == WS1200_PULSECOUNT)
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
    count_protocol[6]++;
    return true;
}

uint8_t ProtocolAlectoCRC8( uint8_t *addr, uint8_t len)
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
#define THN132N_ID              1230
#define THGN123N_ID              721
#define THGR810_ID             17039
#define THN132N_MIN_PULSECOUNT   196
#define THN132N_MAX_PULSECOUNT   205
#define THGN123N_MIN_PULSECOUNT  228
#define THGN123N_MAX_PULSECOUNT  238
boolean oregonv2()
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
      if ((id == THGN123N_ID) || (id == THGR810_ID))                           // Units with humidity sensor have extra data
        {
          for(byte x=12; x<15;x++) checksumcalc += nibble[x];
          checksum = (nibble[16] << 4) | nibble[15];
        }
      if (checksum != checksumcalc) return false;

      Serial.print(ProgmemString(Protocol_08));
      Serial.print(", ID:");
      Serial.print((nibble[6] << 4) | nibble[5]);

      // if valid sensor type, update user variable and process event
      if ((id == THGN123N_ID) || (id == THGR810_ID) || (id == THN132N_ID))
      {
        datavalue = ((1000 * nibble[10]) + (100 * nibble[9]) + (10 * nibble[8]));
        if ((nibble[11] & 0x8) == 8) datavalue = -1 * datavalue;
        Serial.print(", Temp:");
        Serial.print((float)datavalue/100);
        if ((id == THGN123N_ID) || (id == THGR810_ID))
        {
          datavalue = ((1000 * nibble[13]) + (100 * nibble[12]));
          Serial.print(", Hum:");
          Serial.print((float)datavalue/100);
        }
      }
  Serial.println("");
  count_protocol[7]++;
  return true;
}


/*********************************************************************************************\
 * Flamengo FA20RF protocol
\*********************************************************************************************/
boolean flamengofa20rf()
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

    Serial.print(ProgmemString(Protocol_09));
    Serial.print(", ID:");
    Serial.println(bitstream,HEX);
    count_protocol[8]++;
    return true;
}


/*********************************************************************************************\
 * Home Easy EU protocol
\*********************************************************************************************/
#define HomeEasy_LongLow    0x490    // us
#define HomeEasy_ShortHigh  200      // us
#define HomeEasy_ShortLow   150      // us
boolean homeeasy()
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

      Serial.print(ProgmemString(Protocol_10));
      Serial.print(", Address:");
      Serial.print(address);
      if (state == 0)
        Serial.println(", State:On");
      else
        Serial.println(", State:Off");

      count_protocol[9]++;
      return true;      
}


/*********************************************************************************************\
 * Analyse unknown protocol
\*********************************************************************************************/
void analysepacket(byte mode)
{
      if(RawSignal.Number<8)return;
 
      Serial.print("Unknown");
      Serial.print(", Pulses:");
      Serial.print(RawSignal.Number);
      Serial.print(", ");

      int x;
      unsigned int y,z;
    
      unsigned int MarkShort=50000;
      unsigned int MarkLong=0;
      for(x=5;x<RawSignal.Number;x+=2)
        {
        y=RawSignal.Pulses[x]*RawSignal.Multiply;
        if(y<MarkShort)
          MarkShort=y;
        if(y>MarkLong)
          MarkLong=y;
        }
      z=true;
      while(z)
        {
        z=false;
        for(x=5;x<RawSignal.Number;x+=2)
          {
          y=RawSignal.Pulses[x]*RawSignal.Multiply;
          if(y>MarkShort && y<(MarkShort+MarkShort/2))
            {
            MarkShort=y;
            z=true;
            }
          if(y<MarkLong && y>(MarkLong-MarkLong/2))
            {
            MarkLong=y;
            z=true;
            }
          }
        }
      unsigned int MarkMid=((MarkLong-MarkShort)/2)+MarkShort;
  
      unsigned int SpaceShort=50000;
      unsigned int SpaceLong=0;
      for(x=4;x<RawSignal.Number;x+=2)
        {
        y=RawSignal.Pulses[x]*RawSignal.Multiply;
        if(y<SpaceShort)
          SpaceShort=y;
        if(y>SpaceLong)
          SpaceLong=y;
        }
      z=true;
      while(z)
        {
        z=false;
        for(x=4;x<RawSignal.Number;x+=2)
          {
          y=RawSignal.Pulses[x]*RawSignal.Multiply;
          if(y>SpaceShort && y<(SpaceShort+SpaceShort/2))
            {
            SpaceShort=y;
            z=true;
            }
          if(y<SpaceLong && y>(SpaceLong-SpaceLong/2))
            {
            SpaceLong=y;
            z=true;
            }
          }
        }
      int SpaceMid=((SpaceLong-SpaceShort)/2)+SpaceShort;
    
      // Bepaal soort signaal
      y=0;
      if(MarkLong  > (2*MarkShort  ))y=1; // PWM
      if(SpaceLong > (2*SpaceShort ))y+=2;// PDM

      Serial.print(F( "Bits="));

      if(y==0)Serial.print(F("?"));
      if(y==1)
        {
        for(x=1;x<RawSignal.Number;x+=2)
          {
          y=RawSignal.Pulses[x]*RawSignal.Multiply;
          if(y>MarkMid)
            Serial.write('1');
          else
            Serial.write('0');
          }
        Serial.print(F(", Type=PWM"));
        }
      if(y==2)
        {
        for(x=2;x<RawSignal.Number;x+=2)
          {
          y=RawSignal.Pulses[x]*RawSignal.Multiply;
          if(y>SpaceMid)
            Serial.write('1');
          else
            Serial.write('0');
          }
        Serial.print(F(", Type=PDM"));
        }
      if(y==3)
        {
        for(x=1;x<RawSignal.Number;x+=2)
          {
          y=RawSignal.Pulses[x]*RawSignal.Multiply;
          if(y>MarkMid)
            Serial.write('1');
          else
            Serial.write('0');
          
          y=RawSignal.Pulses[x+1]*RawSignal.Multiply;
          if(y>SpaceMid)
            Serial.write('1');
          else
            Serial.write('0');
          }
        Serial.print(F( ", Type=Manchester"));
        }

      if (mode == 2)
        {
          Serial.print(F(", Pulses(uSec)="));      
          for(x=1;x<RawSignal.Number;x++)
            {
              Serial.print(RawSignal.Pulses[x]*RawSignal.Multiply); 
              Serial.write(',');       
            }
        }
      count_protocol[10]++;
      Serial.println();
}
