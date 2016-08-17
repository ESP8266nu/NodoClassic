// RF Sniffer V1.0.7
// Martinus van den Broek
// 01-01-2014
// 
// Stel de baudrate van de Arduino IDE in op 115200 baud!
//
// Based on work by Paul Tonkes (www.nodo-domotica.nl)
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
// as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
//

/*********************************************************************************************\
 * Do not change anything below this line!
\*********************************************************************************************/

// check if on Mega hardware
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define NODO_MEGA true
#endif

#define HW_BIC_0        0
#define HW_BIC_1        1
#define HW_BIC_2        2
#define HW_BIC_3        3
#define BIC_DEFAULT                  0 // Standaard Nodo zonder specifike hardware aansturing
#define BIC_HWMESH_NES_V1X           1  // Nodo Ethernet Shield V1.x met Aurel tranceiver. Vereist speciale pulse op PIN_BSF_0 voor omschakelen tussen Rx en Tx.
#define PIN_BIC_0                   26 // Board Identification Code: bit-0
#define PIN_BIC_1                   27 // Board Identification Code: bit-1
#define PIN_BIC_2                   28 // Board Identification Code: bit-2
#define PIN_BIC_3                   29 // Board Identification Code: bit-3
#define PIN_BSF_0                   22 // Board Specific Function lijn-0
#define PIN_BSF_1                   23 // Board Specific Function lijn-1
#define PIN_BSF_2                   24 // Board Specific Function lijn-2
#define PIN_BSF_3                   25 // Board Specific Function lijn-3

#if NODO_MEGA
  #define PIN_RF_RX_VCC               16
  #define PIN_RF_RX_DATA              19
#else
  #define PIN_RF_RX_DATA               2 // Connected to RF Receiver data pin. Active HIGH.
  #define PIN_RF_RX_VCC               12 // VCC to receiver (only needed on Nodo hardware!)
#endif

#define RAW_BUFFER_SIZE            256 // Maximum number of RF pulses that can be captured
#define SIGNAL_TIMEOUT_RF            5 // Pulse timings beyond this value indicate end of message

prog_char PROGMEM Text_01[] = "RF Sniffer V1.0.7";
prog_char PROGMEM Text_02[] = "Licensed under GNU General Public License.";
prog_char PROGMEM Text_03[] = "1 - Normal scan mode";
prog_char PROGMEM Text_04[] = "2 - Include raw data for unknown signals";
prog_char PROGMEM Text_05[] = "3 - Show raw data for unknown signals";
prog_char PROGMEM Text_06[] = "4 - Show signal ratio and pulsecount";
prog_char PROGMEM Text_07[] = "5 - Dump ALL RF signals as raw data";
prog_char PROGMEM Text_08[] = "6 - Dump ALL RF signals, pulse info only";
prog_char PROGMEM Text_09[] = "9 - Show statistics";
prog_char PROGMEM Text_10[] = "Signal Ratio:";
prog_char PROGMEM Text_11[] = "%, Pulsecount:";
prog_char PROGMEM Text_12[] = ", Shortest:";
prog_char PROGMEM Text_13[] = ", Longest:";
prog_char PROGMEM Text_14[] = "**********************************************************************";
prog_char PROGMEM Text_15[] = "p - change minimum number of pulses (MIN_RAW_PULSES)";
prog_char PROGMEM Text_16[] = "l - change minimum pulse length (MIN_PULSE_LENGHT)";
prog_char PROGMEM Text_17[] = "r - change sample resolution (RawSignal.Multiply)";

prog_char PROGMEM Protocol_01[] = "Nodo V2";
prog_char PROGMEM Protocol_02[] = "Nodo V1";
prog_char PROGMEM Protocol_03[] = "KAKU V1";
prog_char PROGMEM Protocol_04[] = "KAKU V2";
prog_char PROGMEM Protocol_05[] = "Alecto V1";
prog_char PROGMEM Protocol_06[] = "Alecto V2";
prog_char PROGMEM Protocol_07[] = "Alecto V3";
prog_char PROGMEM Protocol_08[] = "Oregon V2";
prog_char PROGMEM Protocol_09[] = "Flamengo FA20RF";
prog_char PROGMEM Protocol_10[] = "Home Easy 300EU";
prog_char PROGMEM Protocol_11[] = "Unknown";

PROGMEM const char *ProtocolText_tabel[]={Protocol_01,Protocol_02,Protocol_03,Protocol_04,Protocol_05,Protocol_06,Protocol_07,Protocol_08,Protocol_09,Protocol_10,Protocol_11};

uint8_t RFbit,RFport;

struct RawSignalStruct
{
  int  Number;
  byte Multiply;
  byte Pulses[RAW_BUFFER_SIZE+2];
} RawSignal={0,0,0,0,0,0,0,0,0};

byte min_raw_pulses   = 16;    // Minimum number of pulses needed to start decoding.
byte min_pulse_length = 50; // Pulse timings shorter than this value are considered 'noise'
unsigned long timer=millis();
byte mode=0;
int count_protocol[11];
unsigned long HW_Config=0;                                  // Hardware configuratie zoals gedetecteerd door de Nodo.

/*********************************************************************************************\
 * Setup
\*********************************************************************************************/

void setup() {
  Serial.begin(115200);

  #if NODO_MEGA
  // initialiseer BIC-lijnen en lees de BIC uit/
  for(byte x=0;x<=3;x++)
    {
    pinMode(PIN_BIC_0+x,INPUT);
    pinMode(PIN_BIC_0+x,INPUT_PULLUP);
    HW_Config|=digitalRead(PIN_BIC_0+x)<<x;
    }  

    // Hardware specifieke initialisatie.
  switch(HW_Config&0xf)
    {    
    case BIC_DEFAULT:// Standaard Nodo zonder specifike hardware aansturing
      break;                 

    case BIC_HWMESH_NES_V1X: // Nodo Ethernet Shield V1.x met Aurel tranceiver. Vereist speciale pulse op PIN_BSF_0 voor omschakelen tussen Rx en Tx.
      pinMode(PIN_BSF_0,OUTPUT);
      digitalWrite(PIN_BSF_0,HIGH);
      Serial.println("NES Board detected!");
      break;
    }
  #endif

  RawSignal.Multiply=50;
  for (byte x=0; x<10;x++) count_protocol[x]=0;
  RFbit=digitalPinToBitMask(PIN_RF_RX_DATA);
  RFport=digitalPinToPort(PIN_RF_RX_DATA);
  pinMode(PIN_RF_RX_DATA, INPUT);
  pinMode(PIN_RF_RX_VCC,  OUTPUT);
  digitalWrite(PIN_RF_RX_VCC,HIGH);
  digitalWrite(PIN_RF_RX_DATA,INPUT_PULLUP);
  DisplayHelp();
}


/*********************************************************************************************\
 * Main loop
\*********************************************************************************************/

void loop()
{
  if (mode < 10) if((*portInputRegister(RFport)&RFbit)==RFbit) if(FetchSignal(PIN_RF_RX_DATA,HIGH,SIGNAL_TIMEOUT_RF)) AnalyzeRawSignal(mode);
  if (mode == 10) bandwidthUsage();
  if (mode == 11) if((*portInputRegister(RFport)&RFbit)==RFbit) if(FetchSignal(PIN_RF_RX_DATA,HIGH,SIGNAL_TIMEOUT_RF)) DisplayRawSignal();
  if (Serial.available() > 0 ) getcommand();
}


/*********************************************************************************************\
 * Check for simple serial command input
\*********************************************************************************************/

void getcommand(void)
{
  byte command = Serial.read();
  switch(command)
  {
  case '?':
    DisplayHelp();
    break;
  case '1':
    mode = 0;
    Serial.println(ProgmemString(Text_03));
    break;
  case '2':
    mode = 1;
    Serial.println(ProgmemString(Text_04));
    break;
  case '3':
    mode = 2;
    Serial.println(ProgmemString(Text_05));
    break;
  case '5':
    mode = 3;
    Serial.println(ProgmemString(Text_07));
    break;
  case '4':
    mode = 10;
    Serial.println(ProgmemString(Text_06));
    break;
  case '6':
    mode = 11;
    Serial.println(ProgmemString(Text_08));
    break;
  case '9':
    DisplayStats();
    break;
  case 'p':
   change_min_pulses();
   break;
  case 'l':
   change_min_pulse_length();
   break;
  case 'r':
   change_multiply();
   break;
   
  }
}


/*********************************************************************************************\
 * Change minimum number of pulses needed before decoding starts
\*********************************************************************************************/

void change_min_pulses(void)
{
  Serial.println("Minimum Pulses:");
  Serial.println("1 = 8");
  Serial.println("2 = 16");
  Serial.println("3 = 32");
  Serial.println("4 = 64");
  Serial.println("5 = 128");

  delay(10);
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}

  byte command = Serial.read();
  switch(command)
  {
    case '1':
      min_raw_pulses = 8;
      break;
    case '2':
      min_raw_pulses = 16;
      break;
    case '3':
      min_raw_pulses = 32;
      break;
    case '4':
      min_raw_pulses = 64;
      break;
    case '5':
      min_raw_pulses = 128;
      break;
  }
  Serial.print("Min pulses set to: ");
  Serial.println((int)min_raw_pulses);
}


/*********************************************************************************************\
 * Change minimum pulselength (noise filter)
\*********************************************************************************************/

void change_min_pulse_length(void)
{
  Serial.println("Minimum Pulse length:");
  Serial.println("1 = 25");
  Serial.println("2 = 50");
  Serial.println("3 = 75");
  Serial.println("4 = 100");
  Serial.println("5 = 150");

  delay(10);
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}

  byte command = Serial.read();
  switch(command)
  {
    case '1':
      min_pulse_length = 25;
      break;
    case '2':
      min_pulse_length = 50;
      break;
    case '3':
      min_pulse_length = 75;
      break;
    case '4':
      min_pulse_length = 100;
      break;
    case '5':
      min_pulse_length = 150;
      break;
  }
  Serial.print("Min pulse length set to: ");
  Serial.println((int)min_pulse_length);
}

/*********************************************************************************************\
 * Change sample resolution
\*********************************************************************************************/

void change_multiply(void)
{
  Serial.println("Sample resolution:");
  Serial.println("1 = 25");
  Serial.println("2 = 50");

  delay(10);
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {}

  byte command = Serial.read();
  switch(command)
  {
    case '1':
      RawSignal.Multiply=25;
      break;
    case '2':
      RawSignal.Multiply=50;
      break;
  }
  Serial.print("Sample resolution set to: ");
  Serial.println((int)RawSignal.Multiply);
}


/*********************************************************************************************\
 * Display signal ratio
\*********************************************************************************************/

void bandwidthUsage(void)
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
  Serial.print(ProgmemString(Text_10));
  Serial.print((100 * highcounter) / (highcounter+lowcounter));
  Serial.print(ProgmemString(Text_11));
  Serial.print(pulscounter);
  Serial.print(ProgmemString(Text_12));
  Serial.print(shortest);
  Serial.print("uS");
  Serial.print(ProgmemString(Text_13));
  Serial.print(longest);
  Serial.println("uS");
}


/*********************************************************************************************\
 * Analyze signal for known protcols
\*********************************************************************************************/

boolean AnalyzeRawSignal(byte mode)
{
  if(RawSignal.Number==RAW_BUFFER_SIZE)return false;
  Serial.write('+');
  Serial.print(millis()-timer);
  timer = millis();

  Serial.print(" Pulses:");
  Serial.print((int)RawSignal.Number);
  Serial.print(" Protocol: ");

  if (mode==3)
  {
      analysepacket(2);
      return true;
  }
  if(RawSignal_2_Nodo()) return true;
  if(RawSignal_2_NodoNew()) return true;
  if(RawSignal_2_ClassicNodo()) return true;
  if(kaku()) return true;
  if(newkaku()) return true;
  if(alectov1()) return true;
  if(alectov2()) return true;
  if(alectov3()) return true;
  if(oregonv2()) return true;
  if(flamengofa20rf()) return true;
  if(homeeasy()) return true;
  if (mode > 0) analysepacket(mode); 
  else Serial.println("?");
  return false;   
}


/*********************************************************************************************\
 * Fetch signals from RF pin
\*********************************************************************************************/

inline boolean FetchSignal(byte DataPin, boolean StateSignal, int TimeOut)
  {
  uint8_t bit = digitalPinToBitMask(DataPin);
  uint8_t port = digitalPinToPort(DataPin);
  uint8_t stateMask = (StateSignal ? bit : 0);

  // Kijk of er een signaal binnen komt. Zo niet, dan direct deze funktie verlaten.
  if((*portInputRegister(port) & bit) != stateMask)
    return false;

  int RawCodeLength=1;
  unsigned long PulseLength=0;
  unsigned long numloops = 0;
  const unsigned long LoopsPerMilli=400; // Aantal while() *A* loops binnen een milliseconde inc. compensatie overige overhead binnen de while() *B* loop. Uitgeklokt met een analyser 16Mhz ATMega.
  unsigned long maxloops = (unsigned long)TimeOut * LoopsPerMilli;
  boolean toggle=false;

  do{// lees de pulsen in microseconden en plaats deze in de tijdelijke buffer RawSignal
    numloops = 0;
    while(((*portInputRegister(port) & bit) == stateMask) ^ toggle) // while() loop *A*
      if(numloops++ == maxloops)
        break;//timeout opgetreden

    PulseLength=(numloops * 1000) / LoopsPerMilli;// Bevat nu de pulslengte in microseconden

    // bij kleine stoorpulsen die geen betekenis hebben zo snel mogelijk weer terug
    if(PulseLength<min_pulse_length)
      return false;

    toggle=!toggle;    

    // sla op in de tabel RawSignal
    RawSignal.Pulses[RawCodeLength++]=PulseLength/(unsigned long)RawSignal.Multiply;
    }
  while(RawCodeLength<RAW_BUFFER_SIZE && numloops<=maxloops);// loop *B* Zolang nog ruimte in de buffer

  if(RawCodeLength>=min_raw_pulses)
    {
    RawSignal.Number=RawCodeLength-1;
    return true;
    }
  RawSignal.Number=0;
  return false;
  }


/*********************************************************************************************\
 * Display signal details
\*********************************************************************************************/

void DisplayRawSignal()
{
  Serial.print(F("Pulses(uSec)="));      
  for(int x=1;x<RawSignal.Number;x++)
    {
      Serial.print(RawSignal.Pulses[x]*RawSignal.Multiply); 
      Serial.write(',');       
    }
  Serial.println();
}


/*********************************************************************************************\
 * Display help page
\*********************************************************************************************/

void DisplayHelp(void)
{
  Serial.println(ProgmemString(Text_14));
  Serial.println(ProgmemString(Text_01));
  Serial.println(ProgmemString(Text_02));
  Serial.println();
  Serial.println("Commands:");
  Serial.println(ProgmemString(Text_03));
  Serial.println(ProgmemString(Text_04));
  Serial.println(ProgmemString(Text_05));
  Serial.println(ProgmemString(Text_06));
  Serial.println(ProgmemString(Text_07));
  Serial.println(ProgmemString(Text_08));
  Serial.println(ProgmemString(Text_09));
  Serial.println();
  Serial.println("Advanced settings:");
  Serial.println(ProgmemString(Text_15));
  Serial.println(ProgmemString(Text_16));
  Serial.println(ProgmemString(Text_17));

  char* str=(char*)malloc(80);

  Serial.println();
  Serial.println("Supported Protocols:");
  for (byte x=0; x < 10; x++)
    {
      strcpy_P(str,(char*)pgm_read_word(&(ProtocolText_tabel[x])));
      Serial.println(str);
    }
  Serial.println(ProgmemString(Text_14));
  free(str);    
}


/*********************************************************************************************\
 * Display statistics page
\*********************************************************************************************/

void DisplayStats()
{
  char* str=(char*)malloc(80);
  Serial.println();
  Serial.println(ProgmemString(Text_14));
  Serial.println("Statistics:");
  
  for (byte x=0; x < 11; x++)
    {
      strcpy_P(str,(char*)pgm_read_word(&(ProtocolText_tabel[x])));
      Serial.print(str);
      Serial.print(":");
      Serial.println(count_protocol[x]);
    }
  Serial.println(ProgmemString(Text_14));
  free(str);    
}


/*********************************************************************************************\
 * Display string from progmem
\*********************************************************************************************/

char* ProgmemString(prog_char* text)
{
  byte x=0;
  static char buffer[80];

  do
  {
    buffer[x]=pgm_read_byte_near(text+x);
  }
  while(buffer[x++]!=0);
  return buffer;
}


/*********************************************************************************************\
 * Display free RAM
\*********************************************************************************************/

void printFreeRam(void)
{
  Serial.print("Free RAM:");
  Serial.println(freeRam());
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

