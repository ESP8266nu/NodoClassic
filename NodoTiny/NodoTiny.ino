//==============================================================================================================
// Arduino project "Nodo Compatible Tiny" © Copyright 2012,2013,2014 Martinus van den Broek
// Work based on Arduino project "Nodo" © Copyright 2012 Paul Tonkes 
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
// as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.//
//
// You received a copy of the GNU General Public License along with this program in tab '_COPYING'.
//==============================================================================================================
// Purpose                     : Arduino Source code for Nodo Compatible, based on Atmel ATTiny85, running on 8MHz
// Version                     : R76
// Date                        : 20150105
// Compatible with Nodo release: 3.7 (R765)
//
// This code turns an Atmel ATTiny85 chip into a Nodo compatible unit.
// The Source code is based on work by Paul Tonkes, founder of the Nodo project (www.nodo-domotica.nl)
// This code is not of any use without running a full Nodo Domotica Unit within RF range to send/receive commands and data.
//=============================================================================================================
#define FUNCTION  42
#define HOME_NODO  1 

//  0  = Base config                                   Use hardware config 1!
//  1  = Battery Operated Sensor for Dallas DS18B20    Use hardware config 2!
//  2  = Battery Operated Sensor for DHT11             Use hardware config 2!
//  11 = Led Dimmer                                    Use hardware config 5!
//  12 = RF Gateway                                    Use hardware config 1!
//  21 = I2C Analog Pulscounter                        Use hardware config 3!
//  22 = I2C Digital Pulscounter                       Use hardware config 3!
//  31 = Analog to Digital converter                   Use hardware config 4!
//  41 = NRF transceiver, 4 pin                        Use hardware config 6!
//  42 = NRF transceiver, 3 pin                        Use hardware config 7!

//==============================================================================================================
// ATMEL ATTINY85 GENERIC PINOUTS
//                          o-\/-+ 
//                  Reset  1|    |8  VCC 
//              D3/A3 PB3  2|    |7  PB2 D2/A1/I2C-SCL
//              D4/A2 PB4  3|    |6  PB1 D1/PWM1
//                    GND  4|    |5  PB0 D0/PWM0/I2C-SDA
//                          +----+ 
//==============================================================================================================
// HARDWARE CONFIG 1      ATMEL ATTINY85 NORMAL MODE!!
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//             RF RX Data  2|    |7  General purpose in or output (A1 or D2)
//             RF TX Data  3|    |6  Serial RX
//                    GND  4|    |5  Serial TX
//                          +----+ 
//==============================================================================================================
// HARDWARECONFIG 2       ATMEL ATTINY85 BATTERY MODE!!
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//              RF TX VCC  2|    |7  Sensor Datapin (Dallas or DHT11)
//             RF TX Data  3|    |6  Serial RX
//                    GND  4|    |5  Serial TX
//                          +----+ 
//==============================================================================================================
// HARDWARECONFIG 3       ATMEL ATTINY85 I2C Slave device!!
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//      Analog/Digital in  2|    |7  I2C SCL
//              Serial TX  3|    |6  Serial RX
//                    GND  4|    |5  I2C SDA
//                          +----+ 
//==============================================================================================================
// HARDWARE CONFIG 4      ATMEL ATTINY85 ADC MODE!!
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//              Analog in  2|    |7  NC
//            Digital out  3|    |6  Serial RX
//                    GND  4|    |5  Serial Tx
//                          +----+ 
//==============================================================================================================
// HARDWARE CONFIG 5      ATMEL ATTINY85 LED DIMMER MODE!!
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//             RF RX Data  2|    |7  Serial TX
//             RF TX Data  3|    |6  Serial RX
//                    GND  4|    |5  PWM output (LED)
//                          +----+ 
//==============================================================================================================
// HARDWARE CONFIG 6      ATMEL ATTINY85 NRF
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//                   MOSI  2|    |7  SCK
//                    CSN  3|    |6  Serial RX / MISO (use 1k resistor to FTDI TX pin!)
//                    GND  4|    |5  Serial TX
//                          +----+ 
//==============================================================================================================
// HARDWARE CONFIG 7      ATMEL ATTINY85 NRF 3 pin and custom CSN logic, needs extra hardware
//                          o-\/-+ 
//                  reset  1|    |8  VCC 
//                   MOSI  2|    |7  MISO
//                    SCK  3|    |6  Serial RX
//                    GND  4|    |5  Serial TX
//                          +----+ 
//
// Connection to NRF for 3-pin mode:
//
// Tiny                   NRF
// ====                   ====
// MISO ------------------MISO
// MOSI ------------------MOSI
//        ----------------SCK
//        |       ___
//  SCK -----|>--|___|--- VCC
//              |
//              |---------CSN
//              |
//             === Cap 100 nF
//              |
//             GND
//
//       Diode like 1N4148,  Resistor (1k)
//
// How does it work:
//   SCK is held low for some uSec. Through diode it will decharge the cap to logic '0' level.
//   CSN is now active
//   MCU will send data
//   SCK is held high for some uSec.
//   cap will charge through resistor connected to VCC to logic '1' level.
// This way the CSN pulse high is send as needed for the NRF to start processing data
//==============================================================================================================
// Supported Nodo events and commands:
// Events:
//   Boot
//   NewKaku
//   Timer
//   UserEvent
// Commands:
//   Delay <seconds>
//   DHTRead 1,<var>                 // port is fixed (D2)
//   EventListErase
//   Reboot
//   UserEventSend <par1>,<par2>     // 0-255
//   RGBLedSend                      // one channel only, using value send by newkaku !
//   Status Freemem
//   Status Variableset,1            // Only one variable !
//   Status WiredAnalog
//   Tempread 1,<var>                // port is fixed (D2)
//   TimerSet 1,<value>              // Only 1 timer !
//   UnitSet <unit>                  // 1-31
//   Variableset,1                   // Only one variable !
//   WiredOut 1,<on/off>             // Only 1 output
//   WiredSmittTrigger 1,<value>     // Only 1 input, value 0-1023
//   WiredThreshold 1,<value>        // Only 1 input, value 0-1023
//==============================================================================================================
#define VERSION                       76
#define NODO_VERSION_MAJOR             3  // Ophogen bij DataBlock en NodoEventStruct wijzigingen.
#define NODO_VERSION_MINOR            10  // Ophogen bij gewijzigde settings struct of nummering events/commando's. 
#include <EEPROM.h>
#define NODO_TINY

#if FUNCTION == 0             // Base config
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  #define RF_RECEIVE
  #define RF_SEND
#endif

#if FUNCTION == 1              // Battery operated Dallas Sensor
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  #define BATTERYMODE          // IF ENABLED, USE HARDWARE CONFIG 2 !!! AND DISABLE RF_RECEIVE IN THIS MODE
  #define RF_SEND              // Can be disabled in case of a simple (kaku) receiver without transmitter
  #define DALLAS               // Also enable RF_SEND! and connect a Dallas DS18b20 to pin 2
#endif

#if FUNCTION == 2              // Battery operated DHT11 Sensor
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  #define BATTERYMODE          // IF ENABLED, USE HARDWARE CONFIG 2 !!! AND DISABLE RF_RECEIVE IN THIS MODE
  #define RF_SEND              // Can be disabled in case of a simple (kaku) receiver without transmitter
  #define DHT11                // Also enable RF SEND! and connect a DHT11 sensor to pin 2
#endif

#if FUNCTION == 11             // Led Dimmer
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 2
  #define RF_RECEIVE
  #define NEWKAKU              // Also enable RF_RECEIVE
  #define PWM_OUTPUT           // Also enable RF_RECEIVE and connect something to pin 0
#endif

#if FUNCTION == 12             // RF Gateway
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  #define RF_RECEIVE
  #define RF_SEND
  #define RFGATEWAY
#endif

#if FUNCTION == 21             // I2C Analog Pulse Counter
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 4
  #define ANALOG
  #define ANALOGPIN 3
  #define I2C
#endif

#if FUNCTION == 22             // I2C Digital Pulse Counter
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 4
  #define DIGITAL
  #define DIGITALPIN 3
  #define I2C
#endif

#if FUNCTION == 31             // Analog to Digital converter
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  #define ADCMODE
  #define ANALOGPIN 3
  #define ADC_PIN 4
#endif

#if FUNCTION == 41 // NRF met 4 pinnen
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  //#define DALLAS
  //#define DHT11
  #define NRF24L01
  //#define NRF_DEFAULT_DESTINATION 16
  #define PLUGIN_033
  #define PLUGIN_033_CORE
  #define NODO_BETA_PLUGIN_SENDTO
  #define NRF_ADDRESS      12,13,14,15
  #define NRF_CHANNEL      44
  #define NRF_CSN_PIN      4
  #define NRF_MOSI_PIN     3
  #define NRF_MISO_PIN     1
  #define NRF_SCK_PIN      2
  #define PLUGIN_COMMAND               4 
  #define PLUGIN_INIT                  5
  #define PLUGIN_EVENT_OUT             8
  #define PLUGIN_SCAN_EVENT          255
  #define CMD_SENDTO                  94
  #define SYSTEM_COMMAND_QUEUE_SENDTO  4
#endif

#if FUNCTION == 42 // NRF  met 3 pinnen
  #define SERIAL_RX_PIN 1
  #define SERIAL_TX_PIN 0
  //#define DALLAS
  //#define DHT11
  #define NRF24L01
  //#define NRF_DEFAULT_DESTINATION 5
  #define PLUGIN_033
  #define PLUGIN_033_CORE
  #define NRF_CSN_TRICK
  #define NODO_BETA_PLUGIN_SENDTO
  #define NRF_ADDRESS      12,13,14,15
  #define NRF_CHANNEL      44
  #define NRF_CSN_PIN      4 // 'fake setting for plugin logic
  #define NRF_MOSI_PIN     3
  #define NRF_MISO_PIN     2
  #define NRF_SCK_PIN      4
  #define PLUGIN_COMMAND               4 
  #define PLUGIN_INIT                  5
  #define PLUGIN_EVENT_OUT             8
  #define PLUGIN_SCAN_EVENT          255
  #define CMD_SENDTO                  94
  #define SYSTEM_COMMAND_QUEUE_SENDTO  4
#endif

void NodoOnline(byte y,byte x){}

#define MINI_MMI
//#define UP_DEBUG

#define PIN_LED                          2 
#define DHT11_PIN                        2
#define DALLAS_PIN                       2
#define PIN_WIRED_OUT_1                  2
#define EEPROM_CAL_BYTE_ADDRESS        511

#define RAW_BUFFER_SIZE                196

#define RF_REPEATS                       1 // number of send repeats

#define CMD_BOOT_EVENT                   1
#define CMD_USEREVENT                    3
#define CMD_VARIABLE_EVENT               4
#define CMD_VARIABLE_SET                 5

#define VALUE_ALL                       18
#define CMD_DELAY                       38
#define VALUE_SOURCE_EVENTLIST          44
#define CMD_EVENTLIST_ERASE             46
#define VALUE_FREEMEM                   58
#define CMD_GATEWAY                     59
#define VALUE_DIRECTION_INPUT           67
#define VALUE_OFF                       74 
#define VALUE_ON                        75
#define VALUE_DIRECTION_OUTPUT          77
#define CMD_REBOOT                      89
#define CMD_RESET                       91
#define VALUE_SOURCE_RF                 92
#define CMD_SEND_USEREVENT              95
#define VALUE_SOURCE_SERIAL             96
#define CMD_STATUS                      98
#define VALUE_SOURCE_THISUNIT          101
#define CMD_TIMER_EVENT                105
#define CMD_TIMER_SET                  107
#define CMD_UNIT_SET                   110
#define CMD_PULSE_COUNT                113
#define CMD_WIRED_ANALOG               123
#define CMD_WIRED_OUT                  125
#define CMD_WIRED_SMITTTRIGGER         127
#define CMD_WIRED_THRESHOLD            128
#define CMD_SLEEP                      143  
#define CMD_USERPLUGIN                 255

#define NODO_TYPE_EVENT                  1
#define NODO_TYPE_COMMAND                2
#define NODO_TYPE_SYSTEM                 3
#define NODO_TYPE_PLUGIN_EVENT           4
#define NODO_TYPE_PLUGIN_COMMAND         5

#define CMD_PLUGIN_KAKU_NEW              2
#define CMD_PLUGIN_DALLAS                5
#define CMD_PLUGIN_DHT                   6
#define CMD_PLUGIN_LED                  23

#ifdef BATTERYMODE
  #define PIN_RF_TX_VCC                  3  // Battery Mode, VCC to RF Transmitter, fysieke pin 2 on ATTiny85 
#else
  #define PIN_RF_RX_DATA                 3  // NON Battery Mode, RF receiver datapin, LOW if no signal.
#endif

#define PIN_RF_TX_DATA                   4  // data to RF Transmitter, fysieke pin 3 on ATTiny85

#define SIGNAL_TIMEOUT_RF             5000 // na deze tijd in uSec. wordt één RF signaal als beëindigd beschouwd.
#define SHARP_TIME                     750 // tijd in milliseconden dat de nodo gefocust moet blijven luisteren naar één dezelfde poort na binnenkomst van een signaal
#define SIGNAL_REPEAT_TIME            1000 // Tijd waarbinnen hetzelfde event niet nogmaals via RF of IR mag binnenkomen. Onderdrukt ongewenste herhalingen van signaal
#define MIN_PULSE_LENGTH               100 // pulsen korter dan deze tijd uSec. worden als stoorpulsen beschouwd.
#define MIN_RAW_PULSES                  64 // =16 bits. Minimaal aantal ontvangen bits*2 alvorens cpu tijd wordt besteed aan decodering, etc. Zet zo hoog mogelijk om CPU-tijd te sparen en minder 'onzin' te ontvangen.

#define BAUD_RATE                     9600   // Serial communication baud rate

#define NODO_PULSE_0                   500   // PWM: Tijdsduur van de puls bij verzenden van een '0' in uSec.
#define NODO_PULSE_MID                1000   // PWM: Pulsen langer zijn '1'
#define NODO_PULSE_1                  1500   // PWM: Tijdsduur van de puls bij verzenden van een '1' in uSec. (3x NODO_PULSE_0)
#define NODO_SPACE                     500   // PWM: Tijdsduur van de space tussen de bitspuls bij verzenden van een '1' in uSec.   

#define EVENTLIST_MAX                   25 // aantal events dat de lijst bevat in het EEPROM geheugen. Iedere regel in de eventlist heeft 14 bytes nodig. eerste adres is 0
#define TIMER_MAX                        1 // aantal beschikbare timers voor de user, gerekend vanaf 1
#define USER_VARIABLES_MAX               1 // aantal beschikbare gebruikersvariabelen voor de user.

uint8_t RFbit,RFport;                          // t.b.v. verwerking IR/FR signalen.
float UserVar[USER_VARIABLES_MAX];             // Gebruikers variabelen
unsigned long UserTimer[TIMER_MAX];            // Timers voor de gebruiker.

struct SettingsStruct
  {
  byte    Unit;
  }Settings;

#if defined(RF_SEND) || defined(RF_RECEIVE)
struct RawsignalStruct
  {
  byte Pulses[RAW_BUFFER_SIZE+2];               // Tabel met de gemeten pulsen in microseconden. eerste waarde [0] wordt NIET gebruikt. (legacy redenen).
  byte Source;                                  // Bron waar het signaal op is binnengekomen.
  int Number;                                   // aantal bits, maal twee omdat iedere bit een mark en een space heeft.
  byte Repeats;
  unsigned long Timer;                          // Tijdstip millis() waarop event is binnengekomen.
  }RawSignal;
#endif

struct NodoEventStruct
  {
  // Event deel
  byte Type;  
  byte Command;
  byte Par1;
  unsigned long Par2;

  // Transmissie deel
  byte SourceUnit;
  byte DestinationUnit;
  byte Flags;
  byte Port;
  byte Direction;
  byte Version;
  byte Checksum;
  };

// Definieer een datablock die gebruikt wordt voor de gegevens die via de ether verzonden moeten worden.
// Zo kunnen exact die gevens worden verzonden die nodig zijn en niets teveel.  
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

void(*Reset)(void)=0;

#ifdef ANALOG
  unsigned int WiredInputStatus=0;
  unsigned int WiredInputThreshold=512;
  unsigned int WiredInputSmittTrigger=10;
  unsigned int AnalogValue=0;
  unsigned long pulscounter=0;
  struct NodoEventStruct TempEvent;
#endif

#ifdef DIGITAL
  unsigned long pulscounter=0;
  byte pulsstate=0;
  struct NodoEventStruct TempEvent;
#endif

#ifdef ADCMODE
  unsigned int WiredInputStatus=0;
  unsigned int WiredInputThreshold=512;
  unsigned int WiredInputSmittTrigger=10;
  unsigned int AnalogValue=0;
#endif

#ifdef RFGATEWAY
  byte RepeatCounter=0;
  byte SendOnCount=1;
#endif

char OutputLine[12];  // made global for both int2str and int2strhex, to save RAM.
boolean SoftwareSerial = true;

#ifdef NRF24L01
  byte nrfloopcounter=0;
#endif

/*********************************************************************************************\
 * Setup stuff
\*********************************************************************************************/
void setup()
{
  if ((EEPROM.read(EEPROM_CAL_BYTE_ADDRESS) < 255)) OSCCAL = EEPROM.read(EEPROM_CAL_BYTE_ADDRESS);

  SoftwareSerial_init(SERIAL_RX_PIN,SERIAL_TX_PIN);         // RX, TX

  #ifdef RF_RECEIVE
    pinMode(PIN_RF_RX_DATA, INPUT); 
    digitalWrite(PIN_RF_RX_DATA,INPUT_PULLUP);
    RFbit=digitalPinToBitMask(PIN_RF_RX_DATA);
    RFport=digitalPinToPort(PIN_RF_RX_DATA);  
  #endif
  pinMode(PIN_LED, OUTPUT);
  #ifdef RF_SEND
    pinMode(PIN_RF_TX_DATA, OUTPUT);
  #endif
  digitalWrite(PIN_LED,LOW);

  struct NodoEventStruct TempEvent;
  
  LoadSettings();      // laad alle settings zoals deze in de EEPROM zijn opgeslagen
  if (Settings.Unit == 255)
  {
    Settings.Unit=15;
    Save_Settings();
    TempEvent.Command=CMD_EVENTLIST_ERASE;
    TempEvent.Par1=Settings.Unit;
    TempEvent.Par2=0;
    ExecuteCommand(&TempEvent,0);
    Reset();
  }
 
  SoftwareSerial_write('>');
  SoftwareSerial_print(int2str(Settings.Unit));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2str(HOME_NODO));
  SoftwareSerial_write(',');
  SoftwareSerial_println(int2str(VERSION));

  #ifdef I2C
    TinyWireS_begin(Settings.Unit);
  #endif

  #ifdef BATTERYMODE
    Watchdog_setup(9);
    pinMode(PIN_RF_TX_VCC,OUTPUT);                        // Set TX power pin as output
    digitalWrite(PIN_RF_TX_VCC,HIGH);                     // Turn on TX device
  #endif

  #ifdef DIGITAL
    pinMode(DIGITALPIN, INPUT);
    digitalWrite(DIGITALPIN, HIGH);
    // set pin change interrupt to D3 for pulsecounter
    if (digitalPinToPCICR(DIGITALPIN))
      {
        *digitalPinToPCICR(DIGITALPIN) |= _BV(digitalPinToPCICRbit(DIGITALPIN));
        *digitalPinToPCMSK(DIGITALPIN) |= _BV(digitalPinToPCMSKbit(DIGITALPIN));
      }
  #endif

  #ifdef ANALOG
    pinMode(ANALOGPIN, INPUT);
    digitalWrite(ANALOGPIN,HIGH);
  #endif
  
  #ifdef ADCMODE
    pinMode(ADC_PIN, OUTPUT);
    digitalWrite(ADC_PIN, HIGH);
    pinMode(ANALOGPIN, INPUT);
    digitalWrite(ANALOGPIN,HIGH);
  #endif
  
  #ifdef NRF24L01
     SoftwareSerial_irqoff();
     Plugin_033(PLUGIN_INIT, 0, 0);
     SoftwareSerial_irqon();
  #endif

  TempEvent.Type            = NODO_TYPE_EVENT;
  TempEvent.Direction       = VALUE_DIRECTION_INPUT;
  TempEvent.SourceUnit      = Settings.Unit;  
  TempEvent.DestinationUnit = 0;  
  TempEvent.Port            = VALUE_ALL; 
  TempEvent.Command         = CMD_BOOT_EVENT;
  TempEvent.Par1            = Settings.Unit;
  TempEvent.Par2            = 0;
  TempEvent.Flags           = 0;
  TempEvent.Version         = NODO_VERSION_MINOR;
  TempEvent.Checksum        = 0;
  ProcessEvent(&TempEvent);

  #ifdef NRF24L01
    SoftwareSerial_irqoff();
    Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
    SoftwareSerial_irqon();
  #endif

}

/*********************************************************************************************\
 * Main loop
\*********************************************************************************************/
void loop()
{
  struct NodoEventStruct ReceivedEvent;

  #ifdef NRF24L01
    nrfloopcounter++;

    if (nrfloopcounter == 255)
      {
        nrfloopcounter=0;
        SoftwareSerial_irqoff();
        if (Plugin_033(255, &ReceivedEvent, 0))
          {
            SoftwareSerial_irqon();
            ProcessEvent(&ReceivedEvent);
          }
        SoftwareSerial_irqon();
      }
  #endif
  
  #ifdef MINI_MMI
    UserPlugin_MiniMMI();
  #endif
  
  #ifdef RF_RECEIVE
    if(ScanEvent(&ReceivedEvent))
    {
      ProcessEvent(&ReceivedEvent);
    }   
  #endif
  
  #ifdef ANALOG
    AnalogValue=analogRead(ANALOGPIN);
    if(!WiredInputStatus && AnalogValue > (WiredInputThreshold + WiredInputSmittTrigger))
      {
        WiredInputStatus = true;
      }
      
    if(WiredInputStatus && AnalogValue < (WiredInputThreshold - WiredInputSmittTrigger))
      {
        WiredInputStatus = false;
        pulscounter++;
      }  
  #endif

  #ifdef ADCMODE
    AnalogValue=analogRead(ANALOGPIN);
    if(!WiredInputStatus && AnalogValue > (WiredInputThreshold + WiredInputSmittTrigger))
      {
        WiredInputStatus = true;
        digitalWrite(ADC_PIN,HIGH);
      }
      
    if(WiredInputStatus && AnalogValue < (WiredInputThreshold - WiredInputSmittTrigger))
      {
        WiredInputStatus = false;
        digitalWrite(ADC_PIN,LOW);
      }  
  #endif
  
  #ifdef I2C
  if (TinyWireS_available())
    {
      delay(5); // wait for all bytes to receive (13 bytes ~ 1-2 mSec)
      if(ReceiveI2C())
        {
          if (TempEvent.Command == 0)
            {
              byte cmd=TempEvent.Par1-128;
              TempEvent.SourceUnit=Settings.Unit;
              TempEvent.Direction=VALUE_DIRECTION_OUTPUT;
              switch (cmd)
                {
                  case 1:
                  {
                    TempEvent.Command=CMD_PULSE_COUNT;
                    TempEvent.Par1=0;
                    TempEvent.Par2=pulscounter;
                    break;
                  }
                }
              ReplyI2C(&TempEvent);
            }
            PrintEvent(&TempEvent);
         }
       delay(100);
       TinyWireS_flush();
    }
    #endif

    // TIMER
  for(byte x=0;x<TIMER_MAX;x++)
    {
    if(UserTimer[x]!=0L)// if timer active
      {
      if(UserTimer[x]<millis())
        {
          UserTimer[x]=0L;
          struct NodoEventStruct TempEvent;
          TempEvent.Type       = NODO_TYPE_EVENT;
          TempEvent.Direction  = VALUE_DIRECTION_INPUT;
          TempEvent.SourceUnit = Settings.Unit;  
          TempEvent.Port       = VALUE_SOURCE_THISUNIT;
          TempEvent.Command    = CMD_TIMER_EVENT;
          TempEvent.Par1       = x+1;
          TempEvent.Par2       = 0;
          ProcessEvent(&TempEvent);
        }
      }
    }
}

/*********************************************************************************************\
 * Deze functie checked of de code die ontvangen is een uitvoerbare opdracht is/
 * Als het een correct commando is wordt deze uitgevoerd en 
 * true teruggegeven. Zo niet dan wordt er een 'false' retour gegeven.
\*********************************************************************************************/
boolean ExecuteCommand(struct NodoEventStruct *EventToExecute, unsigned long PreviousInEvent)
  {

  byte Type           = EventToExecute->Type;
  byte Command        = EventToExecute->Command;
  byte Par1           = EventToExecute->Par1;
  unsigned long Par2  = EventToExecute->Par2;

  struct NodoEventStruct TempEvent=*EventToExecute;
  TempEvent.SourceUnit=Settings.Unit;
  TempEvent.Direction=VALUE_DIRECTION_OUTPUT;
  TempEvent.Port=VALUE_SOURCE_RF;
  TempEvent.Flags=0;

  if ((Type == NODO_TYPE_EVENT) || (Type == NODO_TYPE_COMMAND) || (Type == NODO_TYPE_SYSTEM))
  {
    switch(Command)
    {
    case CMD_UNIT_SET:
      Settings.Unit=Par1;
      Save_Settings();
      Reset();
      break;

    case CMD_DELAY:
      delay(1000*Par1);
      break;

    case CMD_WIRED_OUT:
      digitalWrite(PIN_WIRED_OUT_1,(Par2==VALUE_ON));
      break;

    #if defined(ANALOG) || defined(ADCMODE)
    case CMD_WIRED_THRESHOLD:
      WiredInputThreshold=Par2;
      SoftwareSerial_println(int2str(WiredInputThreshold));
      break;

    case CMD_WIRED_SMITTTRIGGER:
      WiredInputSmittTrigger=Par2;
      SoftwareSerial_println(int2str(WiredInputSmittTrigger));
      break;
    #endif  
    
    case CMD_SEND_USEREVENT:
      #ifdef RF_SEND
        Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_USEREVENT,Par1,Par2);
        PrintEvent(&TempEvent);
        RawSendRF();
      #endif
      #ifdef NRF24L01
        TempEvent.Type=NODO_TYPE_EVENT;
        TempEvent.Command=CMD_USEREVENT;
        SoftwareSerial_irqoff();
        Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
        SoftwareSerial_irqon();
        PrintEvent(&TempEvent);
      #endif
      break;

    case CMD_TIMER_SET:
      if (Par2 == 0)
        UserTimer[Par1-1]=0L;
      else
        UserTimer[Par1-1]=millis()+(unsigned long)Par2*1000L;
      break;

    case CMD_VARIABLE_SET:
      UserVar[Par1-1]=ul2float(Par2);
      break;
      
    case CMD_STATUS:
      if (Par1 == CMD_VARIABLE_SET)
        {
          #ifdef RF_SEND
            Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_VARIABLE_EVENT, Par2, float2ul(UserVar[Par2-1]));
            PrintEvent(&TempEvent);
            RawSendRF();
          #endif
          #ifdef NRF24L01
            TempEvent.Type=NODO_TYPE_EVENT;
            TempEvent.Command=CMD_VARIABLE_EVENT;
            TempEvent.Par1=Par2;
            TempEvent.Par2=float2ul(UserVar[Par2-1]);
            SoftwareSerial_irqoff();
            Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
            SoftwareSerial_irqon();
            PrintEvent(&TempEvent);
          #endif
        }
      if (Par1 == CMD_WIRED_ANALOG)
        {
          #ifdef RF_SEND
            Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_WIRED_ANALOG, Par2, analogRead(1));
            PrintEvent(&TempEvent);
            RawSendRF();
          #endif
          #ifdef ADCMODE
            TempEvent.Command = CMD_WIRED_ANALOG;
            TempEvent.Par1 = 1;
            TempEvent.Par2 = analogRead(ANALOGPIN);
            PrintEvent(&TempEvent);
          #endif
        }

      #ifdef UP_DEBUG
      if (Par1 == VALUE_FREEMEM)
        {
          printFreeRam();
        }
      #endif
      break;
   
    #ifdef BATTERYMODE
    case CMD_SLEEP:
      digitalWrite(PIN_RF_TX_VCC,LOW);                      // Turn off TX device
      pinMode(PIN_RF_TX_VCC,INPUT);                         // Set TX pin as input (float to preserve power)
      Watchdog_sleep(Par1);
      pinMode(PIN_RF_TX_VCC,OUTPUT);                        // Set TX power pin as output
      digitalWrite(PIN_RF_TX_VCC,HIGH);                     // Turn on TX device
      delay(10);                                // Wait for TX device to settle
      break;
    #endif

    case CMD_GATEWAY:
      #ifdef RFGATEWAY
        if (Par1 == 10) SendOnCount=Par2;
      #endif
      break;
      
    case CMD_REBOOT:
      Reset();
      break;        

    case CMD_EVENTLIST_ERASE:
      int baseaddress=sizeof(struct SettingsStruct);
      for (int x=0; x < (EVENTLIST_MAX*14); x++)
        {
          EEPROM.write(baseaddress+x, 0);
        }
      break;        

    } // end case
  } // if event/command

  if ((Type == NODO_TYPE_PLUGIN_EVENT) || (Type == NODO_TYPE_PLUGIN_COMMAND))
  {
  switch(Command)
    {
      #ifdef DALLAS
      case CMD_PLUGIN_DALLAS:
        #ifdef RF_SEND
          Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_VARIABLE_EVENT, Par2,float2ul(ReadDallas()));
          PrintEvent(&TempEvent);
          RawSendRF();
        #endif
        #ifdef NRF24L01
          TempEvent.Type=NODO_TYPE_EVENT;
          TempEvent.Command=CMD_VARIABLE_EVENT;
          TempEvent.Par2 = float2ul(ReadDallas());
          Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
          PrintEvent(&TempEvent);
        #endif
         break;
      #endif

      #ifdef DHT11
      case CMD_PLUGIN_DHT:
        byte DHTtemperature = 0;
        byte DHThumidity = 0;
        byte DHTresult = 0;
        DHTresult=dhtread(DHTtemperature,DHThumidity);
        #ifdef RF_SEND
          Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_VARIABLE_EVENT, Par2, float2ul(DHTtemperature *100));
          PrintEvent(&TempEvent);
          RawSendRF();
          delay(1000);
          Nodo_2_RawSignal(NODO_TYPE_EVENT, CMD_VARIABLE_EVENT, Par2, float2ul(DHThumidity *100));
          PrintEvent(&TempEvent);
          RawSendRF();
          delay(1000);
        #endif
        #ifdef NRF24L01
          TempEvent.Type=NODO_TYPE_EVENT;
          TempEvent.Command=CMD_VARIABLE_EVENT;
          TempEvent.Par2 = float2ul(DHTtemperature *100);
          Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
          delay(1000);
          TempEvent.Par2 = float2ul(DHThumidity *100);
          Plugin_033(PLUGIN_EVENT_OUT, &TempEvent, 0);
          PrintEvent(&TempEvent);
        #endif
        break;
      #endif

    #ifdef PWM_OUTPUT
    case CMD_PLUGIN_LED:
      Par2 = PreviousInEvent & 0xff;
      if (((PreviousInEvent >> 8) & 0xff) == CMD_PLUGIN_AKU_NEW)
      {
        if (Par2 == VALUE_OFF) Par2=0;
        if (Par2 == VALUE_ON) Par2=15;
        Par2 = Par2 << 4;
      }
      analogWrite(0,(byte)Par2);
      break;
    #endif

    } // end case
  } // if plugin event/command
  
  }

/**********************************************************************************************\
 * Voert alle relevante acties in de eventlist uit die horen bij het binnengekomen event
 * Doorlopen van een volledig gevulde eventlist duurt ongeveer 15ms inclusief printen naar serial
 * maar exclusief verwerking n.a.v. een 'hit' in de eventlist
\*********************************************************************************************/

void ProcessEvent(struct NodoEventStruct *EventToExecute)
{
  unsigned long Event_1, Event_2, Mask, IncommingEvent;
  unsigned long Par2_1, Par2_2;
  int x;
  byte Type = EventToExecute->Type;
  byte Command = EventToExecute->Command;
  byte Par1    = EventToExecute->Par1;
  unsigned long Par2    = EventToExecute->Par2;
  boolean match;

  PrintEvent(EventToExecute);
  
  IncommingEvent=((unsigned long)Type << 16 | Command << 8) | Par1;
  
  for(x=1; x<=EVENTLIST_MAX; x++)
  {
    match = false;
    Eventlist_Read(x,&Event_1,&Event_2, &Par2_1, &Par2_2);

    Mask = 0xffffff;
    if ((Type == NODO_TYPE_PLUGIN_EVENT) && (Command == CMD_PLUGIN_KAKU_NEW))
      {
        if (Par2 == Par2_1) match = true; // Kaku match on 26 bit address
      }
      else 
      {
        if ((Event_1 & 0xff) == 0) Mask = 0xffff00;
        if ((IncommingEvent & Mask) == (Event_1 & Mask))
          {
          if ((Par2 == Par2_1) || (Par2_1 == 0)) match = true;
          }
      }

    if (match)
    {
        struct NodoEventStruct TempEvent;
        TempEvent.SourceUnit = Settings.Unit;  
        TempEvent.Type       = (Event_2 >> 16) & 0xff;
        TempEvent.Command    = (Event_2 >> 8) & 0xff;
        TempEvent.Par1       = Event_2 & 0xff;
        TempEvent.Par2       = Par2_2;
        TempEvent.Direction  = VALUE_DIRECTION_OUTPUT;
        TempEvent.Port       = VALUE_SOURCE_EVENTLIST;
        TempEvent.DestinationUnit = 0;  
        TempEvent.Flags           = 0;
        TempEvent.Version         = NODO_VERSION_MINOR;
        TempEvent.Checksum        = 0;
        PrintEvent(&TempEvent);
        ExecuteCommand(&TempEvent, IncommingEvent);
    }
  }
}

/**********************************************************************************************\
 * Revision 01, 09-12-2009, P.K.Tonkes@gmail.com
\*********************************************************************************************/
boolean Eventlist_Read(int address, unsigned long *Event, unsigned long *Action, unsigned long *Par2_1, unsigned long *Par2_2)// LET OP: eerste adres=1
{
  if(address>EVENTLIST_MAX)return(false);
  address--;// echte adressering begint vanaf nul. voor de user vanaf 1.
  address=address*14+sizeof(struct SettingsStruct);     // Eerste deel van het EEPROM geheugen is voor de settings. Reserveer deze bytes. Deze niet te gebruiken voor de Eventlist!

  *Event  =((unsigned long)(EEPROM.read(address++))) <<  16;
  *Event |=((unsigned long)(EEPROM.read(address++))) <<  8;
  *Event |=((unsigned long)(EEPROM.read(address++))) <<  0;
  *Par2_1 =((unsigned long)(EEPROM.read(address++))) << 0;
  *Par2_1|=((unsigned long)(EEPROM.read(address++))) << 8;
  *Par2_1|=((unsigned long)(EEPROM.read(address++))) << 16;
  *Par2_1|=((unsigned long)(EEPROM.read(address++))) << 24;
  
  *Action =((unsigned long)(EEPROM.read(address++))) <<  16;
  *Action|=((unsigned long)(EEPROM.read(address++))) <<   8;
  *Action|=((unsigned long)(EEPROM.read(address++))) <<   0;
  *Par2_2 =((unsigned long)(EEPROM.read(address++))) <<  0;
  *Par2_2|=((unsigned long)(EEPROM.read(address++))) <<  8;
  *Par2_2|=((unsigned long)(EEPROM.read(address++))) << 16;
  *Par2_2|=((unsigned long)(EEPROM.read(address++))) << 24;

  if(*Event==0L)
    return(false);
  else
    return(true);
}

/*********************************************************************************************\
 * Print een event: debug mode Nodo-Small
 \*********************************************************************************************/
void PrintEvent(struct NodoEventStruct *Event)
  {
  SoftwareSerial_print(int2str(Event->Direction));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2str(Event->Port));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2str(Event->Command));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2str(Event->Par1));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2strhex(Event->Par2));
  SoftwareSerial_write(',');
  SoftwareSerial_print(int2str(Event->SourceUnit & 0x1f));
  SoftwareSerial_write(',');
  SoftwareSerial_println(int2str(Event->Type));
  } 

#ifdef RF_RECEIVE
/*********************************************************************************************\
 * Scan for events on RF pin
 \*********************************************************************************************/
boolean ScanEvent(struct NodoEventStruct *Event)
  {
    
  byte Fetched=0;
  static unsigned long PreviousTime=0L;
  static unsigned long PreviousHash=0L;

    // RF: *************** kijk of er data start op RF en genereer een event als er een code ontvangen is **********************
    if((*portInputRegister(RFport)&RFbit)==RFbit)// Kijk if er iets op de RF poort binnenkomt. (Pin=HOOG als signaal in de ether). 
      if(FetchSignal(PIN_RF_RX_DATA,HIGH,SIGNAL_TIMEOUT_RF))// Als het een duidelijk RF signaal was
        Fetched=VALUE_SOURCE_RF;

    if(Fetched)
      {
      if(AnalyzeRawSignal(Event))// als AnalyzeRawSignal een event heeft opgeleverd dan is het struct Event gevuld.
        {
          if(RawSignal.Repeats)
            {
            unsigned long Hash=(unsigned long)(Event->Command<<24) || (unsigned long)(Event->Par1<<16) || (unsigned long)(Event->Par2&0xffff);
        
            if(Hash!=PreviousHash)
              {
              PreviousHash=Hash;
              return false;
              }

            if(PreviousTime>(millis()-SIGNAL_REPEAT_TIME))
              {
              return false;
              }
            }

            PreviousTime=millis();
            Event->Port=Fetched;
            Event->Direction=VALUE_DIRECTION_INPUT;

            if(Event->DestinationUnit==0 || Event->DestinationUnit==Settings.Unit)
              {
                return true;
              }
        }  // analyse rawsignal
      Fetched=0;
      }  // fetched

  return false;
  }

 /**********************************************************************************************\
 * Haal de pulsen en plaats in buffer. Op het moment hier aangekomen is de startbit actief.
 * bij de TSOP1738 is in rust is de uitgang hoog. StateSignal moet LOW zijn
 * bij de 433RX is in rust is de uitgang laag. StateSignal moet HIGH zijn
 \*********************************************************************************************/
boolean FetchSignal(byte DataPin, boolean StateSignal, int TimeOut)
  {
  int RawCodeLength=1;
  unsigned long PulseLength=0;

  do{// lees de pulsen in microseconden en plaats deze in een tijdelijke buffer
    PulseLength=WaitForChangeState(DataPin, StateSignal, TimeOut);

    if(PulseLength<MIN_PULSE_LENGTH)
      return false;
    RawSignal.Pulses[RawCodeLength++]=PulseLength/25;
    PulseLength=WaitForChangeState(DataPin, !StateSignal, TimeOut);
    RawSignal.Pulses[RawCodeLength++]=PulseLength/25;
    }while(RawCodeLength<RAW_BUFFER_SIZE && PulseLength!=0);// Zolang nog niet alle bits ontvangen en er niet vroegtijdig een timeout plaats vindt

  if(RawCodeLength>=MIN_RAW_PULSES)
    {
    RawSignal.Number=RawCodeLength-1;
    return true;
    }
  RawSignal.Number=0;
  return false;
  }
  
 /**********************************************************************************************\
 * Wacht totdat de pin verandert naar status state. Geeft de tijd in uSec. terug. 
 * Als geen verandering, dan wordt na timeout teruggekeerd met de waarde 0L
 \*********************************************************************************************/
unsigned long WaitForChangeState(uint8_t pin, uint8_t state, unsigned long timeout)
	{
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	unsigned long numloops = 0; // keep initialization out of time critical area
	unsigned long maxloops = microsecondsToClockCycles(timeout) / 19;

	// wait for the pulse to stop. One loop takes 19 clock-cycles
	while((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;//timeout opgetreden
	return clockCyclesToMicroseconds(numloops * 19 + 16); 
	}

/*********************************************************************************************\
 * Analyse RF signals to events
\*********************************************************************************************/
boolean AnalyzeRawSignal(struct NodoEventStruct *E)
  {
    ClearEvent(E);
    if(RawSignal_2_Nodo(E)) return true;

    #ifdef RFGATEWAY
      SoftwareSerial_print("R:");
      SoftwareSerial_println(int2str(RawSignal.Number));
      if((RawSignal.Number == 160) or (RawSignal.Number == 176))
        {
          RepeatCounter++;
          if(RepeatCounter >= SendOnCount)
            {
              SoftwareSerial_println("S!");
              RawSendRF();
              RepeatCounter=0;
            }
        }
    #endif
    
    #ifdef NEWKAKU
      if((RawSignal.Number == 132) or (RawSignal.Number == 148)) if (RawSignal_2_NewKAKU(E)) return true;
    #endif
     
    return false;
  }

/*********************************************************************************************\
* Deze routine berekent de uit een RawSignal een NODO code
* Geeft een false retour als geen geldig NODO signaal
\*********************************************************************************************/
boolean RawSignal_2_Nodo(struct NodoEventStruct *Event)
{
  byte b,x,y,z;

  if(RawSignal.Number!=16*sizeof(struct DataBlockStruct)+2) // Per byte twee posities + startbit.
    return false;

  struct DataBlockStruct DataBlock;
  
  byte *B=(byte*)&DataBlock; // B wijst naar de eerste byte van de struct
  z=3;  // RwaSignal pulse teller: 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e pulslengte. Dus start loop met drie.

  for(x=0;x<sizeof(struct DataBlockStruct);x++) // vul alle bytes van de struct 
  {
    b=0;
    for(y=0;y<=7;y++) // vul alle bits binnen een byte
    {
      if((RawSignal.Pulses[z] *25) > NODO_PULSE_MID)
        b|=1<<y; //LSB in signaal wordt  als eerste verzonden
      z+=2;
    }
    *(B+x)=b;
  }

  if(DataBlock.SourceUnit>>5!=HOME_NODO) return false;
  RawSignal.Repeats = false;
  Event->SourceUnit=DataBlock.SourceUnit;  
  Event->DestinationUnit=DataBlock.DestinationUnit;
  Event->Flags=DataBlock.Flags;
  Event->Type=DataBlock.Type;
  Event->Command=DataBlock.Command;
  Event->Par1=DataBlock.Par1;
  Event->Par2=DataBlock.Par2;
  Event->Version=DataBlock.Version;
  Event->Checksum=DataBlock.Checksum;
  if(Checksum(Event)) return true;

  return false; 
}

  
#ifdef NEWKAKU
/*********************************************************************************************\
* Decode NewKaku signals
\*********************************************************************************************/
#define NewKAKU_RawSignalLength      132
#define NewKAKUdim_RawSignalLength   148
#define NewKAKU_1T                   275        // us
#define NewKAKU_mT                   500        // us, midden tussen 1T en 4T 
#define NewKAKU_4T                  1100        // us
#define NewKAKU_8T                  2200        // us, Tijd van de space na de startbit
boolean RawSignal_2_NewKAKU(struct NodoEventStruct *event)
  {
  unsigned long bitstream=0L;
  boolean Bit;
  int i;
  int P0,P1,P2,P3;
  int PTMF=25;
  event->Par1=0;

  // nieuwe KAKU bestaat altijd uit start bit + 32 bits + evt 4 dim bits. Ongelijk, dan geen NewKAKU
  if (RawSignal.Number==NewKAKU_RawSignalLength || (RawSignal.Number==NewKAKUdim_RawSignalLength))
    {
    // RawSignal.Number bevat aantal pulsen * 2  => negeren
    // RawSignal.Pulses[1] bevat startbit met tijdsduur van 1T => negeren
    // RawSignal.Pulses[2] bevat lange space na startbit met tijdsduur van 8T => negeren
    i=3; // RawSignal.Pulses[3] is de eerste van een T,xT,T,xT combinatie
        
    do 
      {
      P0=RawSignal.Pulses[i]    * PTMF;
      P1=RawSignal.Pulses[i+1]  * PTMF;
      P2=RawSignal.Pulses[i+2]  * PTMF;
      P3=RawSignal.Pulses[i+3]  * PTMF;
          
      if     (P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3>NewKAKU_mT)Bit=0; // T,T,T,4T
      else if(P0<NewKAKU_mT && P1>NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)Bit=1; // T,4T,T,T
      else if(P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)       // T,T,T,T Deze hoort te zitten op i=111 want: 27e NewKAKU bit maal 4 plus 2 posities voor startbit
        {
        if(RawSignal.Number!=NewKAKUdim_RawSignalLength) // als de dim-bits er niet zijn
          return false;
        }
      else
        return false; // andere mogelijkheden zijn niet geldig in NewKAKU signaal.  
            
      if(i<130) // alle bits die tot de 32-bit pulstrein behoren 32bits * 4posities per bit + pulse/space voor startbit
        bitstream=(bitstream<<1) | Bit;
      else // de resterende vier bits die tot het dimlevel behoren 
        event->Par1=(event->Par1<<1) | Bit;
       
      i+=4;// volgende pulsenquartet
      }while(i<RawSignal.Number-2); //-2 omdat de space/pulse van de stopbit geen deel meer van signaal uit maakt.
            
    // Adres deel:
    if(bitstream>0xffff)                         // Is het signaal van een originele KAKU zender afkomstig, of van een Nodo ingegeven door de gebruiker ?
    event->Par2=bitstream &0xFFFFFFCF;         // dan hele adres incl. unitnummer overnemen. Alleen de twee commando-bits worden er uit gefilterd
        
    else                                         // Het is van een andere Nodo afkomstig. 
      event->Par2=(bitstream>>6)&0xff;           // Neem dan alleen 8bit v/h adresdeel van KAKU signaal over
          
    // Commando en Dim deel
    if(i>140)
      event->Par1++;                             // Dim level. +1 omdat gebruiker dim level begint bij één.
    else
      event->Par1=((bitstream>>4)&0x01)?VALUE_ON:VALUE_OFF; // On/Off bit omzetten naar een Nodo waarde. 

    event->Type          = NODO_TYPE_PLUGIN_EVENT;
    event->Command       = CMD_PLUGIN_KAKU_NEW;
    event->SourceUnit    = 0;                     // Komt niet van een Nodo unit af, dus unit op nul zetten
    RawSignal.Repeats    = true;                  // het is een herhalend signaal. Bij ontvangst herhalingen onderdrukken.
    return true;
    }
  return false;
  }

#endif // NEWKAKU
#endif // RF_RECEIVE

#ifdef RF_SEND

/*********************************************************************************************\
 * Calculate the RAW pulses for a 32-bit Nodo-code and store into RawSignal buffer
\*********************************************************************************************/
void Nodo_2_RawSignal(byte Type, byte Command, byte Par1, unsigned long Par2)
{
  struct DataBlockStruct DataBlock;
  byte BitCounter=1;

  struct NodoEventStruct Event;
  Event.Type            = Type;
  Event.Command         = Command;
  Event.Par1            = Par1;
  Event.Par2            = Par2;
  Event.SourceUnit      = Settings.Unit;  
  Event.DestinationUnit = 0;
  Event.Flags           = 0;
  Event.Port            = VALUE_SOURCE_RF;
  Event.Direction       = VALUE_DIRECTION_OUTPUT;
  Event.Version         = 0;
  Event.Checksum        = 0;
  Checksum(&Event);
  
  DataBlock.SourceUnit      = Event.SourceUnit | (HOME_NODO<<5);  
  DataBlock.DestinationUnit = Event.DestinationUnit;
  DataBlock.Flags           = Event.Flags;
  DataBlock.Type            = Event.Type;
  DataBlock.Command         = Event.Command;
  DataBlock.Par1            = Event.Par1;
  DataBlock.Par2            = Event.Par2;
  DataBlock.Checksum        = Event.Checksum;
  DataBlock.Version         = NODO_VERSION_MINOR;

  byte *B=(byte*)&DataBlock;
  
  // begin met een startbit. 
  RawSignal.Pulses[BitCounter++]=(NODO_PULSE_1*4)/25;
  RawSignal.Pulses[BitCounter++]=(NODO_SPACE*2)/25;

  for(byte x=0;x<sizeof(struct DataBlockStruct);x++)
  {
    for(byte Bit=0; Bit<=7; Bit++)
    {
      if((*(B+x)>>Bit)&1)
        RawSignal.Pulses[BitCounter++]=NODO_PULSE_1/25; 
      else
        RawSignal.Pulses[BitCounter++]=NODO_PULSE_0/25;   
      RawSignal.Pulses[BitCounter++]=NODO_SPACE/25;   
    }
  }
  
  RawSignal.Pulses[BitCounter-1]=NODO_SPACE/25; // pauze tussen de pulsreeksen, vast ingesteld op 6375 uSec (maximum)
  RawSignal.Number=BitCounter;
}

/*********************************************************************************************\
 * Send Nodo message through RF transmitter
\*********************************************************************************************/
void RawSendRF(void)
{
  int x;
  RawSignal.Pulses[RawSignal.Number]=1;
  for(byte y=0; y < RF_REPEATS; y++) // herhaal verzenden RF code
  {
    x=1;
    while(x<RawSignal.Number)
    {
      digitalWrite(PIN_RF_TX_DATA,HIGH); // 1
      delayMicroseconds(RawSignal.Pulses[x++]*25); 
      digitalWrite(PIN_RF_TX_DATA,LOW); // 0
      delayMicroseconds(RawSignal.Pulses[x++]*25); 
    }
  }
}
#endif // RF_SEND

#if defined(RF_SEND) || defined(RF_RECEIVE) || defined(NRF24L01)
/*********************************************************************************************\
* Calculate Nodo RF checkum
\*********************************************************************************************/
boolean Checksum(NodoEventStruct *event)
  {
  byte OldChecksum=event->Checksum;
  byte NewChecksum=NODO_VERSION_MAJOR;  // Verwerk versie in checksum om communicatie tussen verschillende versies te voorkomen

  event->Checksum=0; // anders levert de beginsituatie een andere checksum op

  for(int x=0;x<sizeof(struct NodoEventStruct);x++)
    NewChecksum^(*((byte*)event+x)); 

  event->Checksum=NewChecksum;
  return(OldChecksum==NewChecksum);
  }
#endif

/*********************************************************************************************\
 * Clear event structure.
\*********************************************************************************************/
void ClearEvent(struct NodoEventStruct *Event)
{    
  Event->Command            = 0;
  Event->Par1               = 0;
  Event->Par2               = 0L;
  Event->Type               = 0;
  Event->Flags              = 0;
  Event->Port               = 0;
  Event->Direction          = 0;
  Event->DestinationUnit    = 0;
  Event->SourceUnit         = Settings.Unit;
  Event->Version            = NODO_VERSION_MINOR;
  Event->Checksum           = 0;
}

/*********************************************************************************************\
 * Save settings to EEPROM memory.
\*********************************************************************************************/
void Save_Settings(void)  
{
  char ByteToSave,*pointerToByteToSave=pointerToByteToSave=(char*)&Settings;    //pointer verwijst nu naar startadres van de struct. 

  for(int x=0; x<sizeof(struct SettingsStruct) ;x++)
  {
    EEPROM.write(x,*pointerToByteToSave); 
    pointerToByteToSave++;
  }  
}

/*********************************************************************************************\
 * Laad de settings uit het EEPROM geheugen.
 \*********************************************************************************************/
boolean LoadSettings()
{
  byte x;

  char ByteToSave,*pointerToByteToRead=(char*)&Settings;    //pointer verwijst nu naar startadres van de struct.

  for(int x=0; x<sizeof(struct SettingsStruct);x++)
  {
    *pointerToByteToRead=EEPROM.read(x);
    pointerToByteToRead++;// volgende byte uit de struct
  }
}

unsigned long float2ul(float f)
  {
  unsigned long ul;
  memcpy(&ul, &f,4);
  return ul;
  }
  
float ul2float(unsigned long ul)
  {
  float f;
  memcpy(&f, &ul,4);
  return f;
  }

/*********************************************************************************************\
 * Mini MMI
\*********************************************************************************************/
#ifdef MINI_MMI

byte Par[16];

void UserPlugin_MMI_Periodically()
{
  UserPlugin_MiniMMI();
  return;
}

void UserPlugin_MiniMMI(void)
{
  byte SerialInByte;

  if(SoftwareSerial_available())
  {
    delay(50);
    
    for(byte x=0; x < 16; x++) Par[x]=0;
    SerialInByte=SoftwareSerial_read();

    if (SerialInByte > 16) SerialInByte=16;

    for(byte x=0; x < SerialInByte; x++) Par[x]=SoftwareSerial_read();

    if (Par[0] == 's')
      {
        int baseaddress=sizeof(struct SettingsStruct);
        SoftwareSerial_write(0xff); // Byte value to signal start of binary data
        SoftwareSerial_write((byte)(EVENTLIST_MAX*14+1) & 0xff); // send message length, LSB
        SoftwareSerial_write((byte)(((EVENTLIST_MAX*14+1) >> 8) & 0xff)); // send message length, MSB
        SoftwareSerial_write(0x1); // send messagetype (1=eventlistshow)
        for (int x=0; x < EVENTLIST_MAX*14; x++) SoftwareSerial_write(EEPROM.read(baseaddress+x));
      }
      
    if (Par[0] == 'c')
      {
        NodoEventStruct TempEvent;
        TempEvent.Type=Par[1];
        TempEvent.Command = Par[2];
        TempEvent.Par1 = Par[3];
        TempEvent.Par2 = 0;
        for (byte x=4; x < 8; x++)
          {
            TempEvent.Par2 = TempEvent.Par2 + ((unsigned long)Par[x] << (8*(x-4)));
          }
        TempEvent.Port = VALUE_SOURCE_RF;
        TempEvent.SourceUnit=0;
        TempEvent.DestinationUnit=0;
        TempEvent.Direction=VALUE_DIRECTION_INPUT;
        ExecuteCommand(&TempEvent,0);
      }
      
    if (Par[0] == 'w')
      {
        int baseaddress=sizeof(struct SettingsStruct)+(Par[1]-1)*14;
        for (byte x=2; x < 16; x++)
          {
            EEPROM.write(baseaddress+x-2, Par[x]);
          }
      }
    SoftwareSerial_println("ok");
    SoftwareSerial_flush();
  }
}
#endif // MINI MMI

#ifdef I2C
boolean ReceiveI2C(void)
{
  byte b,*B=(byte*)&TempEvent;
  byte Checksum=0;
  int x=0;

  while(TinyWireS_available()) // Haal de bytes op
  {
    b=TinyWireS_receive(); 
    if(x<sizeof(struct NodoEventStruct))
    {
      *(B+x)=b; 
      Checksum^=b; 
    }
    x++;
  }

  // laatste ontvangen byte bevat de checksum. Als deze gelijk is aan de berekende checksum, dan event uitvoeren
  if(b==Checksum)    
  {   
    return true;
  }
  else
    return false;
}

boolean ReplyI2C(struct NodoEventStruct *EventBlock)
{  
  byte x;
  // bereken checksum: crc-8 uit alle bytes in de queue.
  byte b,*B=(byte*)EventBlock;
  byte Checksum=0;
  for(x=0;x<sizeof(struct NodoEventStruct);x++)
  {
    b=*(B+x); 
    TinyWireS_send(b);
    Checksum^=b; 
  }
  TinyWireS_send(Checksum);
}
#endif

/*********************************************************************************************\
* Sleep stuff
\*********************************************************************************************/
#ifdef BATTERYMODE
#include <avr/sleep.h>
#include <avr/wdt.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void Watchdog_setup(int ii)
{  
  // The prescale value is held in bits 5,2,1,0
  // This block moves ii itno these bits
  byte bb;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  
  // Reset the watchdog reset flag
  MCUSR &= ~(1<<WDRF);
  // Start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // Set new watchdog timeout value
  WDTCR = bb;
  // Enable interrupts instead of reset
  WDTCR |= _BV(WDIE);
}

void Watchdog_sleep(int waitTime)
{
  // Calculate the delay time
  int waitCounter = 0;
  while (waitCounter != waitTime) 
  {
    cbi(ADCSRA,ADEN); // Switch Analog to Digital converter OFF 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode
    sleep_mode(); // System sleeps here
    sbi(ADCSRA,ADEN);  // Switch Analog to Digital converter ON
    waitCounter++;
  }
}

ISR(WDT_vect) 
{
  // Don't do anything here but we must include this
  // block of code otherwise the interrupt calls an
  // uninitialized interrupt handler.
}

#endif

/**********************************************************************************************\
 * Converteert een unsigned long naar een string met decimale integer.
 \*********************************************************************************************/
char* int2str(unsigned long x)
{
  //static char OutputLine[12];
  char* OutputLinePosPtr=&OutputLine[10];
  int y;

  *OutputLinePosPtr=0;

  if(x==0)
  {
    *--OutputLinePosPtr='0';
  }
  else
  {  
    while(x>0)
    {
      *--OutputLinePosPtr='0'+(x%10);
      x/=10;
    }
  }    
  return OutputLinePosPtr;
}

/**********************************************************************************************\
 * Converteert een unsigned long naar een hexadecimale string.
 \*********************************************************************************************/
char* int2strhex(unsigned long x)
{
  //static char OutputLine[12];
  char* OutputLinePosPtr=&OutputLine[10];
  int y;

  *OutputLinePosPtr=0;

  if(x==0)
  {
    *--OutputLinePosPtr='0';
  }
  else
  {  
    while(x>0)
    {
      y=x&0xf;

      if(y<10)
        *--OutputLinePosPtr='0'+y;
      else
        *--OutputLinePosPtr='A'+(y-10);

      x=x>>4;
      ;
    }
    *--OutputLinePosPtr='x';
    *--OutputLinePosPtr='0';
  }
  return OutputLinePosPtr;
}

/*********************************************************************************************\
* Debug stuff
\*********************************************************************************************/
#ifdef UP_DEBUG
void printFreeRam(void)
{
  SoftwareSerial_write('R');
  SoftwareSerial_println(int2str(freeRam()));
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif


