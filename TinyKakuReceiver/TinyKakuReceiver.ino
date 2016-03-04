//==============================================================================================================
// Arduino project "Kaku Compatible Receiver" © Copyright 2013 Martinus van den Broek
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
// as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.//
//
// You received a copy of the GNU General Public License along with this program in tab '_COPYING'.
//==============================================================================================================
// Purpose        : Arduino Source code for a Kaku Compatible Receiver, based on Atmel ATTiny85, running on 8MHz
// Version        : R46
// Date           : 20130421
// 
// This code turns an Atmel ATTiny85 chip into a Kaku compatible receiver.
// This receiver is compatible ONLY with newer type of KAKU transmitters with automatic code select
// (the 'A' types)
//==============================================================================================================
// ATMEL ATTINY85 KAKU RECEIVER PINOUT CONFIG:
//
//                          o-\/-+ 
//                  Reset  1|    |8  VCC 
//          RF RX (input)  2|    |7  Serial TX (Output)
//    Push Button (input)  3|    |6  Serial RX (Input)
//                    GND  4|    |5  LED (PWM output)
//                          +----+ 
//==============================================================================================================
// Getting started:
// Connect the datapin from an RF Receiver to physical pin 2
// Connect an FTDI cable to physical pins 6 and 7
// Launch Serial Monitor select COM port and set the baudrate to 9600
// Type ? <enter> to check communication, should respond with 'KAKU Compatible Receiver'
// Type L <enter> to start learn mode
// Press the ON button on some KAKU transmitter
// Start having fun...
// Type e <enter> to erase memory
// You can also learn and erase with the pushbutton (connect to ground)
// Press shortly to start learn (will wait for max 30 secs)
// Press for > 5 seconds and release, all KAKU addresses will be erased!
//   The LED will blink once to confirm learn mode
//   The LED will blink twice to conform end of learn mode
//   The LED will blink three times to confirm erase mode
//==============================================================================================================
#include <EEPROM.h>

#define LED_PIN             0
#define SERIAL_RX_PIN       1
#define SERIAL_TX_PIN       2
#define RF_RX_PIN           3
#define BUTTON_PIN          4
#define EEPROM_BASE_ADDR  128
#define NEWKAKUIRQ
#define MAX_ADDRESS         6

prog_char PROGMEM Text_01[] = "KAKU Compatible Receiver";
prog_char PROGMEM Text_02[] = "Licensed under GNU General Public License.";
prog_char PROGMEM Text_03[] = "KAKU Transmitters stored in EEPROM:";
prog_char PROGMEM Text_04[] = "Learning mode started...";
prog_char PROGMEM Text_05[] = "Learning mode ended... ";
prog_char PROGMEM Text_06[] = "Address : Channel";
prog_char PROGMEM Text_07[] = "PWM Value set to:";
prog_char PROGMEM Text_08[] = "Address:";
prog_char PROGMEM Text_09[] = "Channel:";
prog_char PROGMEM Text_10[] = "Dim Value:";
prog_char PROGMEM Text_11[] = "Off";
prog_char PROGMEM Text_12[] = "On";
prog_char PROGMEM Text_13[] = "Group";

struct NewRemoteCode {
  unsigned int period;		// Detected duration in microseconds of 1T in the received signal
  unsigned long address;	// Address of received code. [0..2^26-1]
  boolean groupBit;		// Group bit set or not
  unsigned short switchType;	// 0: swich off; 1: switch on; 2: set dim level
  unsigned short unit;		// Unit code of received code [0..15]
  unsigned short dimLevel;	// Dim level [0..15] iff switchType == 2
};

struct NewRemoteCode NewKaku;
volatile unsigned short _state = -1;		// State of decoding process. There are 49 states, 1 for "waiting for signal" and 48 for decoding the 48 edges in a valid code.
unsigned short _minRepeats     =  0;
byte volatile RFState          =  0;
char OutputLine[12];                            // made global for both int2str and int2strhex, to save RAM.
boolean SoftwareSerial         =  true;
byte PWMValue                  =  0;
boolean Dimming                =  false;
boolean DimUp                  =  true;
unsigned long lastSignal       =  millis();
byte lastPWM                   =  255;

/**********************************************************************************************\
 * Setup
\*********************************************************************************************/
void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  
  SoftwareSerial_init(SERIAL_RX_PIN,SERIAL_TX_PIN);         // RX, TX
  SoftwareSerial_println(ProgmemString(Text_01));
  SoftwareSerial_println(ProgmemString(Text_02));

  // enable RF interrupt on pin change
  if (digitalPinToPCICR(RF_RX_PIN))
  {
    *digitalPinToPCICR(RF_RX_PIN) |= _BV(digitalPinToPCICRbit(RF_RX_PIN));
    *digitalPinToPCMSK(RF_RX_PIN) |= _BV(digitalPinToPCMSKbit(RF_RX_PIN));
  }
  NewKaku.address = 0;
}


/**********************************************************************************************\
 * Main program loop
\*********************************************************************************************/
void loop()
{
  if (NewKaku.address)
  {
    if (Eeprom_check(NewKaku.address,NewKaku.unit))
    {
      switch (NewKaku.switchType) {
      case 0:
        if (PWMValue > 0) lastPWM = PWMValue;
        PWMValue = 0;
        Dimming = false;
        delay(100);
        break;
      case 1:
        if (PWMValue > 0)
        {
          if (Dimming)
            {
              Dimming = false;
              if (DimUp) DimUp = false;
                else DimUp = true;
            }
          else
            Dimming = true;
        }
        else 
          PWMValue = lastPWM;
          delay(100);
        break;
      case 2:
        PWMValue = ((NewKaku.dimLevel + 1) * 16) - 1;
        break;
      }
      analogWrite(LED_PIN,PWMValue);
      SoftwareSerial_print(ProgmemString(Text_07));
      SoftwareSerial_println(int2str(PWMValue));
    }
    showCode(NewKaku);
    NewKaku.address=0;
  }

  if (Dimming)
    {
      if (PWMValue == 255) DimUp = false;
      if (PWMValue == 1) DimUp = true;
      if (DimUp)
        PWMValue++;
      else
        PWMValue--;
      analogWrite(LED_PIN,PWMValue);
      delay(25);
    }
    
  if(SoftwareSerial_available())
  {
    byte serial = SoftwareSerial_read();
      switch (serial) {
      case '?':
        SoftwareSerial_println(ProgmemString(Text_01));
        SoftwareSerial_println(ProgmemString(Text_02));
        Eeprom_list();
        break;
      case 'l':
      case 'L':
        learnAddress();
        break;
      case 'e':
      case 'E':
        Eeprom_wipe();
        break;
      case 'd':
      case 'D':
        SoftwareSerial_print("RAM:");
        SoftwareSerial_println(int2str(freeRam()));
        break;
      default:
        if (serial != 10 && serial != 13) SoftwareSerial_println("?");
        break;
      }
  }

  if (digitalRead(BUTTON_PIN) == 0)
    {
      delay(100);
      unsigned long holdtimer = millis();
      while (digitalRead(BUTTON_PIN) == 0) delay(1);
      if ((millis()-holdtimer) < 5000)
        {
          blink(1);
          learnAddress();
          blink(2);
        }
      else
        {
          Eeprom_wipe();
          blink(3);
        }
    }
}

/**********************************************************************************************\
 * Blink light to signal learn mode
\*********************************************************************************************/
void blink(byte repeat)
{
  for (byte x=0; x < repeat; x++)
  {
    analogWrite(LED_PIN,255);
    delay(500);
    analogWrite(LED_PIN,0);
    delay(500);
  }
}


/**********************************************************************************************\
 * Learn Kaku address and store into EEPROM
\*********************************************************************************************/
void learnAddress(void)
{
  unsigned long timer = millis() + 30000;
  unsigned long tmpaddress=0;
  byte tmpchannel=0;

  SoftwareSerial_println(ProgmemString(Text_04));

  while (millis() < timer)
    {
      // If valid address, mode = on and not already stored
      if ((NewKaku.address > 0)  && (NewKaku.switchType == 1) && !Eeprom_check(NewKaku.address,NewKaku.unit))
        {
          for(byte x=0; x < MAX_ADDRESS; x++)
          {
            Eeprom_load(EEPROM_BASE_ADDR + (5*x), &tmpaddress, &tmpchannel);
            if (tmpaddress == 0)
              {
              Eeprom_save(EEPROM_BASE_ADDR + (5*x),NewKaku.address,NewKaku.unit);
              Eeprom_list();
              NewKaku.address=0;
              timer = millis();
              break;
              }
          }
        }

      // If valid address, mode = off and stored
      if ((NewKaku.address > 0)  && (NewKaku.switchType == 0) && Eeprom_check(NewKaku.address,NewKaku.unit))
        {
          for(byte x=0; x < MAX_ADDRESS; x++)
          {
            Eeprom_load(EEPROM_BASE_ADDR + (5*x), &tmpaddress, &tmpchannel);
            if ((tmpaddress == NewKaku.address) && (tmpchannel == NewKaku.unit))
              {
              Eeprom_save(EEPROM_BASE_ADDR + (5*x), 0, 0);
              Eeprom_list();
              NewKaku.address=0;
              timer = millis();
              break;
              }
          }
        }

    }

  SoftwareSerial_println(ProgmemString(Text_05));
}


/**********************************************************************************************\
 * Show received Kaku event through serial port
\*********************************************************************************************/
void showCode(struct NewRemoteCode receivedCode) {

  // Print the received code.
  SoftwareSerial_print(ProgmemString(Text_08));
  SoftwareSerial_print(int2str(receivedCode.address));
  SoftwareSerial_write(',');
  SoftwareSerial_write(' ');

  if (receivedCode.groupBit) {
    SoftwareSerial_print(ProgmemString(Text_13));
  } 
  else {
    SoftwareSerial_print(ProgmemString(Text_09));
    SoftwareSerial_print(int2str(receivedCode.unit));
  }

  SoftwareSerial_write(',');
  SoftwareSerial_write(' ');

  switch (receivedCode.switchType) {
  case 0:
    SoftwareSerial_print(ProgmemString(Text_11));
    break;
  case 1:
    SoftwareSerial_print(ProgmemString(Text_12));
    break;
  case 2:
    SoftwareSerial_print(ProgmemString(Text_10));
    SoftwareSerial_print(int2str(receivedCode.dimLevel));
    break;
  }

  SoftwareSerial_println("");
}


/**********************************************************************************************\
 * Convert unsigned long to string
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


/*********************************************************************************************\
 * List Learned addresses
\*********************************************************************************************/
void Eeprom_list(void)
{
  SoftwareSerial_println(ProgmemString(Text_03));
  SoftwareSerial_println(ProgmemString(Text_06));
  unsigned long address;
  byte channel;
  for (byte x=0; x < MAX_ADDRESS; x++)
  {
    Eeprom_load(EEPROM_BASE_ADDR + (5*x), &address, &channel);
    SoftwareSerial_print(int2str(address));
    SoftwareSerial_print(" : ");
    SoftwareSerial_println(int2str(channel));
  }
}


/*********************************************************************************************\
 * Wipe Learned addresses
\*********************************************************************************************/
void Eeprom_wipe(void)
{
  for (byte x=0; x < MAX_ADDRESS; x++)
  {
    Eeprom_save(EEPROM_BASE_ADDR + (5*x), 0, 0);
  }
}


/*********************************************************************************************\
 * Check KAKU address
\*********************************************************************************************/
boolean Eeprom_check(unsigned long address, byte channel)
{
  unsigned long tmpaddress;
  byte tmpchannel;
  for (byte x=0; x < MAX_ADDRESS; x++)
  {
    Eeprom_load(EEPROM_BASE_ADDR + (5*x), &tmpaddress, &tmpchannel);
    if ((tmpaddress == address) && (tmpchannel == channel)) return true;
  }
  return false;
}


/*********************************************************************************************\
 * Save to EEPROM
\*********************************************************************************************/
void Eeprom_save(int address, unsigned long Value, byte Channel)
{
  EEPROM.write(address,(Value>>24  & 0xFF));
  EEPROM.write(address+1,(Value>>16  & 0xFF));
  EEPROM.write(address+2,(Value>> 8  & 0xFF));
  EEPROM.write(address+3,(Value      & 0xFF));
  EEPROM.write(address+4,Channel);
}


/*********************************************************************************************\
 * Load from EEPROM
\*********************************************************************************************/
void Eeprom_load(int address, unsigned long *Value, byte *Channel)
{
  *Value  = ((unsigned long)(EEPROM.read(address))) << 24;
  *Value |= ((unsigned long)(EEPROM.read(address+1))) << 16;
  *Value |= ((unsigned long)(EEPROM.read(address+2))) <<  8;
  *Value |= ((unsigned long)(EEPROM.read(address+3)))      ;
  *Channel = EEPROM.read(address+4);
  return;
}

/**********************************************************************************************\
 * Read strings from PROGMEM
 \*********************************************************************************************/
char* ProgmemString(prog_char* text)
{
  byte x=0;
  static char buffer[50];

  do
  {
    buffer[x]=pgm_read_byte_near(text+x);
  }
  while(buffer[x++]!=0);
  return buffer;
}

/*********************************************************************************************\
 * Debug stuff
 \*********************************************************************************************/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

