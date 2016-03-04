//#######################################################################################################
//#################################### Plugin-21: LCD I2C 1602 ##########################################
//#######################################################################################################

/*********************************************************************************************\
* Dit protocol zorgt voor communicatie met een LCD Display dat wordt bestuurd via I2C/TWI
* Er worden twee versies ondersteund, aangeduidt met 1602 (2 x 16 tekens) of 2004 (4 x 20 tekens)
* Getest op een I2C/TWI LCD1602 van DFRobot en een Funduino I2C LCD2004
* Auteur             : Martinus van den Broek
* Support            : www.nodo-domotica.nl
* Datum              : 27 Maart 2014
* Versie             : 12-2013 versie 1.2 Modificatie WireNodo library (Hans Man)
*                      02-2014 versie 1.3 Support 4x20 display en uitbreiding functionaliteit met Par3/Par4 (Martinus)
*                      02-2014 versie 1.4 Support PortInput, LCDWrite 0,0 clears screen (Martinus)
*                      03-2014 versie 1.5 Support Backlight on/off (Martinus)
*                      03-2014 versie 1.6 Fix buffer issue in progmem (Martinus)
*                      03-2014 versie 1.7 Fix I2C Multimaster "conflict" (Martinus)
* Nodo productnummer : 
* Compatibiliteit    : Vanaf Nodo build nummer 707
* Syntax             : "LCDWrite <row>, <column>, <command>, <option>
*
*                       Command:	Message		<option> = ID in plugin label definities
*					Variable	<option> = Variabele nummer
*					Clock		toont datum en tijd		
*					IP		toont IP adres (alleen Mega)
*					PortInput	toont input port voor http verkeer (alleen Mega)
*					Event		toont laatste event (alleen Mega)
*
*                       Special commands:
*					<x>,<0>, Reset	wist regel x
*					<0>,<0>, Reset	wist scherm
*					<0>,<0>, On	backlight aan
*					<0>,<0>, Off	backlight uit
***********************************************************************************************
* Technische beschrijving:
*
* De LCDI2C1602 is een LCD Display van twee regels en 16 tekens per regel.
* De aansturing vindt plaats via de standaard TwoWire interface van de Nodo (I2C)
* Er wordt op het convertor board gebruik gemaakt van een PCF8574 op adres 0x27
* Mocht je ook losse PCF8574 chips gebruiken met Plugin 025, stel deze dan niet in op 0x27!
* Er kan dus maar 1 display per I2C bus worden aangesloten.
\*********************************************************************************************/

#define PLUGIN_ID 21
#define PLUGIN_NAME "LCDWrite"

prog_char PROGMEM LCD_01[] = "Nodo Domotica";
prog_char PROGMEM LCD_02[] = "Mega R:%03d U:%d";
prog_char PROGMEM LCD_03[] = "Small R:%03d U:%d";
prog_char PROGMEM LCD_04[] = PLUGIN_021_LABEL_04;
prog_char PROGMEM LCD_05[] = PLUGIN_021_LABEL_05;
prog_char PROGMEM LCD_06[] = PLUGIN_021_LABEL_06;
prog_char PROGMEM LCD_07[] = PLUGIN_021_LABEL_07;
prog_char PROGMEM LCD_08[] = PLUGIN_021_LABEL_08;
prog_char PROGMEM LCD_09[] = PLUGIN_021_LABEL_09;
prog_char PROGMEM LCD_10[] = PLUGIN_021_LABEL_10;
prog_char PROGMEM LCD_11[] = PLUGIN_021_LABEL_11;
prog_char PROGMEM LCD_12[] = PLUGIN_021_LABEL_12;
prog_char PROGMEM LCD_13[] = PLUGIN_021_LABEL_13;
prog_char PROGMEM LCD_14[] = PLUGIN_021_LABEL_14;
prog_char PROGMEM LCD_15[] = PLUGIN_021_LABEL_15;
prog_char PROGMEM LCD_16[] = PLUGIN_021_LABEL_16;
prog_char PROGMEM LCD_17[] = PLUGIN_021_LABEL_17;
prog_char PROGMEM LCD_18[] = PLUGIN_021_LABEL_18;
prog_char PROGMEM LCD_19[] = PLUGIN_021_LABEL_19;
prog_char PROGMEM LCD_20[] = PLUGIN_021_LABEL_20;

#define LCDI2C_MSG_MAX        20

PROGMEM const char *LCDText_tabel[]={LCD_01,LCD_02,LCD_03,LCD_04,LCD_05,LCD_06,LCD_07,LCD_08,LCD_09,LCD_10,LCD_11,LCD_12,LCD_13,LCD_14,LCD_15,LCD_16,LCD_17,LCD_18,LCD_19,LCD_20};

#define LCD_I2C_ADDRESS 0x27

#if PLUGIN_021_CORE==2
  #define PLUGIN_021_ROWS  2
  #define PLUGIN_021_COLS 16
#else
  #define PLUGIN_021_ROWS  4
  #define PLUGIN_021_COLS 20
#endif

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En B00000100  // Enable bit
#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

void LCD_I2C_printline(byte row, byte col, char* message);
inline size_t LCD_I2C_write(uint8_t value);
void LCD_I2C_display();
void LCD_I2C_clear();
void LCD_I2C_home();
void LCD_I2C_setCursor(uint8_t col, uint8_t row);
inline void LCD_I2C_command(uint8_t value);
void LCD_I2C_send(uint8_t value, uint8_t mode);
void LCD_I2C_write4bits(uint8_t value);
void LCD_I2C_expanderWrite(uint8_t _data);                                        
void LCD_I2C_pulseEnable(uint8_t _data);

#ifdef PLUGIN_021_CORE
uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _numlines;
uint8_t _backlightval=LCD_BACKLIGHT;
#endif

boolean Plugin_021(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  switch(function)
   {
#ifdef PLUGIN_021_CORE
   case PLUGIN_INIT:
     {
     _displayfunction = LCD_2LINE;
     _numlines = PLUGIN_021_ROWS;
     delay(50); 
     // Now we pull both RS and R/W low to begin commands
     LCD_I2C_expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
     delay(1000);

     //put the LCD into 4 bit mode, this is according to the hitachi HD44780 datasheet, figure 24, pg 46
     LCD_I2C_write4bits(0x03 << 4);        // we start in 8bit mode, try to set 4 bit mode
     delayMicroseconds(4500);              // wait min 4.1ms
     LCD_I2C_write4bits(0x03 << 4);        // second try
     delayMicroseconds(4500);              // wait min 4.1ms
     LCD_I2C_write4bits(0x03 << 4);        // third go!
     delayMicroseconds(150);
     LCD_I2C_write4bits(0x02 << 4);        // finally, set to 4-bit interface
     LCD_I2C_command(LCD_FUNCTIONSET | _displayfunction);              // set # lines, font size, etc.
     _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;   // turn the display on with no cursor or blinking default
     LCD_I2C_display();
     LCD_I2C_clear();                                                  // clear it off
     _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;           // Initialize to default text direction (for roman languages)
     LCD_I2C_command(LCD_ENTRYMODESET | _displaymode);                 // set the entry mode
     LCD_I2C_home();

     LCD_I2C_printline(0,0,ProgmemString(LCD_01));
     char TempString[PLUGIN_021_COLS+1];
     #if NODO_MEGA
      sprintf(TempString,ProgmemString(LCD_02), NODO_BUILD, Settings.Unit);
     #else
       sprintf(TempString,ProgmemString(LCD_03), NODO_BUILD, Settings.Unit);
     #endif
     LCD_I2C_printline(1,0,TempString);
     }
  case PLUGIN_COMMAND:
     {
     byte Par2=event->Par2 & 0xff;		// Column
     byte Par3=event->Par2>>8 & 0xff;		// Data to display
     byte Par4=event->Par2>>16 & 0xff;		// In case of var, variable number
     boolean Print = true;

     if (event->Par1 >= 0 && event->Par1 <= PLUGIN_021_ROWS)
       {
       #if NODO_MEGA
         char TempString[80];
       #else
         char TempString[PLUGIN_021_COLS+1];
       #endif
       TempString[0]=0;

       switch (Par3)
         {
           case 0:
           case CMD_RESET:
             if (event->Par1 == 0)
               LCD_I2C_clear();
             else
               LCD_I2C_printline(event->Par1-1,0, "");
             Print=false;
             break;

           case VALUE_ON:
             _backlightval=LCD_BACKLIGHT;
             LCD_I2C_expanderWrite(0);
             Print=false;
             break;

           case VALUE_OFF:
             _backlightval=LCD_NOBACKLIGHT;
             LCD_I2C_expanderWrite(0);
             Print=false;
             break;

           case EVENT_MESSAGE:
             if ((Par4 > 0) && (Par4 <= LCDI2C_MSG_MAX))
              strcpy_P(TempString,(char*)pgm_read_word(&(LCDText_tabel[Par4-1])));
             break;

           case EVENT_VARIABLE:
             if (Par4 > 0 && Par4 <16)
               {
                 #if NODO_MEGA
                   dtostrf(UserVar[Par4-1], 0, 2,TempString);
                 #else
                   int d1 = UserVar[Par4-1];            // Get the integer part
                   float f2 = UserVar[Par4-1] - d1;     // Get fractional part
                   int d2 = trunc(f2 * 10);   // Turn into integer
                   if (d2<0) d2=d2*-1;
                   sprintf(TempString,"%d.%01d", d1,d2);
                 #endif
               }
             break;

           #if CLOCK
           case VALUE_SOURCE_CLOCK:	// Display date/time
             sprintf(TempString,"%02d-%02d-%04d %02d:%02d",Time.Date,Time.Month,Time.Year, Time.Hour, Time.Minutes);
             break;
           #endif

           #if NODO_MEGA
           #ifdef ethernetserver_h
           case CMD_NODO_IP:	// Display IP on Mega
           sprintf(TempString,"%u.%u.%u.%u", EthernetNodo.localIP()[0],EthernetNodo.localIP()[1],EthernetNodo.localIP()[2],EthernetNodo.localIP()[3]);
           break;

           case CMD_PORT_INPUT:	// Display Port on Mega
             sprintf(TempString,"%s",int2str(Settings.PortInput));
             break;
           #endif
           case VALUE_RECEIVED_EVENT:	// Display event on Mega
             Event2str(&LastReceived,TempString);
             break;
           #endif

         }  // case

         if (Print)
           LCD_I2C_printline(event->Par1-1, Par2-1, TempString);

         WireNodo.endTransmission(true);
         success=true;
       }

     break;
     }
#endif // PLUGIN_021_CORE

   #if NODO_MEGA
   case PLUGIN_MMI_IN:
     {
     char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);
     
     if(GetArgv(string,TempStr,1))
       {
       if(strcasecmp(TempStr,PLUGIN_NAME)==0)
         {
         if(event->Par1 >= 0 && event->Par1 <= PLUGIN_021_ROWS)
           {

           if(GetArgv(string,TempStr,4))
               event->Par2|=str2cmd(TempStr)<<8;

           if(GetArgv(string,TempStr,5))
               event->Par2|=str2int(TempStr)<<16;

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
     strcat(string,int2str(event->Par2 & 0xff));
     strcat(string,",");
     strcat(string,cmd2str(event->Par2>>8 & 0xff));
     strcat(string,",");
     strcat(string,int2str(event->Par2>>16 & 0xff));
     break;
     }
   #endif //NODO_MEGA
 }      
 return success;
}

#ifdef PLUGIN_021_CORE
/*********************************************************************/
void LCD_I2C_printline(byte row, byte col, char* message)
/*********************************************************************/
{
 LCD_I2C_setCursor(col,row);
 byte maxcol = PLUGIN_021_COLS-col;

 //clear line if empty message
 if (message[0]==0)
   for (byte x=0; x<PLUGIN_021_COLS; x++) LCD_I2C_write(' ');
 else
   for (byte x=0; x < maxcol; x++)
     {
       if (message[x] != 0) LCD_I2C_write(message[x]);
       else break;
     }
}

/*********************************************************************/
inline size_t LCD_I2C_write(uint8_t value)
/*********************************************************************/
{
LCD_I2C_send(value, Rs);
return 0;
}

/*********************************************************************/
void LCD_I2C_display() {
/*********************************************************************/
 _displaycontrol |= LCD_DISPLAYON;
 LCD_I2C_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/*********************************************************************/
void LCD_I2C_clear(){
/*********************************************************************/
 LCD_I2C_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
 delayMicroseconds(2000);  // this command takes a long time!
}

/*********************************************************************/
void LCD_I2C_home(){
/*********************************************************************/
 LCD_I2C_command(LCD_RETURNHOME);  // set cursor position to zero
 delayMicroseconds(2000);  // this command takes a long time!
}

/*********************************************************************/
void LCD_I2C_setCursor(uint8_t col, uint8_t row){
/*********************************************************************/
 int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
 if ( row > _numlines ) {
row = _numlines-1;    // we count rows starting w/0
 }
 LCD_I2C_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/*********************************************************************/
inline void LCD_I2C_command(uint8_t value) {
/*********************************************************************/
 LCD_I2C_send(value, 0);
}

/*********************************************************************/
void LCD_I2C_send(uint8_t value, uint8_t mode) {
/*********************************************************************/
 uint8_t highnib=value&0xf0;
 uint8_t lownib=(value<<4)&0xf0;
 LCD_I2C_write4bits((highnib)|mode);
 LCD_I2C_write4bits((lownib)|mode); 
}

/*********************************************************************/
void LCD_I2C_write4bits(uint8_t value) {
/*********************************************************************/
 LCD_I2C_expanderWrite(value);
 LCD_I2C_pulseEnable(value);
}

/*********************************************************************/
void LCD_I2C_expanderWrite(uint8_t _data){                                        
/*********************************************************************/
 WireNodo.beginTransmission(LCD_I2C_ADDRESS);
 WireNodo.write((int)(_data) | _backlightval);
 WireNodo.endTransmission(false);   
}

/*********************************************************************/
void LCD_I2C_pulseEnable(uint8_t _data){
/*********************************************************************/
 LCD_I2C_expanderWrite(_data | En);	// En high
 delayMicroseconds(1);	 // enable pulse must be >450ns

 LCD_I2C_expanderWrite(_data & ~En);	// En low
 delayMicroseconds(50);	 // commands need > 37us to settle
} 

// fix as of R555 (?)
#if NODO_MEGA
#else
/**********************************************************************************************\
* Deze functie haalt een tekst op uit PROGMEM en geeft als string terug
\*********************************************************************************************/
char* ProgmemString(prog_char* text)
{
 byte x=0;
 static char buffer[PLUGIN_021_COLS+5];

 do
 {
   buffer[x]=pgm_read_byte_near(text+x);
 }
 while(buffer[x++]!=0);
 return buffer;
}
#endif


#endif //PLUGIN_021_CORE