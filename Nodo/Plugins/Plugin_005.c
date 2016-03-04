//#######################################################################################################
//#################################### Plugin-05: TempSensor Dallas DS18B20  ############################
//#######################################################################################################


/*********************************************************************************************\
 * Deze funktie leest een Dallas temperatuursensor uit. 
 * Na uitgelezen van de waarde wordt in de opgegeven variabele de temperatuur opgeslagen. 
 * Na uitlezing wordt er een event gegenereerd.
 *
 * Auteur             : Paul Tonkes, p.k.tonkes@gmail.com
 * Support            : <website>
 * Datum              : Feb.2013
 * Versie             : 1.1
 * Nodo productnummer : ???
 * Compatibiliteit    : Vanaf Nodo build nummer 508
 * Syntax             : "TempRead <Par1:Poortnummer>, <Par2:Variabele>"
 *
 ***********************************************************************************************
 * Technische beschrijving:
 *
 * Compiled size      : ??? bytes voor een Mega en ??? voor een Small.
 * Externe funkties   : float2ul(), 
 *
 * De sensor kan volgens de paracitaire mode worden aangesloten. De signaallijn tevens verbinden met een 4K7 naar de Vcc/+5
 * Deze fucntie kan worden gebruikt voor alle Wired poorten van de Nodo.
 * Er wordt gebruik gemaakt van de ROM-skip techniek, dus er worden geen adressering gebruikt.
 * Dit betekent max. Ã©Ã©n sensor per poort. Dit om (veel) geheugen te besparen.  *
 \*********************************************************************************************/

uint8_t DallasPin;
#define PLUGIN_ID 05

#define PLUGIN_NAME "TempRead"

void DS_write(uint8_t ByteToWrite); 
uint8_t DS_read(void);
uint8_t DS_reset();

boolean Plugin_005(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  static byte Call_Status = 0x00; // Each bit represents one relative port. 0=not called before, 1=already called before. 
  
  switch(function)
    {
    #ifdef PLUGIN_005_CORE
      
    case PLUGIN_COMMAND:
      {
      int DSTemp;                           // Temperature in 16-bit Dallas format.
      byte ScratchPad[12];                  // Scratchpad buffer Dallas sensor.   
      byte var=event->Par2;                 // Variabele die gevuld moet worden.
      byte RelativePort=event->Par1-1;
      
      // De Dallas sensor kan worden aangesloten op iedere digitale poort van de Arduino. In ons geval kiezen we er voor
      // om de sensor aan te sluiten op de Wired-Out poorten van de Nodo. Met Par2 is de Wired poort aangegeven.
      // 1 = WiredOut poort 1.  
      DallasPin=PIN_WIRED_OUT_1+event->Par1-1;
      
      ClearEvent(event);                                      // Ga uit van een default schone event. Oude eventgegevens wissen.        
  
      noInterrupts();
      while (!(bitRead(Call_Status, RelativePort)))
        {
        // if this is the very first call to the sensor on this port, reset it to wake it up 
        boolean present=DS_reset();
        bitSet(Call_Status, RelativePort);
        }        
      boolean present=DS_reset();DS_write(0xCC /* rom skip */); DS_write(0x44 /* start conversion */);
      interrupts();
              
      if(present)
        {
        delay(800);     // uitleestijd die de sensor nodig heeft
    
        noInterrupts();
        DS_reset(); DS_write(0xCC /* rom skip */); DS_write(0xBE /* Read Scratchpad */);
    
        // Maak de lijn floating zodat de sensor de data op de lijn kan zetten.
        digitalWrite(DallasPin,LOW);
        pinMode(DallasPin,INPUT);
    
        for (byte i = 0; i < 9; i++)            // copy 8 bytes
          ScratchPad[i] = DS_read();
        interrupts();
      
        DSTemp = (ScratchPad[1] << 8) + ScratchPad[0];  
    
        event->Type         = NODO_TYPE_COMMAND;
        event->Port         = VALUE_SOURCE_PLUGIN;
        event->Command      = CMD_VARIABLE_SET;                 // Commando "VariableSet"
        event->Par1         = var;                              // Variabele die gevuld moet worden.
        event->Par2         = float2ul(float(DSTemp)*0.0625);   // DS18B20 variant. Waarde terugstoppen in de variabele
        success=true;
        }
      break;
      }
    #endif // CORE
      
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
          // Par1 en Par2 hoeven niet te worden geparsed omdat deze default al door de MMI invoer van de Nodo 
          // worden gevuld indien het integer waarden zijn. Toetsen op bereiken moet nog wel plaats vinden.
          if(event->Par1>0 && event->Par1<=WIRED_PORTS && event->Par2>0 && event->Par2<=USER_VARIABLES_MAX)            
            {
            success=true;
            event->Type = NODO_TYPE_PLUGIN_COMMAND;
            event->Command = 5; // Plugin nummer  
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
    #endif //MMI
    }      
  return success;
  }

#ifdef PLUGIN_005_CORE
uint8_t DS_read(void)
  {
  uint8_t bitMask;
  uint8_t r = 0;
  uint8_t BitRead;
  
  for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
    pinMode(DallasPin,OUTPUT);
    digitalWrite(DallasPin,LOW);
    delayMicroseconds(3);

    pinMode(DallasPin,INPUT);// let pin float, pull up will raise
    delayMicroseconds(10);
    BitRead = digitalRead(DallasPin);
    delayMicroseconds(53);

    if (BitRead)
      r |= bitMask;
    }
  return r;
  }

void DS_write(uint8_t ByteToWrite) 
  {
  uint8_t bitMask;

  pinMode(DallasPin,OUTPUT);
  for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {// BitWrite
    digitalWrite(DallasPin,LOW);
    if(((bitMask & ByteToWrite)?1:0) & 1)
      {
      delayMicroseconds(5);// Dallas spec.= 5..15 uSec.
      digitalWrite(DallasPin,HIGH);
      delayMicroseconds(55);// Dallas spec.= 60uSec.
      }
    else
      {
      delayMicroseconds(55);// Dallas spec.= 60uSec.
      digitalWrite(DallasPin,HIGH);
      delayMicroseconds(5);// Dallas spec.= 5..15 uSec.
      }
    }
  }

uint8_t DS_reset()
  {
  uint8_t r;
  uint8_t retries = 125;
  
  pinMode(DallasPin,INPUT);
  do  {  // wait until the wire is high... just in case
      if (--retries == 0) return 0;
      delayMicroseconds(2);
      } while ( !digitalRead(DallasPin));

  pinMode(DallasPin,OUTPUT); digitalWrite(DallasPin,LOW);
  delayMicroseconds(492); // Dallas spec. = Min. 480uSec. Arduino 500uSec.
  pinMode(DallasPin,INPUT);//Float
  delayMicroseconds(40);
  r = !digitalRead(DallasPin);// Dallas sensor houdt voor 60uSec. de bus laag nadat de Arduino de lijn heeft vrijgegeven.
  delayMicroseconds(420);
  return r;
  }
#endif //CORE
