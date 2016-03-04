//#######################################################################################################
//#################################### Plugin-027: Servo ################################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor aansturing van een servo
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 10 Sep 2013
 * Versie             : 1.0
 * Nodo productnummer : 
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "Servo <Par1:Poort>, <Par2:Servo Position [0-180]>"
 *********************************************************************************************
 * Technische beschrijving:
 *
 * Compiled size      : 1112 bytes voor een Mega en 1112 voor een Small.
 * Externe funkties   : <geef hier aan welke funkties worden gebruikt.
 * Met behulp van dit device is het mogelijk om via de Nodo maximaal 4 servo's aan te sturen
 *   om b.v. zelf een pan en tilt camera te maken
 * Servo's kunnen worden aangesloten op de WiredOut poort 1 t/m 4
 \*********************************************************************************************/

// prototypes
void Servo_Init(byte servoIndex);
uint8_t Servo_attach(byte servoIndex, int pin);
uint8_t Servo_attach2(byte servoIndex, int pin, int min, int max);
void Servo_write(byte servoIndex, int value);
void Servo_writeMicroseconds(byte servoIndex, int value);
bool Servo_attached(byte servoIndex);


#define PLUGIN_NAME_027 "Servo"
boolean Plugin_027(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  #ifdef PLUGIN_027_CORE
  switch(function)
    {
    case PLUGIN_COMMAND:
      {
        if (!Servo_attached(event->Par1-1))
          {
            Servo_Init(event->Par1-1);
            Servo_attach(event->Par1-1,PIN_WIRED_OUT_1 + event->Par1-1);
          }
        Servo_write(event->Par1-1,event->Par2);
        success=true;
        break;
      }
      
    #endif // CORE
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      if(GetArgv(string,TempStr,1))
        {
        if(strcasecmp(TempStr,PLUGIN_NAME_027)==0)
          {
          if(GetArgv(string,TempStr,2)) 
            {
            if(GetArgv(string,TempStr,3))
              {
              if(event->Par1>0 && event->Par1<5 && event->Par2>=0 && event->Par2<=180)            
                {
                  event->Type = NODO_TYPE_PLUGIN_COMMAND;
                  event->Command = 27; // Plugin nummer
                  success=true;
                }
              }
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      strcpy(string,PLUGIN_NAME_027);
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


#ifdef PLUGIN_027_CORE

#define MAX_SERVOS   	           4
#define SERVOS_PER_TIMER         4      // This code only supports 1 timer!

#define SERVO_MIN_PULSE_WIDTH  544     // the shortest pulse sent to a servo  
#define SERVO_MAX_PULSE_WIDTH 2400     // the longest pulse sent to a servo 
#define SERVO_DEFAULT_PULSE_WIDTH   1500     // default pulse width when servo is attached
#define SERVO_REFRESH_INTERVAL     20000     // minumim time to refresh servos in microseconds 
#define SERVO_TRIM_DURATION            2     // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

// convenience macros
#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds
#define SERVO_MIN() (SERVO_MIN_PULSE_WIDTH - min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (SERVO_MAX_PULSE_WIDTH - max * 4)  // maximum value in uS for this servo 

typedef struct  {
  uint8_t nbr        :6 ;             // a pin number from 0 to 63
  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false 
} ServoPin_t   ;  

typedef struct {
  ServoPin_t Pin;
  unsigned int ticks;
} servo_t;


int8_t min;                           // minimum is this value times 4 added to SERVO_MIN_PULSE_WIDTH    
int8_t max;                           // maximum is this value times 4 added to SERVO_MAX_PULSE_WIDTH   
static servo_t servos[MAX_SERVOS];    // static array of servo structures
static volatile int8_t Channel;       // counter for the servo being pulsed for each timer (or -1 if refresh interval)
uint8_t ServoCount = 0;               // the total number of attached servos

static inline void handle_interrupts(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  if( Channel < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
  else{
    if( Channel < ServoCount && servos[Channel].Pin.isActive == true )  
      digitalWrite( servos[Channel].Pin.nbr,LOW); // pulse this channel low if activated   
  }

  Channel++;    // increment to the next channel
  if( Channel < ServoCount && Channel < SERVOS_PER_TIMER) {
    *OCRnA = *TCNTn + servos[Channel].ticks;
    if(servos[Channel].Pin.isActive == true)     // check if activated
      digitalWrite( servos[Channel].Pin.nbr,HIGH); // its an active channel so pulse it high   
  }  
  else { 
    // finished all channels so wait for the refresh period to expire before starting over 
    if( ((unsigned)*TCNTn) + 4 < usToTicks(SERVO_REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)usToTicks(SERVO_REFRESH_INTERVAL);  
    else 
      *OCRnA = *TCNTn + 4;  // at least SERVO_REFRESH_INTERVAL has elapsed
    Channel = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}


#if NODO_MEGA
SIGNAL (TIMER5_COMPA_vect) 
{
  handle_interrupts(&TCNT5, &OCR5A); 
}
#else
SIGNAL (TIMER1_COMPA_vect) 
{ 
  handle_interrupts(&TCNT1, &OCR1A); 
}
#endif


static void initISR()
{  
#if NODO_MEGA
    TCCR5A = 0;             // normal counting mode 
    TCCR5B = _BV(CS51);     // set prescaler of 8  
    TCNT5 = 0;              // clear the timer count 
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts; 
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt      
#else
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1 = 0;              // clear the timer count 
    TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt 
#endif
} 


static boolean isTimerActive()
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(servos[channel].Pin.isActive == true)
      return true;
  }
  return false;
}

void Servo_Init(byte servoIndex)
{
  if( ServoCount < MAX_SERVOS) {
    ServoCount++;
    servos[servoIndex].ticks = usToTicks(SERVO_DEFAULT_PULSE_WIDTH);   // store default values
  }
}

uint8_t Servo_attach(byte servoIndex, int pin)
{
  return Servo_attach2(servoIndex, pin, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
}

uint8_t Servo_attach2(byte servoIndex,int pin, int min, int max)
{
  if(servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    servos[servoIndex].Pin.nbr = pin;  
// todo dit moet per channel worden gedaan, nog uitwerken
    min  = (SERVO_MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    max  = (SERVO_MAX_PULSE_WIDTH - max)/4; 
    if(isTimerActive() == false)
      initISR();    
    servos[servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  } 
  return servoIndex ;
}

void Servo_write(byte servoIndex, int value)
{  
  if(value < SERVO_MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());      
  }
  Servo_writeMicroseconds(servoIndex, value);
}

void Servo_writeMicroseconds(byte servoIndex, int value)
{
  if( (servoIndex < MAX_SERVOS) )   // ensure channel is valid
  {  
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();   
    
  	value = value - SERVO_TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    uint8_t oldSREG = SREG;
    cli();
    servos[servoIndex].ticks = value;  
    SREG = oldSREG;   
  } 
}

bool Servo_attached(byte servoIndex)
{
  return servos[servoIndex].Pin.isActive ;
}
#endif
