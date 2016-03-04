//#######################################################################################################
//#################################### Plugin-036: PID-Regelaar        ##################################
//#######################################################################################################

/*********************************************************************************************\
* Auteur             : p.k.tonkes
* Support            : www.nodo-domotica.nl
* Datum              : 29-01-2015
* Versie             : 0.1 Zandbak versie 
* Nodo productnummer : Plugin-036 PID-Controller (SWACDE-Plugin36 PID-Controller-V0.1)
* Compatibiliteit    : Nodo versie 3.7 (Mega of Small)
* Syntax             : PID <parameter>, <waarde>
*
* Funktionele beschrijving: 
*  
* Deze pluging vormt een PID-regelaar zoals die vaak wordt gebruikt in de meet-&regeltechniek. Toepassingen 
* zijn onder andere:
* 
* - temperatuur regelingen (b.v. CV-Ketel, reservoirs met vloestof, etc.)
* - servobesturing;
* - Op koers houden van een zeeschip.
* - etc.
* 
* Met deze PID-regelaar is het mogelijk om fysieke processen te besturen zoals bijvoorbeeld het verwarmen van een ruimte, 
* volgen van een koers of ander fysiek proces waarbij gebruik wordt gemaakt van een ingestelde streefwaarde, meting en stuursignaal. 
* De PID-regelaar heeft een 'zelflerend' effect door meten van het effect van stuursignalen en de feitelijke verschijnselen zoals die 
* worden gemeten. De eigenschappen van de PID-regelaar kunnen worden aangepast door de parameters kP, kI en kD. Instellen van een 
* PID-regeling vergt wat werk, maar daar staat tegenover dat nauwkeurig geregeld kan worden. Voor documentatie over PID-regelingen 
* zie: http://nl.wikipedia.org/wiki/PID-regelaar
*
* Eigenschappen van deze PID-regelaar plugin:
* - plugin volledig draait volledig op de achtegrond;
* - Eenmaal per seconde een berekening van de output;
* - maakt gebruik van de opgegeven Nodo variabelen;
* - Analoge mode voor analoge stuurwaarde;
* - Digitale mode voor aan sturen relais waarbij de PID-berekening de dutycycle bepaald;
* - Monitor mode voor continue bekijken van alle PID-parameters;
* - Alle PID-parameters instelbaar tijdens uitvoer, dus geen hercompilatie nodig;
* - Instelbare onder- en bovengrens van de output.
* 
* Parameter:      Omschrijving: 
*---------------------------------------------------------------------------------------------
* kP              Proportionele actie van de regelaar (P-actie)
* kI              Integratietijd van de regelaar (I-actie)
* kD              Differentiatietijd van de regelaar (D-actie): hoe groter Td, hoe meer D-actie
* OutputMin       Minimale output van de regelaar (default 0)
* OutputMax       Maximale output van de regelaar (default 100))
* Monitor         Toggle monitor mode. Laat alle PID parameters zien tijdens uitvoer op Serial
* VarInput        Nodo Variabelenummer die wordt gebruikt als input van de PID regeling 
* VarOutput       Nodo Variabelenummer die wordt gebruikt als output van de PID regeling (in de digitale modus worden events gegenereerd)
* VarSetpoint     Nodo Variabelenummer die wordt gebruikt als setpoint van de PID regeling
* Manual          Handmatige mode: PID-regeling uitgeschakeld
* Analog          Analoge mode: PID-regelaar geeft analoge waarde als output
* Digital         Digitale mode: PID-regelaar bepaalt de dutycycle. OutputMax is de maximale window tijd in seconden.
* 
*********************************************************************************************/

#define VARIABLE_SETPOINT                        1                              // Nodo variabele die door deze plugin wordt gevuld met de stookmodulatie (0..100%)
#define VARIABLE_INPUT                           2                              // Nodo Variabelenummer waar de streef tempertuur in aangegeven moet worden. 
#define VARIABLE_OUTPUT                          3                              // Nodo variabelenummer die continue gevuld moet zijn met de actuele temperatuur.
 
#define PID_KP                                   1                              // Default waarde kP
#define PID_KI                                   0                              // Default waarde kI
#define PID_KD                                   0                              // Default waarde kD
#define PID_OUTPUT_MIN                           0
#define PID_OUTPUT_MAX                         100

#define PLUGIN_NAME                          "PID"
#define PLUGIN_ID                               36
#define PID_MODE_MANUAL                          0
#define PID_MODE_ANALOG                          1
#define PID_MODE_DIGITAL                         2

#define PLUGIN_036_TXT_KP                     "kP"
#define PLUGIN_036_TXT_KI                     "kI"
#define PLUGIN_036_TXT_KD                     "kD"
#define PLUGIN_036_TXT_OUTPUT_MIN      "OutputMin"
#define PLUGIN_036_TXT_OUTPUT_MAX      "OutputMax"
#define PLUGIN_036_TXT_MONITOR           "Monitor"
#define PLUGIN_036_TXT_VAR_INPUT        "VarInput"
#define PLUGIN_036_TXT_VAR_OUTPUT      "VarOutput"
#define PLUGIN_036_TXT_VAR_SETPOINT  "VarSetpoint"
#define PLUGIN_036_TXT_MANUAL             "Manual"
#define PLUGIN_036_TXT_ANALOG             "Analog"
#define PLUGIN_036_TXT_DIGITAL           "Digital"


#define PLUGIN_036_COMMAND                       1
#define PLUGIN_036_PAR_KP                        2
#define PLUGIN_036_PAR_KI                        3
#define PLUGIN_036_PAR_KD                        4
#define PLUGIN_036_PAR_OUTPUT_MIN                5
#define PLUGIN_036_PAR_OUTPUT_MAX                6
#define PLUGIN_036_PAR_MONITOR                   7
#define PLUGIN_036_PAR_VAR_INPUT                 8
#define PLUGIN_036_PAR_VAR_OUTPUT               10
#define PLUGIN_036_PAR_VAR_SETPOINT             11
#define PLUGIN_036_PAR_MANUAL                   12
#define PLUGIN_036_PAR_ANALOG                   13
#define PLUGIN_036_PAR_DIGITAL                  14


float PID_Input, PID_Output, PID_Setpoint;
float lastInput=0;
float timeChange, error=0, errSum=0, dErr=0, lastErr=0;
float PID_kP=PID_KP, PID_kI=PID_KI, PID_kD=PID_KD;
float PID_outMin=PID_OUTPUT_MIN, PID_outMax=PID_OUTPUT_MAX, TempFloat;
unsigned long WindowCounter=1, WindowMax=0, PID_LastTime=0L, Now=0, PID_Now;
boolean PID_Mode=PID_MODE_ANALOG;
boolean Monitor=false, DigitalOutput=false,DigitalOutputPrev=false; 
byte VarInput=VARIABLE_INPUT, VarOutput=VARIABLE_OUTPUT,VarSetpoint=VARIABLE_SETPOINT;


boolean Plugin_036(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  int x,y;
   
  switch(function)
    {    
    #ifdef PLUGIN_036_CORE

    case PLUGIN_INIT:
      {
      break;
      }

    case PLUGIN_ONCE_A_SECOND:
      {
      PID_Setpoint=UserVar[VarSetpoint-1];
      PID_Input=UserVar[VarInput-1];

      if(PID_Mode!=PID_MODE_MANUAL)
        {
        PID_Now = millis();
        timeChange = (double)(PID_Now - PID_LastTime);
       
        error = PID_Setpoint - PID_Input;
        errSum += (error * timeChange);
        dErr = (error - lastErr) / timeChange;
     
        PID_Output = PID_kP*error + PID_kI*errSum + PID_kD*dErr;

        if(PID_Output > PID_outMax) PID_Output = PID_outMax;
        else if(PID_Output < PID_outMin) PID_Output = PID_outMin;

        lastErr = error;
        PID_LastTime =PID_Now;        
        }

      switch(PID_Mode)
        {
        case PID_MODE_MANUAL:
          break;
          
        case PID_MODE_ANALOG:
          UserVar[VarOutput-1]=PID_Output;
          break;
          
        case PID_MODE_DIGITAL:

          if(WindowCounter<=PID_Output)
            DigitalOutput=true;
          else
            DigitalOutput=false;
          
          UserVar[VarOutput-1]=(float)DigitalOutput;

          if(DigitalOutput!=DigitalOutputPrev)
            {
            event->SourceUnit   = Settings.Unit;
            event->Type         = NODO_TYPE_EVENT;
            event->Command      = EVENT_VARIABLE;
            event->Port         = VALUE_SOURCE_PLUGIN;
            event->Direction    = VALUE_DIRECTION_INPUT;
            event->Par1         = VarOutput-1;
            event->Par2         = float2ul(DigitalOutput);
            ProcessEvent(event);
            DigitalOutputPrev=DigitalOutput;
            }

          if(++WindowCounter>WindowMax)
            {
            WindowMax=PID_outMax;
            WindowCounter=1;
            }

          break;
        }

      if(Monitor)
        {
        Serial.print("PID: VarSetpoint=");Serial.print(VarSetpoint);
        Serial.print(", VarInput=");Serial.print(VarInput);
        Serial.print(", VarOutput=");Serial.print(VarOutput);
        Serial.print(", Mode=");
        switch(PID_Mode)
          {
          case PID_MODE_MANUAL: Serial.print("Manual"); break;
          case PID_MODE_ANALOG: Serial.print("Analog"); break;
          case PID_MODE_DIGITAL: Serial.print("Digital"); break;
          }
        Serial.print(", kp=");Serial.print(PID_kP);
        Serial.print(", ki=");Serial.print(PID_kI);
        Serial.print(", kd=");Serial.print(PID_kD);
        Serial.print(", OutputMin=");Serial.print(PID_outMin);
        Serial.print(", OutputMax=");Serial.print(PID_outMax);
        Serial.print(", Setpoint=");Serial.print(PID_Setpoint);
        Serial.print(", Input=");Serial.print(PID_Input);
        Serial.print(", Output=");Serial.print(PID_Output);
        Serial.print(" (");Serial.print(((long)PID_Output)*100/PID_outMax);Serial.print("%)");

        if(PID_Mode==PID_MODE_DIGITAL)
          {
          Serial.print(", State=");Serial.print(DigitalOutput);
          }
        Serial.println();
        }
      break;
      }

    case PLUGIN_COMMAND:
      {
      TempFloat=ul2float(event->Par2);
      success=true;
      
      switch(event->Par1)
        {
        case PLUGIN_036_PAR_KP:
          PID_kP=TempFloat;
          break;          

        case PLUGIN_036_PAR_KI:
          PID_kI=TempFloat;
          break;          

        case PLUGIN_036_PAR_KD:
          PID_kD=TempFloat;
          break;          

        case PLUGIN_036_PAR_OUTPUT_MIN:
          PID_outMin=TempFloat;
          break;  

        case PLUGIN_036_PAR_OUTPUT_MAX:
          PID_outMax=TempFloat;
          WindowMax=PID_outMax;
          break; 

        case PLUGIN_036_PAR_MONITOR:
          Monitor=!Monitor;
          break;       

        case PLUGIN_036_PAR_VAR_INPUT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarInput=TempFloat;
          break;   

        case PLUGIN_036_PAR_VAR_OUTPUT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarOutput=TempFloat;
          break;  

        case PLUGIN_036_PAR_VAR_SETPOINT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarSetpoint=TempFloat;
          break;

        case PLUGIN_036_PAR_MANUAL:
          PID_Mode=PID_MODE_MANUAL;
          break;  

        case PLUGIN_036_PAR_ANALOG:
          PID_Mode=PID_MODE_ANALOG;
          break;  

        case PLUGIN_036_PAR_DIGITAL:
          PID_Mode=PID_MODE_DIGITAL;
          break;
        }
      break;
      }
    #endif // CORE
    
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);
      success=false;

      if(GetArgv(string,TempStr,1))
        {
        event->Type=0;
        
        // Command: PID <Parameter>,<Waarde>
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)                       // Zoek naar commando
          event->Type = NODO_TYPE_PLUGIN_COMMAND;

        if(event->Type)                                                         // Als commando gevonden
          {
          event->Command = PLUGIN_ID;                                           // Plugin nummer  
          event->Par1 = 0;
          event->Par2 = 0;
          success=true;
                    
          if(StringFind(string,PLUGIN_036_TXT_KP)!=-1)
            event->Par1=PLUGIN_036_PAR_KP;

          else if(StringFind(string,PLUGIN_036_TXT_KI)!=-1)
            event->Par1=PLUGIN_036_PAR_KI;
  
          else if(StringFind(string,PLUGIN_036_TXT_KD)!=-1)
            event->Par1=PLUGIN_036_PAR_KD;
  
          else if(StringFind(string,PLUGIN_036_TXT_OUTPUT_MIN)!=-1)
            event->Par1=PLUGIN_036_PAR_OUTPUT_MIN;
  
          else if(StringFind(string,PLUGIN_036_TXT_OUTPUT_MAX)!=-1)
            event->Par1=PLUGIN_036_PAR_OUTPUT_MAX;
  
          else if(StringFind(string,PLUGIN_036_TXT_MONITOR)!=-1)
            event->Par1=PLUGIN_036_PAR_MONITOR;
  
          else if(StringFind(string,PLUGIN_036_TXT_MANUAL)!=-1)
            event->Par1=PLUGIN_036_PAR_MANUAL;
  
          else if(StringFind(string,PLUGIN_036_TXT_ANALOG)!=-1)
            event->Par1=PLUGIN_036_PAR_ANALOG;
  
          else if(StringFind(string,PLUGIN_036_TXT_DIGITAL)!=-1)
            event->Par1=PLUGIN_036_PAR_DIGITAL;
            
          else if(StringFind(string,PLUGIN_036_TXT_VAR_SETPOINT)!=-1)
            event->Par1=PLUGIN_036_PAR_VAR_SETPOINT;
            
          else if(StringFind(string,PLUGIN_036_TXT_VAR_OUTPUT)!=-1)
            event->Par1=PLUGIN_036_PAR_VAR_OUTPUT;
            
          else if(StringFind(string,PLUGIN_036_TXT_VAR_INPUT)!=-1)
            event->Par1=PLUGIN_036_PAR_VAR_INPUT;
            
          else           
            success=false;
            
          if(event->Par1)  
            {
            if(GetArgv(string,TempStr,3))                                       // volgende deel van de string met de invoerwaarde op. 
              {
              event->Par2=float2ul(atof(TempStr));
              }
            }
          }
        }
      free(TempStr);

      break;
      }
      
    case PLUGIN_MMI_OUT:
      {
      x=false;
      strcpy(string,PLUGIN_NAME);
      strcat(string," ");

      switch(event->Par1)
        {
        case PLUGIN_036_PAR_KP:  
          strcat(string,PLUGIN_036_TXT_KP);
          x=true;
          break;
        case PLUGIN_036_PAR_KI:  
          strcat(string,PLUGIN_036_TXT_KI);
          x=true;
          break;
        case PLUGIN_036_PAR_KD:  
          strcat(string,PLUGIN_036_TXT_KD);
          x=true;
          break;
        case PLUGIN_036_PAR_OUTPUT_MIN:  
          strcat(string,PLUGIN_036_TXT_OUTPUT_MIN);
          x=true;
          break;
        case PLUGIN_036_PAR_OUTPUT_MAX:  
          strcat(string,PLUGIN_036_TXT_OUTPUT_MAX);
          x=true;
          break;
        case PLUGIN_036_PAR_MONITOR:  
          strcat(string,PLUGIN_036_TXT_MONITOR);
          break;
        case PLUGIN_036_PAR_VAR_INPUT:  
          strcat(string,PLUGIN_036_TXT_VAR_INPUT);
          x=true;
          break;
        case PLUGIN_036_PAR_VAR_OUTPUT:  
          strcat(string,PLUGIN_036_TXT_VAR_OUTPUT);
          x=true;
          break;
        case PLUGIN_036_PAR_VAR_SETPOINT:  
          strcat(string,PLUGIN_036_TXT_VAR_SETPOINT);
          x=true;
          break;
        case PLUGIN_036_PAR_MANUAL:  
          strcat(string,PLUGIN_036_TXT_MANUAL);
          break;
        case PLUGIN_036_PAR_ANALOG:  
          strcat(string,PLUGIN_036_TXT_ANALOG);
          break;
        case PLUGIN_036_PAR_DIGITAL:  
          strcat(string,PLUGIN_036_TXT_DIGITAL);
          break;
        }  

      if(x)
        {
        strcat(string,",");
        dtostrf(ul2float(event->Par2), 0, 3, strlen(string)+string);
        }
      }
    #endif
    }
  return success;
  }

