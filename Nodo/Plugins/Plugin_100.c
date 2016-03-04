//#######################################################################################################
//#################################### Plugin-100: PID-Regelaar        ##################################
//#######################################################################################################

/*********************************************************************************************\
* Funktionele beschrijving: 
*  
* Deze pluging vormt een PID-regelaar zoals die vaak wordt gebruikt in de meet-&regeltechniek. Met deze PID-regelaar
* is het mogelijk om fysieke processen te besturen zoals bijvoorbeeld het verwarmen van een ruimte, volgen van een koers of
* ander fysiek proces waarbij gebruik wordt gemaakt van een ingestelde streefwaarde, meting en stuursignaal. De PID-regelaar
* heeft een 'zelflerend' effect door meten van het effect van stuursignalen en de feitelijke verschijnselen zoals die worden gemeten.
* De eigenschappen van de PID-regelaar kunnen worden aangepast door de parameters kP, kI en kD. Voor documentatie over PID-regelingen 
* zie: http://nl.wikipedia.org/wiki/PID-regelaar
* 
* Na het zetten van parameters die belangrijk zijn voor het gedrag van de PID-regeling, zal deze plugin volledig
* op de achtegrond draaien gebruik maken van de opgegeven Nodo variabelen.
* 
* ??? Nader toelichten hoe de plugin werkt.
* 
* ??? uitwerken van een praktische casus: een CV-thermostaat met aan/uit regeling op basis van PID-regeling.
* 
* Auteur             : p.k.tonkes
* Support            : www.nodo-domotica.nl
* Datum              : 29-01-2015
* Versie             : 0.1 Zandbak versie 
* Nodo productnummer : <????>
* Compatibiliteit    : Nodo versie 3.7 (Mega of Small)
* Syntax             : 
*
* 
* 
*********************************************************************************************/

#define VARIABLE_SETPOINT                        1                              // Nodo variabele die door deze plugin wordt gevuld met de stookmodulatie (0..100%)
#define VARIABLE_INPUT                           2                              // Nodo Variabelenummer waar de streef tempertuur in aangegeven moet worden. 
#define VARIABLE_OUTPUT                          3                              // Nodo variabelenummer die continue gevuld moet zijn met de actuele temperatuur.
 
#define PID_KP                                   1                              // Default waarde kP
#define PID_KI                                   0                              // Default waarde kI
#define PID_KD                                   0                              // Default waarde kD

#define PLUGIN_NAME                          "PID"
#define PLUGIN_ID                              100
#define PID_MODE_MANUAL                          0
#define PID_MODE_ANALOG                          1
#define PID_MODE_DIGITAL                         2

#define PLUGIN_100_TXT_COMMAND               "PID"
#define PLUGIN_100_TXT_KP                     "kP"
#define PLUGIN_100_TXT_KI                     "kI"
#define PLUGIN_100_TXT_KD                     "kD"
#define PLUGIN_100_TXT_OUTPUT_MIN      "OutputMin"
#define PLUGIN_100_TXT_OUTPUT_MAX      "OutputMax"
#define PLUGIN_100_TXT_DEBUG               "Debug"
#define PLUGIN_100_TXT_VAR_INPUT        "VarInput"
#define PLUGIN_100_TXT_VAR_OUTPUT      "VarOutput"
#define PLUGIN_100_TXT_VAR_SETPOINT  "VarSetpoint"
#define PLUGIN_100_TXT_MANUAL             "Manual"
#define PLUGIN_100_TXT_ANALOG             "Analog"
#define PLUGIN_100_TXT_DIGITAL           "Digital"
#define PLUGIN_100_TXT_TEST                 "Test"


#define PLUGIN_100_COMMAND                       1
#define PLUGIN_100_PAR_KP                        2
#define PLUGIN_100_PAR_KI                        3
#define PLUGIN_100_PAR_KD                        4
#define PLUGIN_100_PAR_OUTPUT_MIN                5
#define PLUGIN_100_PAR_OUTPUT_MAX                6
#define PLUGIN_100_PAR_DEBUG                     7
#define PLUGIN_100_PAR_VAR_INPUT                 8
#define PLUGIN_100_PAR_VAR_OUTPUT               10
#define PLUGIN_100_PAR_VAR_SETPOINT             11
#define PLUGIN_100_PAR_MANUAL                   12
#define PLUGIN_100_PAR_ANALOG                   13
#define PLUGIN_100_PAR_DIGITAL                  14
#define PLUGIN_100_PAR_TEST                     15

/*working variables*/
unsigned long WindowStart,WindowMax, PID_LastTime;
float PID_Input, PID_Output, PID_Setpoint;
float ITerm, lastInput;
float errSum, lastErr;
float PID_kP=PID_KP, PID_kI=PID_KI, PID_kD=PID_KD;
float PID_outMin, PID_outMax, TempFloat;
boolean PID_Mode=PID_MODE_MANUAL;
boolean Debug=false, DigitalOutput=false,DigitalOutputPrev=false; 
byte VarInput=VARIABLE_INPUT, VarOutput=VARIABLE_OUTPUT,VarSetpoint=VARIABLE_SETPOINT;

boolean Plugin_100(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  int x,y;
   
  switch(function)
    {    
    #ifdef PLUGIN_100_CORE

    case PLUGIN_INIT:
      {
      break;
      }

    case PLUGIN_EVENT_IN:
      {
      break;      
      }

    case PLUGIN_ONCE_A_SECOND:
      {
      PID_Setpoint=UserVar[VarSetpoint-1];
      PID_Input=UserVar[VarInput-1];
      

      if(PID_Mode!=PID_MODE_MANUAL)
        {
        /*How long since we last calculated*/
        unsigned long PID_Now = millis();
        double timeChange = (double)(PID_Now - PID_LastTime);
       
        /*Compute all the working error variables*/
        double error = PID_Setpoint - PID_Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr) / timeChange;
       
        /*Compute PID Output*/
        PID_Output = PID_kP * error + PID_kI * errSum + PID_kD * dErr;
       
        /*Remember some variables for next time*/
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
          if(WindowStart < (millis()-(unsigned long)PID_outMax))
            WindowStart=millis();
          
          if( (WindowStart+(unsigned long)PID_Output)>=millis())
            DigitalOutput=true;
          else
            DigitalOutput=false;
          
          UserVar[VarOutput-1]=(float)DigitalOutput;

          if(DigitalOutput!=DigitalOutputPrev)
            {
            Serial.println("*** Digital: Event!!!");
/*            event->SourceUnit   = Settings.Unit;
            event->Type         = NODO_TYPE_EVENT;
            event->Command      = EVENT_VARIABLE;
            event->Port         = VALUE_SOURCE_PLUGIN;
            event->Direction    = VALUE_DIRECTION_INPUT;
            event->Par1         = VarOutput-1;
//            event->Par2         = float2ul(UserVar[VarOutput-1]);
            //ProcessEvent(event);      // verwerk  event.

*/
            }

          DigitalOutputPrev=DigitalOutput;
          break;
        }

      if(Debug)
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
        Serial.print(", Output=");Serial.print(UserVar[VARIABLE_OUTPUT-1]);

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
        case PLUGIN_100_PAR_KP:
          PID_kP=TempFloat;
          break;          

        case PLUGIN_100_PAR_KI:
          PID_kI=TempFloat;
          break;          

        case PLUGIN_100_PAR_KD:
          PID_kD=TempFloat;
          break;          

        case PLUGIN_100_PAR_OUTPUT_MIN:
          PID_outMin=TempFloat;
          break;  

        case PLUGIN_100_PAR_OUTPUT_MAX:
          PID_outMax=TempFloat;
          break; 

        case PLUGIN_100_PAR_DEBUG:
          Debug=!Debug;
          break;       

        case PLUGIN_100_PAR_VAR_INPUT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarInput=TempFloat;
          break;   

        case PLUGIN_100_PAR_VAR_OUTPUT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarOutput=TempFloat;
          break;  

        case PLUGIN_100_PAR_VAR_SETPOINT:
          if(TempFloat>=1 && TempFloat<=USER_VARIABLES_MAX)VarSetpoint=TempFloat;
          break;

        case PLUGIN_100_PAR_MANUAL:
          PID_Mode=PID_MODE_MANUAL;
          break;  

        case PLUGIN_100_PAR_ANALOG:
          PID_Mode=PID_MODE_ANALOG;
          break;  

        case PLUGIN_100_PAR_DIGITAL:
          PID_Mode=PID_MODE_DIGITAL;
          break;
        
        case PLUGIN_100_PAR_TEST:
          DigitalOutputPrev=!DigitalOutputPrev;
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
        if(strcasecmp(TempStr,PLUGIN_100_TXT_COMMAND)==0)                       // Zoek naar commando
          event->Type = NODO_TYPE_PLUGIN_COMMAND;

        if(event->Type)                                                         // Als commando gevonden
          {
          event->Command = PLUGIN_ID;                                           // Plugin nummer  
          event->Par1 = 0;
          event->Par2 = 0;
          success=true;
                    
          if(StringFind(string,PLUGIN_100_TXT_KP)!=-1)
            event->Par1=PLUGIN_100_PAR_KP;

          else if(StringFind(string,PLUGIN_100_TXT_KI)!=-1)
            event->Par1=PLUGIN_100_PAR_KI;
  
          else if(StringFind(string,PLUGIN_100_TXT_KD)!=-1)
            event->Par1=PLUGIN_100_PAR_KD;
  
          else if(StringFind(string,PLUGIN_100_TXT_OUTPUT_MIN)!=-1)
            event->Par1=PLUGIN_100_PAR_OUTPUT_MIN;
  
          else if(StringFind(string,PLUGIN_100_TXT_OUTPUT_MAX)!=-1)
            event->Par1=PLUGIN_100_PAR_OUTPUT_MAX;
  
          else if(StringFind(string,PLUGIN_100_TXT_DEBUG)!=-1)
            event->Par1=PLUGIN_100_PAR_DEBUG;
  
          else if(StringFind(string,PLUGIN_100_TXT_MANUAL)!=-1)
            event->Par1=PLUGIN_100_PAR_MANUAL;
  
          else if(StringFind(string,PLUGIN_100_TXT_ANALOG)!=-1)
            event->Par1=PLUGIN_100_PAR_ANALOG;
  
          else if(StringFind(string,PLUGIN_100_TXT_DIGITAL)!=-1)
            event->Par1=PLUGIN_100_PAR_DIGITAL;
            
          else if(StringFind(string,PLUGIN_100_TXT_VAR_SETPOINT)!=-1)
            event->Par1=PLUGIN_100_PAR_VAR_SETPOINT;
            
          else if(StringFind(string,PLUGIN_100_TXT_VAR_OUTPUT)!=-1)
            event->Par1=PLUGIN_100_PAR_VAR_OUTPUT;
            
          else if(StringFind(string,PLUGIN_100_TXT_VAR_INPUT)!=-1)
            event->Par1=PLUGIN_100_PAR_VAR_INPUT;
            
          else if(StringFind(string,PLUGIN_100_TXT_TEST)!=-1)
            event->Par1=PLUGIN_100_PAR_TEST;
            
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
  #endif
    }
  return success;
  }


