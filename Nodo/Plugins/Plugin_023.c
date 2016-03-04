//#######################################################################################################
//#################################### Plugin-23: PWM Led dimmer ########################################
//#######################################################################################################

/*********************************************************************************************\
 * Funktionele beschrijving: Dit device stuurt een PWM signaal uit waarmee een RGB-led kan worden aangestuurd.
 *                           Als de Arduino uitgang wordt gebruikt om een MOSFET of een andere schakeling
 *                           aan te sturen, dan kunnen hiermee led-strips worden aangestuurd.
 *
 *                           
 * Auteur             : P.K.Tonkes
 * Support            : P.K.Tonkes@gmail.com
 * Versie             : 21-02-2015, Versie 1,4, P.K.Tonkes: Bug verwijderd: Issue 864:	Fade proces van plugin 23 wordt afgebroken
 *                    : 02-02-2014, Versie 1,2, P.K.Tonkes: Bug verwijderd: Leds schakelden soms ook op andere events. FadeOn,FadeOff,VarOn,Varoff toegevoegd
 *                    : 22-12-2013, Versie 1,1, P.K.Tonkes: Event en Send commando in 1 plugin ondergebracht.
 *                      26-05-2013, Versie 1.0, P.K.Tonkes: Beta versie 
 *
 * Nodo productnummer : Plugin-23 PWM Led-dimmer (SWACDE-23-V14)
 * Compatibiliteit    : Vanaf Nodo build nummer 645
 * Syntax             : RGBLed <Red>,<Green>,<Blue>,<FadeOn|FadeOff|VarOn|VarrOff> ==> Stuurt de RGB-led aan
 *                      RGBLedSend <NodoUnit>,<Red>,<Green>,<Blue>,<FadeOn|FadeOff|VarOn|VarrOff> ==> Verzendt event en stuurt
 *                      de RGB-led van een andere Nodo aan
 *                      
 *                      FadeOn laat de kleuren langzaam in elkaar overvloeien, FadeOff schakelt deze optie weer uit.
 *                      VarOn laat de plugin de waarden overnemen van de variabelen 1,2 en 3. VarOff schakelt dit weer uit.
 *                      Als de optie VarOn en FadeOn zijn ingeschakeld, dan is Variabele-4 de fade tijd.    
 * 
 * LET OP: Een van de RGB-kleuren maakt gebruik van de IR_TX_DATA pen. Het continue
 *         aansturen van de IR-Leds zou de Leds/transistor kunnen overbelasten.   
 *         Als je deze plugin wilt gebruiken voor aansturing een RGB-LED en je hebt GEEN infrarood-LED's 
 *         aangesloten, dan in je config file de volgende regel opnemen:
 *         
 *         #define PLUGIN_023_CORE_RGBLED 
 *          
 *         Zonder deze regel zal de plugin geen LED's aansturen en is alleen RGBLedSend beschikbaar.
 *         Dit is uit veiligheid om te voorkomen dat de Nodo beschadigd raakt omdat default Nodo's op deze 
 *         poort de IR zender aangesloten heeft. 
 * 
 * 
 ***********************************************************************************************
 * Technische beschrijving:
 *
 * De dimfunktie werkt op basis van Pulsbreedte modulatie (PWM) Dit is een standaard methode voor 
 * dimmen van LED's, mmaar wrdt ook gebruikt voor regelen van snelheid van motoren. 
 * Omdat de PWM funkties niet op alle pinnen van de Arduino beschikbaar zijn,
 * zijn deze in dit device hard toegewezen is de pinkeuze geen gebruikers optie.
 *
 * Nodo Mega:  Rood  = Arduino pen PWM-5
 *             Groen = Arduino pen PWM-6
 *             Blauw = Arduino pen PWM-7
 *
 * Nodo Small: Rood  = Arduino pen D9
 *             Groen = Arduino pen D10
 *             Blauw = Arduino pen D11 (LET OP: Op een standaard Nodo is dit de IR zend-led. Deze verwijderen!!!)
 *             
 \*********************************************************************************************/
 
#define PLUGIN_ID 23
#define PLUGIN_23_COMMAND "RGBLedSend"
#define PLUGIN_23_EVENT   "RGBLed"
#define PLUGIN_23_FADEON  "FadeOn"
#define PLUGIN_23_FADEOFF "FadeOff"
#define PLUGIN_23_VARON   "VarOn"
#define PLUGIN_23_VAROFF  "VarOff"

#if NODO_MEGA
#define PWM_R          5
#define PWM_G          6
#define PWM_B          7
#else
#define PWM_R          9
#define PWM_G         10
#define PWM_B         11
#endif

#ifdef PLUGIN_023_CORE
byte InputLevelR=0, InputLevelG=0, InputLevelB=0;
byte OutputLevelR=0, OutputLevelG=0, OutputLevelB=0;
int CalledCounter=0, FadeTimeCounter=0;
boolean RGBFade=false;
boolean RGBVariables=false;


#define RGBLED_MODE_SETRGB         1
#define RGBLED_MODE_VARIABLES_ON   2
#define RGBLED_MODE_VARIABLES_OFF  4
#define RGBLED_MODE_FADE_ON        8
#define RGBLED_MODE_FADE_OFF      16

int StringFind(char *string, char *keyword);


boolean FadeLed(void)
  {
  if(InputLevelR>OutputLevelR)OutputLevelR++;
  if(InputLevelG>OutputLevelG)OutputLevelG++;
  if(InputLevelB>OutputLevelB)OutputLevelB++;

  if(InputLevelR<OutputLevelR)OutputLevelR--;
  if(InputLevelG<OutputLevelG)OutputLevelG--;
  if(InputLevelB<OutputLevelB)OutputLevelB--;

  analogWrite(PWM_R,OutputLevelR);
  analogWrite(PWM_G,OutputLevelG);
  analogWrite(PWM_B,OutputLevelB);

  if(RGBVariables)
    {
    UserVar[0]=OutputLevelR;
    UserVar[1]=OutputLevelG;
    UserVar[2]=OutputLevelB;
    }
    
  if(InputLevelR==OutputLevelR && InputLevelG==OutputLevelG && InputLevelB==OutputLevelB)
    return false;                                                               // klaar met faden naar nieuwe waarde
  else
    return true;
  }
  
void Plugin_023_FLC(void)
  {  
  if(FadeTimeCounter==CalledCounter++)
    {
    CalledCounter=0;
    if(!FadeLed())
      FastLoopCall_ptr==0;
    }
  }
  
#endif  

boolean Plugin_023(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false, Set=false;
  byte x,y;
  struct NodoEventStruct TempEvent;  
  
  switch(function)
    { 
    #ifdef PLUGIN_023_CORE_RGBLED

    case PLUGIN_INIT:
      {
      analogWrite(PWM_R,255);
      analogWrite(PWM_G,255);
      analogWrite(PWM_B,255);
      delay(100);
      analogWrite(PWM_R,0);
      analogWrite(PWM_G,0);
      analogWrite(PWM_B,0);

      break;
      }

    case PLUGIN_EVENT_IN:
      {
      if(event->Command==EVENT_VARIABLE && event->Type==NODO_TYPE_EVENT)        // Er is en variabele gewijzigd. 
        {
        if(RGBVariables && !RGBFade)
          {
          if(UserVar[0]>255)UserVar[0]=255;
          if(UserVar[1]>255)UserVar[1]=255;
          if(UserVar[2]>255)UserVar[2]=255;

          if(UserVar[0]<0)UserVar[0]=0;
          if(UserVar[1]<0)UserVar[1]=0;
          if(UserVar[2]<0)UserVar[2]=0;

          InputLevelR = UserVar[0];
          InputLevelG = UserVar[1];
          InputLevelB = UserVar[2];
          Set=true;
          }
        }

      else if(event->Command==PLUGIN_ID && event->Type==NODO_TYPE_PLUGIN_EVENT) 
        { 
        x=event->Par1 & 0x1f;                                                   // Unit nummer
  
        if(x==0 || x==Settings.Unit)
          {
          if(((event->Par2>>24) & 0xff) & RGBLED_MODE_SETRGB)
            {
            InputLevelR = (event->Par2    ) & 0xff;
            InputLevelG = (event->Par2>>8 ) & 0xff;
            InputLevelB = (event->Par2>>16) & 0xff;
            }
                                       
          if(((event->Par2>>24) & 0xff) & RGBLED_MODE_FADE_ON)
            RGBFade=true;

          if(((event->Par2>>24) & 0xff) & RGBLED_MODE_FADE_OFF)
            RGBFade=false;

          if(((event->Par2>>24) & 0xff) & RGBLED_MODE_VARIABLES_ON)
            RGBVariables=true;

          if(((event->Par2>>24) & 0xff) & RGBLED_MODE_VARIABLES_OFF)
            RGBVariables=false;

          Set=true;
          }
        }

      if(Set)
        {            
        if(RGBFade)
          {
          // De waarde fade bevat het aantal minuten dat de omschakeling maximaal mag duren.
          // Per minuut 60000/SCAN_HIGH_TIME keer een call naar Plugin_023_FLC
          // In een minuut max. 255 dimniveaus ==> (60000/(SCAN_HIGH_TIME * 255)) calls nodig voor 1 dimniveau verschil. 
          if(RGBVariables)
            x=UserVar[3];
          else
            x=3;
            
          FadeTimeCounter=x*60000/(SCAN_HIGH_TIME*255);
          CalledCounter=0;
          FastLoopCall_ptr=&Plugin_023_FLC;
          }
        else
          {                                                                       
          while(FadeLed())
            delay(5);
          success=true;
          }
        }
      break;
      }
    #endif    

    #ifdef PLUGIN_023_CORE
    case PLUGIN_COMMAND:
      {
      ClearEvent(&TempEvent);
      TempEvent.DestinationUnit=event->Par1; 
      TempEvent.Port      = VALUE_ALL;
      TempEvent.Type      = NODO_TYPE_PLUGIN_EVENT;
      TempEvent.Command   = PLUGIN_ID;
      TempEvent.Par1      = event->Par1;
      TempEvent.Par2      = event->Par2;
      SendEvent(&TempEvent, false ,true, Settings.WaitFree==VALUE_ON);
      success=true;
      break;
      }
    #endif // CORE
    
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      y=1;                                                                      // Parameter teller
      if(GetArgv(string,TempStr,y++))
        {
        event->Type=0;
        event->Par1 = 0;
        
        // Command: RGBLedSend <NodoUnit>,<R>,<G>,<B> | fade | variables 
        if(strcasecmp(TempStr,PLUGIN_23_COMMAND)==0)
          {
          event->Type = NODO_TYPE_PLUGIN_COMMAND;

          if(GetArgv(string,TempStr,y++))                                       // RGBLed adres
            event->Par1=str2int(TempStr);
          }

        // Event: RGBLed <R>,<G>,<B>  | fade | variables
        if(strcasecmp(TempStr,PLUGIN_23_EVENT)==0)
          event->Type = NODO_TYPE_PLUGIN_EVENT;

        if(event->Type)
          {
          event->Command = PLUGIN_ID;                                           // Plugin nummer  
          event->Par2 = 0;
          
          if(GetArgv(string,TempStr,y++))                                       // R 
            {
            event->Par2=str2int(TempStr);
            if(GetArgv(string,TempStr,y++))                                     // G 
              {
              event->Par2|=str2int(TempStr)<<8;
              if(GetArgv(string,TempStr,y++))                                   // B
                {
                event->Par2|=str2int(TempStr)<<16;
                event->Par2|=((unsigned long)RGBLED_MODE_SETRGB)<<24;
                success=true;
                }
              }
            }
            
          if(!success)
            event->Par2=0;

          x=0;

          if(StringFind(string,PLUGIN_23_FADEON)!=-1)
            x|=RGBLED_MODE_FADE_ON;
            
          if(StringFind(string,PLUGIN_23_FADEOFF)!=-1)
            x|=RGBLED_MODE_FADE_OFF;
            
          if(StringFind(string,PLUGIN_23_VARON)!=-1)
            x|=RGBLED_MODE_VARIABLES_ON;

          if(StringFind(string,PLUGIN_23_VAROFF)!=-1)
            x|=RGBLED_MODE_VARIABLES_OFF;

          if(x)
            {
            event->Par2|=((unsigned long)x)<<24;
            success=true;
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      y=0;
      if(event->Type==NODO_TYPE_PLUGIN_COMMAND)
        {
        strcpy(string,PLUGIN_23_COMMAND);
        strcat(string," ");
        strcat(string,int2str(event->Par1));                                    // Unit
        y++;
        }
      else
        {
        strcpy(string,PLUGIN_23_EVENT);
        }
        
      if(((event->Par2>>24) & 0xff) & RGBLED_MODE_SETRGB)
        {
        if(y++)
          strcat(string,",");
        else
          strcat(string," ");
        
        strcat(string,int2str((event->Par2       ) &0xff));                     // Parameter-1 = R (8-bit)
        strcat(string,",");
        strcat(string,int2str((event->Par2 >> 8  ) &0xff));                     // Parameter-2 = G (8-bit)
        strcat(string,",");                                     
        strcat(string,int2str((event->Par2 >> 16 ) &0xff));                     // Parameter-3 = B (8-bit)
        y++;
        }

      x=(event->Par2>>24) & 0xff;
  
      if(y==0)strcat(string," ");
  
      if(x & RGBLED_MODE_FADE_ON)
        {
        if(y++)strcat(string,",");
        strcat(string,PLUGIN_23_FADEON);
        }

      if(x & RGBLED_MODE_FADE_OFF)
        {
        if(y++)strcat(string,",");
        strcat(string,PLUGIN_23_FADEOFF);
        }

      if(x & RGBLED_MODE_VARIABLES_ON)
        {
        if(y++)strcat(string,",");
        strcat(string,PLUGIN_23_VARON);
        }

      if(x & RGBLED_MODE_VARIABLES_OFF)
        {
        if(y++)strcat(string,",");
        strcat(string,PLUGIN_23_VAROFF);
        }
        
      break;
      }
    #endif //MMI
    }      
  return success;
  }

  