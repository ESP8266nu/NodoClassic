//#######################################################################################################
//#################################### Plugin-02: NewKAKU ###############################################
//#######################################################################################################

/*********************************************************************************************\
 * Dit protocol zorgt voor ontvangst Klik-Aan-Klik-Uit zenders
 * die werken volgens de automatische codering (Ontvangers met leer-knop) Dit protocol is 
 * eveneens bekend onder de naam HomeEasy. Het protocol ondersteunt eveneens dim funktionaliteit.
 * 
 * Auteur             : Nodo-team (P.K.Tonkes) www.nodo-domotca.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : Jan.2013
 * Versie             : 1.1
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 645
 ***********************************************************************************************
 *
 * Binnenkomend event: "NewKaku     <adres>,<On|Off|Dimlevel 1..15>
 * Versturen         : "NewKakuSend <adres>,<On|Off|Dimlevel 1..15> 
 *
 ***********************************************************************************************
 * Vereiste library   : - geen -
 *
 * Pulse (T) is 275us PDM
 * 0 = T,T,T,4T, 1 = T,4T,T,T, dim = T,T,T,T op bit 27
 *
 * NewKAKU ondersteund:
 *   on/off       ---- 000x Off/On
 *   all on/off   ---- 001x AllOff/AllOn
 *   dim absolute xxxx 0110 Dim16        // dim op bit 27 + 4 extra bits voor dim level
 *
 *  NewKAKU bitstream= (First sent) AAAAAAAAAAAAAAAAAAAAAAAAAACCUUUU(LLLL) -> A=KAKU_adres, C=commando, U=KAKU-Unit, L=extra dimlevel bits (optioneel)
 *
 * Interne gebruik van de parameters in het Nodo event:
 * 
 * Cmd  : Hier zit het commando SendNewKAKU of het event NewKAKU in. Deze gebruiken we verder niet.
 * Par1 : Commando VALUE_ON, VALUE_OFF of dim niveau [1..15]
 * Par2 : Adres
 *
 \*********************************************************************************************/

#define PLUGIN_ID 2
#define PLUGIN_002_EVENT       "NewKAKU"
#define PLUGIN_002_COMMAND "NewKAKUSend"
#define NewKAKU_RawSignalLength      132
#define NewKAKUdim_RawSignalLength   148
#define NewKAKU_1T                   275        // us
#define NewKAKU_mT                   650        // us, ergens ongeveer midden tussen 1T en 4T 
#define NewKAKU_4T                  1100        // us
#define NewKAKU_8T                  2200        // us, Tijd van de space na de startbit

boolean Plugin_002(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  
  switch(function)
    {
    #ifdef PLUGIN_002_CORE
    case PLUGIN_RAWSIGNAL_IN:
      {
      unsigned long bitstream=0L;
      boolean Bit;
      int i;
      int P0,P1,P2,P3;
      event->Par1=0;
      
      // nieuwe KAKU bestaat altijd uit start bit + 32 bits + evt 4 dim bits. Ongelijk, dan geen NewKAKU
      if (RawSignal.Number==NewKAKU_RawSignalLength || RawSignal.Number==NewKAKUdim_RawSignalLength)
        {
        // RawSignal.Number bevat aantal pulsRawSignal.Multiplyen * 2  => negeren
        // RawSignal.Pulses[1] bevat startbit met tijdsduur van 1T => negeren
        // RawSignal.Pulses[2] bevat lange space na startbit met tijdsduur van 8T => negeren
        i=3; // RawSignal.Pulses[3] is de eerste van een T,xT,T,xT combinatie
        
        do 
          {
          P0=RawSignal.Pulses[i]    * RawSignal.Multiply;
          P1=RawSignal.Pulses[i+1]  * RawSignal.Multiply;
          P2=RawSignal.Pulses[i+2]  * RawSignal.Multiply;
          P3=RawSignal.Pulses[i+3]  * RawSignal.Multiply;
          
          if     (P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3>NewKAKU_mT)Bit=0; // T,T,T,4T
          else if(P0<NewKAKU_mT && P1>NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)Bit=1; // T,4T,T,T
          else if(P0<NewKAKU_mT && P1<NewKAKU_mT && P2<NewKAKU_mT && P3<NewKAKU_mT)       // T,T,T,T Deze hoort te zitten op i=111 want: 27e NewKAKU bit maal 4 plus 2 posities voor startbit
            {
            if(RawSignal.Number!=NewKAKUdim_RawSignalLength)                    // als de dim-bits er niet zijn
              return false;
            }
          else
            return false;                                                       // andere mogelijkheden zijn niet geldig in NewKAKU signaal.  
            
          if(i<130)                                                             // alle bits die tot de 32-bit pulstrein behoren 32bits * 4posities per bit + pulse/space voor startbit
            bitstream=(bitstream<<1) | Bit;
          else                                                                  // de resterende vier bits die tot het dimlevel behoren 
            event->Par1=(event->Par1<<1) | Bit;
       
          i+=4;                                                                 // volgende pulsenquartet
          }while(i<RawSignal.Number-2);                                         //-2 omdat de space/pulse van de stopbit geen deel meer van signaal uit maakt.
            
        if(bitstream>0xffff)                                                    // Adres-deel: Is het signaal van een originele KAKU zender afkomstig, of van een Nodo ingegeven door de gebruiker ?
          event->Par2=bitstream &0x0FFFFFCF;                                    // Op hoogste nibble zat vroeger het signaaltype. & is t.b.v. compatibiliteit 
        else                                                                    // Het is van een andere Nodo afkomstig. 
          event->Par2=(bitstream>>6)&0xff;                                      // Neem dan alleen 8bit v/h adresdeel van KAKU signaal over
        
        if(i>140)                                                               // Commando en Dim deel
          event->Par1++;                                                        // Dim level. +1 omdat gebruiker dim level begint bij één.
        else
          event->Par1=((bitstream>>4)&0x01)?VALUE_ON:VALUE_OFF;                 // On/Off bit omzetten naar een Nodo waarde. 
        event->SourceUnit    = 0;                                               // Komt niet van een Nodo unit af, dus unit op nul zetten
        RawSignal.Repeats    = true;                                            // het is een herhalend signaal. Bij ontvangst herhalingen onderdrukken.
        event->Type          = NODO_TYPE_PLUGIN_EVENT;
        event->Command       = PLUGIN_ID;                                       // nummer van dit device
        success=true;
        }   
      break;
      }
      
    case PLUGIN_COMMAND:
      {
      unsigned long bitstream=0L;
      byte i=1;
      byte x;                                                                   // aantal posities voor pulsen/spaces in RawSignal
        
      // bouw het KAKU adres op. Er zijn twee mogelijkheden: Een adres door de gebruiker opgegeven binnen het bereik van 0..255 of een lange hex-waarde
      if(event->Par2<=255)
        bitstream=1|(event->Par2<<6);                                           // Door gebruiker gekozen adres uit de Nodo_code toevoegen aan adres deel van de KAKU code. 
      else
        bitstream=event->Par2 & 0xFFFFFFCF;                                     // adres geheel over nemen behalve de twee bits 5 en 6 die het schakel commando bevatten.
    
      event->Port=VALUE_ALL;                                                    // Signaal mag naar alle door de gebruiker met [Output] ingestelde poorten worden verzonden.
      RawSignal.Repeats=7;                                                      // Aantal herhalingen van het signaal.
      RawSignal.Delay=20;                                                       // Tussen iedere pulsenreeks enige tijd rust.
      RawSignal.Multiply=25;

      if(event->Par1==VALUE_ON || event->Par1==VALUE_OFF)
        {
        bitstream|=(event->Par1==VALUE_ON)<<4;                                  // bit-5 is het on/off commando in KAKU signaal
        x=130;                                                                  // verzend startbit + 32-bits = 130
        }
      else
        x=146;                                                                  // verzend startbit + 32-bits = 130 + 4dimbits = 146
     
      // bitstream bevat nu de KAKU-bits die verzonden moeten worden.
    
      for(i=3;i<=x;i++)RawSignal.Pulses[i]=NewKAKU_1T/RawSignal.Multiply;       // De meeste tijden in signaal zijn T. Vul alle pulstijden met deze waarde. Later worden de 4T waarden op hun plek gezet
      
      i=1;
      RawSignal.Pulses[i++]=NewKAKU_1T/RawSignal.Multiply;                      //pulse van de startbit
      RawSignal.Pulses[i++]=NewKAKU_8T/RawSignal.Multiply;                      //space na de startbit
      
      byte y=31;                                                                // bit uit de bitstream
      while(i<x)
        {
        if((bitstream>>(y--))&1)
          RawSignal.Pulses[i+1]=NewKAKU_4T/RawSignal.Multiply;                  // Bit=1; // T,4T,T,T
        else
          RawSignal.Pulses[i+3]=NewKAKU_4T/RawSignal.Multiply;                  // Bit=0; // T,T,T,4T
    
        if(x==146)                                                              // als het een dim opdracht betreft
          {
          if(i==111)                                                            // Plaats van de Commando-bit uit KAKU 
            RawSignal.Pulses[i+3]=NewKAKU_1T/RawSignal.Multiply;                // moet een T,T,T,T zijn bij een dim commando.
          if(i==127)                                                            // als alle pulsen van de 32-bits weggeschreven zijn
            {
            bitstream=(unsigned long)event->Par1-1;                             //  nog vier extra dim-bits om te verzenden. -1 omdat dim niveau voor gebruiker begint bij 1
            y=3;
            }
          }
        i+=4;
        }
      RawSignal.Pulses[i++]=NewKAKU_1T/RawSignal.Multiply;                      //pulse van de stopbit
      RawSignal.Pulses[i]=0;                                                    //space van de stopbit
      RawSignal.Number=i;                                                       // aantal bits*2 die zich in het opgebouwde RawSignal bevinden
      SendEvent(event,true,true,Settings.WaitFree==VALUE_ON);
      success=true;
      break;
      }


    #endif // CORE
      
    #if NODO_MEGA
    case PLUGIN_MMI_IN:
      {
      char* str=(char*)malloc(INPUT_COMMAND_SIZE);
    
      if(GetArgv(string,str,1))
        {
        event->Type=0;

        if(strcasecmp(str,PLUGIN_002_EVENT)==0)
          event->Type=NODO_TYPE_PLUGIN_EVENT;

        if(strcasecmp(str,PLUGIN_002_COMMAND)==0)
          event->Type=NODO_TYPE_PLUGIN_COMMAND;
        
        if(event->Type)
          {
          if(GetArgv(string,str,2))
            {
            event->Par2=str2int(str);    
            if(GetArgv(string,str,3))
              {
              // Vul Par1 met het KAKU commando. Dit kan zijn: VALUE_ON, VALUE_OFF, 1..16. Andere waarden zijn ongeldig.
              
              // haal uit de tweede parameter een 'On' of een 'Off'.
              if(event->Par1=str2cmd(str))
                success=true;
                
              // als dit niet is gelukt, dan uit de tweede parameter de dimwaarde halen.
              else
                {
                event->Par1=str2int(str);                                       // zet string om in integer waarde
                if(event->Par1>=1 && event->Par1<=16)                           // geldig dim bereik 1..16 ?
                   success=true;
                }
              event->Command = PLUGIN_ID;                                       // Plugin nummer  
              }
            }
          }
        }
      free(str);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      if(event->Type==NODO_TYPE_PLUGIN_EVENT)
        strcpy(string,PLUGIN_002_EVENT);                                         // Eerste argument=het commando deel

      if(event->Type==NODO_TYPE_PLUGIN_COMMAND)
        strcpy(string,PLUGIN_002_COMMAND);                                      // Eerste argument=het commando deel

      strcat(string," ");
    
      // In Par3 twee mogelijkheden: Het bevat een door gebruiker ingegeven adres 0..255 of een volledig NewKAKU adres.
      if(event->Par2>=0x0ff)
        strcat(string,int2strhex(event->Par2)); 
      else
        strcat(string,int2str(event->Par2)); 
      
      strcat(string,",");
      
      if(event->Par1==VALUE_ON)
        strcat(string,"On");  
      else if(event->Par1==VALUE_OFF)
        strcat(string,"Off");
      else
        strcat(string,int2str(event->Par1));

      break;
      }
    #endif //MMI
    }      
  return success;
  }

