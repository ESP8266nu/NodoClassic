//#######################################################################################################
//#################################### Plugin-255: MyPlugin   ############################################
//#######################################################################################################

/*********************************************************************************************\
* Funktionele beschrijving: Dit is een leeg plugin dat als voorbeeld is opgenomen. Aan de hand
*                           van dit voorbeeld kan een gebruiker zelf een eigen plugin ontwikkelen.
*                           Als je plugin hebt ontwikkeld die voor anderen ook van nut kan zijn,
*                           meld deze dan aan bij het Nodo-team om deze te delen met andere gebruikers
*                           en om een Plugin-ID aan te vragen.
* 
* <Geef hier een beschrijving van de funktionele werking van het plugin en hoe deze door de gebruiker
* moet worden aangeroepen.>
*
* Auteur             : <naam en EMail adres van de ontwikkelaar>
* Support            : <website>
* Datum              : <datum>
* Versie             : <versie>
* Nodo productnummer : <Nodo productnummer. Toegekend door Nodo team>
* Compatibiliteit    : Vanaf Nodo build nummer <build nummer>
* Syntax             : "MijnPlugin <Par1>, <Par2>"
*
***********************************************************************************************
* Technische beschrijving:
*
* Compiled size      : <grootte> bytes voor een Mega en <grootte> voor een Small.
* Externe funkties   : <geef hier aan welke funkties worden gebruikt. 
*
* <Geef hier een technische toelichting op de werking van het plugin en eventueel gebruikte protocol>
* 
* PIN_WIRED_OUT_n staat voor WiredOut poort, waarbij n overeen komt met het nummer van de WiredOut poort te beginnen met 1.
* PIN_WIRED_IN_n staat voor WiredOut poort, waarbij n overeen komt met het nummer van de WiredOut poort te beginnen met 1.
* PIN_IO_n staat voor additionele IO-poort, waarbij n overeen komt met het nummer van deze poort, te beginnen met 1 (Alleen beschikbaar op de Mega).
* 
\*********************************************************************************************/
 
#define PLUGIN_NAME "BRC" // Boiler Room Controller
#define PLUGIN_ID   11

boolean Plugin_011(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;

  // Deze plugin code wordt vanuit meerdere plaatsen in de Nodo code aangeroepen, steeds met een doel. Dit doel bevindt zich
  // in de variabele [function]. De volgende zijn gedefinieerd:
  //
  // PLUGIN_RAWSIGNAL_IN  => Afhandeling van een via RF/IR ontvangen event
  // PLUGIN_COMMAND       => Commando voor afhandelen/uitsturen van een event.
  // PLUGIN_MMI_IN        => Invoer van de gebruiker/script omzetten naar een event. (Alleen voor mega)
  // PLUGIN_MMI_OUT       => Omzetten van een event naar een voor de gebruiker leesbare tekst (Alleen voor Mega)
  // DEVIDE_ONCE_A_SECOND => ongeveer iedere seconde.
  // PLUGIN_INIT          => Eenmalig, direct na opstarten van de Nodo
  // PLUGIN_EVENT_IN      => Vlak voor verwerking van een binnengekomen event.
  // PLUGIN_SERIAL_IN     => Zodra er bytes via de seriele poort zijn ontvangen
  // PLUGIN_ETHERNET_IN   => Zodra er bytes via de seriele poort zijn ontvangen
  
  
  switch(function)
    {    
    #ifdef PLUGIN_011_CORE

    case PLUGIN_INIT:
      {
      Serial.println(F("*** debug: BRC Plugin started.")); //??? Debug
      break;
      }

    case PLUGIN_EVENT_IN:
      {
      Serial.println(F("*** debug: BRC: PLUGIN_EVENT_IN"));
      break;      
      }

    case PLUGIN_ONCE_A_SECOND:
      {
      break;
      }

    case PLUGIN_RAWSIGNAL_IN:
      {
      Serial.println(F("*** debug: BRC: PLUGIN_RAWSIGNAL_IN"));
      break;
      }
      
    case PLUGIN_COMMAND:
      {
      Serial.print(F("*** debug: BRC: PLUGIN_COMMAND"));
      Serial.print(", Par1=");    Serial.print(event->Par1);
      Serial.print(", Par2=");    Serial.println(event->Par2);
      
      // Als voorbeeld wordt hier variabele 5 gevuld met 123.45
      byte Variable = 5;
      float Value   = 123.45;

      ClearEvent(event);                                      // Ga uit van een default schone event. Oude eventgegevens wissen.      
      event->Type         = NODO_TYPE_COMMAND;                // Het event is een uit te voeren commando
      event->Command      = CMD_VARIABLE_SET;                 // Commando "VariableSet"
      event->Par1         = Variable;                         // Par1 draagt het variabelenummer
      event->Par2         = float2ul(Value);                  // Par2 de waarde. float2ul() zet de waarde m naar een geschikt format.
      
      success=true;                                           // Als verlaten wordt met een true, en er is een nieuw event in de struct geplaatst
                                                              // dan wordt deze automatisch uitgevoerd.
                                                              // Een plugin kan geen andere plugin aanroepen.
                                                              // verder is er geen beperking en kunnen alle events/commando's worden
                                                              // opgegeven voor verdere verwerking.
      break;
      }      

    #endif // CORE
    
    #if NODO_MEGA // alleen relevant voor een Nodo Mega want de Small heeft geen MMI!
    case PLUGIN_MMI_IN:
      {
      Serial.print(F("*** debug: BRC: PLUGIN_MMI_IN, string="));Serial.println(string); //??? Debug
      // Zodra er via een script, HTTP, Telnet of Serial een commando wordt ingevoerd, wordt dit deel van de code langs gelopen.
      // Op deze plek kan de invoer [string] worden geparsed en omgezet naar een struct [event]. Als parsen van de invoerstring [string]
      // is gelukt en de struct is gevuld, dan de variabele [success] vullen met true zodat de Nodo zorg kan dragen voor verdere verwerking van het event.

      char *TempStr=(char*)malloc(INPUT_COMMAND_SIZE);

      // Met PLUGIN_MMI_IN wordt de invoer van de gebruiker (string) omgezet naar een event zodat de Nodo deze intern kan verwerken.
      // Hier aangekomen bevat string het volledige commando. Test als eerste of het opgegeven commando overeen komt met PLUGIN_NAME
      // Dit is het eerste argument in het commando. 
      if(GetArgv(string,TempStr,1))
        {
        // Als het door de gebruiker ingegeven ommando/event overeenkomt met de naam van dit plugin...
        if(strcasecmp(TempStr,PLUGIN_NAME)==0)
          {
          // in dit voorbeeld even laten weten dat de code wordt geraakt. Directe output naar
          // Serial is normaal gesproken NIET wenselijk in een plugin. 

          // Vervolgens tweede parameter gebruiken
          if(GetArgv(string,TempStr,2)) 
            {
            // plaats hier je code die de eerste parameter die zich in [TempStr] bevindt verder uitparsed
            // De byte Par1 en unsigned long Par2 die zich in de struct [event] bevindt kunnen worden gebruikt.
              
            if(GetArgv(string,TempStr,3))
              {
              // indien gewenst kan een tweede parameter worden gebruikt (=derde argument)
              // Plaats wederom de code paar uitparsen van de parameter hier.
              // heb je het laatste parameter geparsen, dan de variabele [success] vullen 
              // met een true zodat verdere verwerking van het event plaats kan vinden.

              // Een plugin kan worden verwerkt als een commando of een event. Geef dit aan want 
              // op moment van invoer moet de Nodo t.b.v. latere verwerking weten hoe de zaken afgehandeld moeten worden
              event->Type    = NODO_TYPE_PLUGIN_COMMAND;
              event->Command = PLUGIN_ID; // nummer van dit plugin
              
              // Als success wordt gevuld met true, dan wordt het commando/event
              // geaccepteerd als geldig.
              
              success=true;
              }
            }
          }
        }
      free(TempStr);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      // De code die zich hier bevindt zorgt er voor dan een event met het unieke ID in de struct [event] weer wordt
      // omgezet naar een leesbaar event voor de gebruiker. het resultaat moet worden geplaatst in de string [string]
      // let op dat het totale commando niet meer dan 25 posities in beslag neemt.
      // Dit deel van de code wordt alleen uitgevoerd door een Nodo Mega, omdat alleen deze over een MMI beschikt.
      Serial.println(F("*** debug: BRC: PLUGIN_MMI_OUT")); //??? Debug

      strcpy(string,PLUGIN_NAME);               // Commando 
      strcat(string," ");
      strcat(string,int2str(event->Par1));      // Parameter-1 (8-bit)
      strcat(string,",");
      strcat(string,int2str(event->Par2));      // Parameter-2 (32-bit)

      break;
      }
    #endif //MMI
    }
  return success;
  }

