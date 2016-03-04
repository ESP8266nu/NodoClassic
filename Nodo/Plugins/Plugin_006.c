//#######################################################################################################
//######################## PLUGIN-06 Temperatuur en Luchtvochtigheid sensor DHT 11/22 ###################
//#######################################################################################################

/*********************************************************************************************\
 * 
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 * Support            : www.nodo-domotica.nl
 * Datum              : 12 Aug 2013
 * Versie             : 1.3
 *                      
 *                      Modificaties: 
 *                      2013-11, V1.3, Paul Tonkes: Geen events meer genereren bij vullen variabelen
 *                      2013-11, V1.2, Paul Tonkes: Retry bij fout tijdens uitlezen en DHT-22 compatibel
 *                         
 * Nodo productnummer : n.v.t. meegeleverd met Nodo code.
 * Compatibiliteit    : Vanaf Nodo build nummer 555
 * Syntax             : "DHTRead <Par1:Poortnummer>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Configuratie:
 * 
 * Deze plugin is zowel geschikt voor een DHT-11 sensor alsmede de DHT-22 die een veel hogere
 * resolutie heeft. Deze laatste heeft dan ook de voorkeur. Geef in je config_nn.c file op welke sensor 
 * je gebruikt met:
 * 
 * #define PLUGIN_006_CORE <DHT-type>
 * 
 * Kies voor <DHT-type> 11 voor een DHT-11 sensor of 22 voor een DHT-22. Verkeerde definitie zal leiden 
 * tot geen of foutieve uitlezing.
 *    
 * Technische informatie:
 *  
 * De DHT sensor is een 3 pin sensor met een bidirectionele datapin. Deze verbind je met een WiredOut poort. 
 * Het principe is "onewire" maar dit protocol is NIET compatible met het bekende Dallas onewire protocol
 * Dit protocol gebruikt twee variabelen, 1 voor temperatuur en 1 voor luchtvochtigheid
 \*********************************************************************************************/

//Onderstaande regel is voor Nodo releases eerder dan 3.6.1.
#ifndef VALUE_SOURCE_PLUGIN
#define VALUE_SOURCE_PLUGIN VALUE_SOURCE_SYSTEM
#endif

#define PLUGIN_ID 06
#define PLUGIN_NAME "DHTRead"
uint8_t DHT_Pin;

#ifdef PLUGIN_006_CORE
/*********************************************************************************************\
 * DHT sub to get an 8 bit value from the receiving bitstream
 \*********************************************************************************************/
byte read_dht_dat(void)
  {
  byte i = 0;
  byte result=0;
  noInterrupts();
  for(i=0; i< 8; i++)
    {
    while(!digitalRead(DHT_Pin));  // wait for 50us
    delayMicroseconds(30);
    if(digitalRead(DHT_Pin)) 
      result |=(1<<(7-i));
    while(digitalRead(DHT_Pin));  // wait '1' finish
    }
  interrupts();
  return result;
  }
#endif // CORE


boolean Plugin_006(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  
  switch(function)
    {
    #ifdef PLUGIN_006_CORE
  
    case PLUGIN_COMMAND:
      {
      DHT_Pin=PIN_WIRED_OUT_1+event->Par1-1;
      byte dht_dat[5];
      byte dht_in;
      byte i;
      byte Retry=0;

      do
        {  
        pinMode(DHT_Pin,OUTPUT);
        // DHT start condition, pull-down i/o pin for 18ms
        digitalWrite(DHT_Pin,LOW);               // Pull low
        delay(18);
        digitalWrite(DHT_Pin,HIGH);              // Pull high
        delayMicroseconds(40);
        pinMode(DHT_Pin,INPUT);                  // change pin to input
        delayMicroseconds(40);
    
        dht_in = digitalRead(DHT_Pin);
        if(!dht_in)
          {
          delayMicroseconds(80);
          dht_in = digitalRead(DHT_Pin);
          if(dht_in)
            {
            delayMicroseconds(40);                     // now ready for data reception
            for (i=0; i<5; i++)
              dht_dat[i] = read_dht_dat();
              
            // Checksum calculation is a Rollover Checksum by design!
            byte dht_check_sum = dht_dat[0]+dht_dat[1]+dht_dat[2]+dht_dat[3];// check check_sum

            if(dht_dat[4]== dht_check_sum)
              {

              #if PLUGIN_006_CORE==11
              UserVar[event->Par2 -1] = float(dht_dat[2]); // Temperatuur
              UserVar[event->Par2   ] = float(dht_dat[0]); // vochtigheid
              #endif
              
              #if PLUGIN_006_CORE==22
              if (dht_dat[2] & 0x80) // negative temperature
                UserVar[event->Par2 -1] = -0.1 * word(dht_dat[2] & 0x7F, dht_dat[3]);
              else
                UserVar[event->Par2 -1] = 0.1 * word(dht_dat[2], dht_dat[3]);
              UserVar[event->Par2] = word(dht_dat[0], dht_dat[1]) * 0.1; // vochtigheid
              #endif

              success=true;
              }
            }
          }
        if(!success)
          {
          delay(2000);
          }
        }while(!success && ++Retry<3);        
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
          if(event->Par1>0 && event->Par1<WIRED_PORTS && event->Par2>0 && event->Par2<=USER_VARIABLES_MAX-1)
            {
            event->Type = NODO_TYPE_PLUGIN_COMMAND;
            event->Command = 6; // Plugin nummer  
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
      strcat(string,int2str(event->Par2));
      break;
      }
    #endif // MMI
    }      
  return success;
  }
