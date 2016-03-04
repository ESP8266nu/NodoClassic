//#######################################################################################################
//#################################### Plugin-099: NodoTool #############################################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides communications with the "NodoTool"
 * 
 * Author             : M vd Broek
 * Date               : 29 Feb 2016
 * Version            : 1.1
 * Compatibility      : R787
 \*********************************************************************************************/

#define PLUGIN_ID 99
#define PLUGIN_NAME "NodoTool"

#define EVENTLIST_MAX 64
byte Par[16];

boolean Plugin_099(byte function, struct NodoEventStruct *event, char *string)
{
  boolean success=false;

  switch(function)
  {
#ifdef PLUGIN_099_CORE

  case PLUGIN_EVENT_IN:
  case PLUGIN_EVENT_OUT:
    {
//Serial.print("EIO:");
      if (event->Command !=0)
        {
          Serial.print(event->Direction);
          Serial.print(",");
          Serial.print(event->Port);
          Serial.print(",");
          Serial.print(event->Command);
          Serial.print(",");
          Serial.print(event->Par1);
          Serial.print(",");
          Serial.print(event->Par2,HEX);
          Serial.print(",");
          Serial.print(event->SourceUnit);
          Serial.print(",");
          Serial.println(event->Type);
        }
      break;
    }

  case PLUGIN_COMMAND:
    {
//Serial.print("CO:");
      if (LastReceived.Command !=0)
        {
          Serial.print(LastReceived.Direction);
          Serial.print(",");
          Serial.print(LastReceived.Port);
          Serial.print(",");
          Serial.print(LastReceived.Command);
          Serial.print(",");
          Serial.print(LastReceived.Par1);
          Serial.print(",");
          Serial.print(LastReceived.Par2,HEX);
          Serial.print(",");
          Serial.print(LastReceived.SourceUnit);
          Serial.print(",");
          Serial.println(LastReceived.Type);
        }
      success=true;
      break;
    }

  case PLUGIN_SERIAL_IN:
    {
      byte SerialInByte;
      if(Serial.available())
      {
        delay(50);
        for(byte x=0; x < 16; x++) Par[x]=0;
        SerialInByte=Serial.read();

        if (SerialInByte > 16) SerialInByte=16;
 
        for(byte x=0; x < SerialInByte; x++) Par[x]=Serial.read();

        if (Par[0] == 's')
          {
            int baseaddress=sizeof(struct SettingsStruct);
            Serial.write(0xff); // Byte value to signal start of binary data
            Serial.write((byte)(EVENTLIST_MAX*14+1) & 0xff); // send message length, LSB
            Serial.write((byte)(((EVENTLIST_MAX*14+1) >> 8) & 0xff)); // send message length, MSB
            Serial.write(0x1); // send messagetype (1=eventlistshow)
            for (int x=0; x < EVENTLIST_MAX*14; x++) Serial.write(EEPROM.read(baseaddress+x));
          }
      
        if (Par[0] == 'c')
          {
            NodoEventStruct TempEvent;
            TempEvent.Type=Par[1];
            TempEvent.Command = Par[2];
            TempEvent.Par1 = Par[3];
            TempEvent.Par2 = 0;
            for (byte x=4; x < 8; x++)
              {
                TempEvent.Par2 = TempEvent.Par2 + ((unsigned long)Par[x] << (8*(x-4)));
              }
            TempEvent.Port = VALUE_SOURCE_SYSTEM;
            TempEvent.SourceUnit=0;
            TempEvent.DestinationUnit=0;
            TempEvent.Direction=VALUE_DIRECTION_INPUT;
            if (TempEvent.Type == NODO_TYPE_COMMAND)
              {
                ExecuteCommand(&TempEvent);
              }
           else
             {
               for(byte x=0;Plugin_ptr[x]!=0 && x<PLUGIN_MAX; x++)
                 if(Plugin_id[x]==TempEvent.Command)
                   Plugin_ptr[x](PLUGIN_COMMAND,&TempEvent,0);
             }
          }
       
        if (Par[0] == 'w')
          {
            int baseaddress=sizeof(struct SettingsStruct)+(Par[1]-1)*14;
            for (byte x=2; x < 16; x++)
              {
                EEPROM.write(baseaddress+x-2, Par[x]);
              }
          }

        Serial.println("ok");
        break;
      }  // if
    } // case
#endif // PLUGIN_099_CORE
  }      
  return success;
}
