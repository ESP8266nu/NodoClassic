//#######################################################################################################
//#################################### Plugin-094: IRRemote #############################################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides receive/send IR Remote commands for Samsung, Humax, Marmitek, Sony, Philips equipment
 * 
 * Author             : M vd Broek
 * Date               : Jun.2014
 * Version            : 0.1
 * Compatibility      : R787
 ***********************************************************************************************
 *
 * Event                  : "IRRemote     <protocol, ir code>
 * Command                : "IRRemoteSend <protocol, ir code> 
 * Supported Protocols	  : 1 = Samsung 32 bit, also working on Humax PVR and Marmitek HDMI switch
 *                        : 2 = Sony 20 bit, working on Sony BD185 Blu-Ray
 *                        : 3 = Philips RC5
 * ========================================================================================
 * "Samsung 32 bit protocol"
 * ========================================================================================
 * Pulselength : 68 pulses
 *               2 start pulses (4000 mSec High, 4000 mSec Low)
 *               64 data pulses (500 mSec High, followed by 500/1400 mSec Low)
 *                               500 mSec Low = 0
 *                               1400 mSec Low = 1
 *               2 stop pulses (500 mSec High, 500mSec Low)
 *
 * This pulse stream (PWM) decodes into 32 bits
 * Is seems that first 16 bits are some sort of manufacturer code
 * Remaining 16 bits are address/command
 *
 * Protocol details as found on Samsung, Humax and Marmitek devices
 * First 16 bits Samsung TV		11100000 11100000
 * First 16 bits Humax PVR		00000000 00001000
 * First 16 bits Marmitek HDMI Switch	00000000 11111111
 * ========================================================================================
 * "Sony 20 bit protocol"
 * ========================================================================================
 * Pulselength : 42 pulses
 *               1 start pulses (2000 mSec High)
 *               40 data pulses (500 mSec Low, followed by 500/1000 mSec High)
 *                               500 mSec Low = 0
 *                               1000 mSec Low = 1
 *               1 stop pulses (500 mSec Low)
 *
 * This pulse stream (PWM) decodes into 20 bits
 * ========================================================================================
 * "Philips RC5 protocol"
 * ========================================================================================
 * Pulselength : varies between 20-28 pulses using manchester encoding
 * low/high = 1
 * high/low = 0
 *               2 start bits (in case of RC5x: second start bit is command bit 7 inverted)
 *               1 toggle bit
 *               5 address bits
 *               6 command bits
 \*********************************************************************************************/

#define PLUGIN_ID                               94
#define PLUGIN_094_EVENT                "IRRemote"
#define PLUGIN_094_COMMAND          "IRRemoteSend"

#define PLUGIN_094_SAMSUNG32_ID                  1
#define PLUGIN_094_SAMSUNG32_PULSECOUNT         68
#define PLUGIN_094_SAMSUNG32_START            3000
#define PLUGIN_094_SAMSUNG32_THRESHOLD         700

#define PLUGIN_094_SONY20_ID                     2
#define PLUGIN_094_SONY20_PULSECOUNT            42
#define PLUGIN_094_SONY20_START               2000
#define PLUGIN_094_SONY20_THRESHOLD            700

#define PLUGIN_094_PHILIPSRC5_ID                 3
#define PLUGIN_094_PHILIPSRC5_PULSECOUNTMIN     20
#define PLUGIN_094_PHILIPSRC5_PULSECOUNTMAX     28
#define PLUGIN_094_PHILIPSRC5_THRESHOLD       1000
#define PLUGIN_094_PHILIPSRC5_PULSELENGTH      900

static byte Plugin_094_debug=0;

boolean Plugin_094(byte function, struct NodoEventStruct *event, char *string)
  {
  boolean success=false;
  
  switch(function)
    {
    #ifdef PLUGIN_094_CORE
    case PLUGIN_RAWSIGNAL_IN:
      {
        if(Plugin_094_debug == 1)
          {
            Serial.print('R');
            Serial.print((int)RawSignal.Number);
            Serial.print(',');
            for(byte x=1;x<=RawSignal.Number;x++)
              {
                Serial.print(RawSignal.Pulses[x]*RawSignal.Multiply);
                Serial.print(',');
              }
            Serial.println();
          }

        byte protocol=0;
        unsigned long bitstream=0L;

        // Samsung 32 bit protocol
        if ((RawSignal.Number == PLUGIN_094_SAMSUNG32_PULSECOUNT) && (RawSignal.Pulses[1]*RawSignal.Multiply > PLUGIN_094_SAMSUNG32_START) && (RawSignal.Pulses[2]*RawSignal.Multiply > PLUGIN_094_SAMSUNG32_START))
          {
            for(byte x=4;x<=66;x=x+2)
            {
              if (RawSignal.Pulses[x]*RawSignal.Multiply > PLUGIN_094_SAMSUNG32_THRESHOLD) bitstream = (bitstream << 1) | 0x1;
              else bitstream = bitstream << 1;
            }
            protocol=PLUGIN_094_SAMSUNG32_ID;
          }

        // Sony 20 bit protocol
        if ((RawSignal.Number == PLUGIN_094_SONY20_PULSECOUNT) && (RawSignal.Pulses[1]*RawSignal.Multiply > PLUGIN_094_SONY20_START))
          {
            for(byte x=3;x<=41;x=x+2)
            {
              if (RawSignal.Pulses[x]*RawSignal.Multiply > PLUGIN_094_SONY20_THRESHOLD) bitstream = (bitstream << 1) | 0x1;
              else bitstream = bitstream << 1;
            }
            protocol=PLUGIN_094_SONY20_ID;
          }

        // Philips RC5 protocol
        if ((RawSignal.Number >= PLUGIN_094_PHILIPSRC5_PULSECOUNTMIN) && (RawSignal.Number <= PLUGIN_094_PHILIPSRC5_PULSECOUNTMAX))
          {
            // "noise filter"
            for(byte x=1;x<RawSignal.Number;x++)
              if ((RawSignal.Pulses[x]*RawSignal.Multiply) < 600 || (RawSignal.Pulses[x]*RawSignal.Multiply) > 2000)
                return false;

            byte rfbit = 1;
            for(byte x=1;x<=RawSignal.Number;x++)
              {
                if(RawSignal.Pulses[x]*RawSignal.Multiply > PLUGIN_094_PHILIPSRC5_THRESHOLD)
                  rfbit = rfbit ^ 1;
                else
                  x++;

                  bitstream = (bitstream << 1) | rfbit;
              }
            // correct by shifting one bit, mask out toggle bit and invert 1st bit (RC5x)
            bitstream = ((bitstream >> 1) & 0x17ff) ^ 0x1000;
            protocol=PLUGIN_094_PHILIPSRC5_ID;
          }

        if (protocol != 0)
          {
            event->Par1          = protocol;
            event->Par2          = bitstream;
            event->SourceUnit    = 0;
            event->Type          = NODO_TYPE_PLUGIN_EVENT;
            RawSignal.Repeats    = true;
            event->Command       = PLUGIN_ID;
            success=true;
          }

      break;
      }
      
    case PLUGIN_COMMAND:
      {
        if (event->Par1 == 255)
          Plugin_094_debug=event->Par2;

        unsigned long bitstream=event->Par2;
        RawSignal.Multiply=25;
        RawSignal.Repeats=3;
        RawSignal.Delay=100;

        // Samsung 32 bit protocol
        if (event->Par1 == PLUGIN_094_SAMSUNG32_ID)
          {
            RawSignal.Pulses[1]=4500/RawSignal.Multiply;
            RawSignal.Pulses[2]=4500/RawSignal.Multiply;
            for(byte x=65;x>=3;x=x-2)
              {
                RawSignal.Pulses[x]=400/RawSignal.Multiply;
                if ((bitstream & 1) == 1) RawSignal.Pulses[x+1] = 1400/RawSignal.Multiply; 
                else RawSignal.Pulses[x+1] = 400/RawSignal.Multiply;

                bitstream = bitstream >> 1;
              }
            RawSignal.Pulses[67]=300/RawSignal.Multiply;
            RawSignal.Pulses[68]=0;
            RawSignal.Number=68;
          }

        // Sony 20 Bit protocol
        if (event->Par1 == PLUGIN_094_SONY20_ID)
          {
            RawSignal.Pulses[1]=2200/RawSignal.Multiply;
            for(byte x=40;x>=2;x=x-2)
              {
                RawSignal.Pulses[x]=450/RawSignal.Multiply;
                if ((bitstream & 1) == 1) RawSignal.Pulses[x+1] = 1000/RawSignal.Multiply; 
                else RawSignal.Pulses[x+1] = 500/RawSignal.Multiply;

                bitstream = bitstream >> 1;
              }
            RawSignal.Pulses[42]=0;
            RawSignal.Number=42;
          }

        // Philips RC5 protocol
        if (event->Par1 == PLUGIN_094_PHILIPSRC5_ID)
          {
            byte prevstate=1;
            byte bitstate=0;
            byte c=0;
            if (bitstream > 0x1000) // RC5x
              {
                bitstream = bitstream & 0xfff; // clear second stop bit (inverted data bit)
                bitstream = bitstream | 0x2000; // add one startbit to stream
              }
            else
              bitstream = bitstream | 0x3000; // add two startbits to stream
            for(byte x=2;x<=14;x++)
              {
                c++;
                bitstate = (bitstream >> (14-x)) & 0x1;
                if (bitstate != prevstate) // bit toggle means long pulse
                  RawSignal.Pulses[c] = 2*PLUGIN_094_PHILIPSRC5_PULSELENGTH/RawSignal.Multiply; 
                else
                  {
                    RawSignal.Pulses[c] = PLUGIN_094_PHILIPSRC5_PULSELENGTH/RawSignal.Multiply;
                    c++;
                    RawSignal.Pulses[c] = PLUGIN_094_PHILIPSRC5_PULSELENGTH/RawSignal.Multiply;
                  }
                prevstate = bitstate;
              }
            if (c%2 == 0) // if ending with a low pulse, add one high pulse to finalize stream
              {
                c++;
                RawSignal.Pulses[c] = 900/RawSignal.Multiply;
              }
            RawSignal.Pulses[c+1]=0;
            RawSignal.Number=c+1;
          }

        RawSendIR();
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

        if(strcasecmp(str,PLUGIN_094_EVENT)==0)
          event->Type=NODO_TYPE_PLUGIN_EVENT;

        if(strcasecmp(str,PLUGIN_094_COMMAND)==0)
          event->Type=NODO_TYPE_PLUGIN_COMMAND;
        
        if(event->Type)
          {
          if(GetArgv(string,str,2))
            {
              event->Par1=str2int(str);
              if(GetArgv(string,str,3))
               {
                 event->Par2=str2int(str);
                 success=true;
               }
            }
            event->Command = PLUGIN_ID;
          }
        }
      free(str);
      break;
      }

    case PLUGIN_MMI_OUT:
      {
      if(event->Type==NODO_TYPE_PLUGIN_EVENT)
        strcpy(string,PLUGIN_094_EVENT);

      if(event->Type==NODO_TYPE_PLUGIN_COMMAND)
        strcpy(string,PLUGIN_094_COMMAND);

      strcat(string," ");
      strcat(string,int2str(event->Par1)); 
      strcat(string,",");
      strcat(string,int2strhex(event->Par2)); 

      break;
      }
    #endif //MMI
    }      
  return success;
  }
