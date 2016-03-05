//#######################################################################################################
//##################################### Plugin-96 Xiron  ################################################
//#######################################################################################################

/*********************************************************************************************\
 * This protocol provides reception/decoding of Xiron weatherstation outdoorsensors
 * 
 * Author             : Martinus van den Broek
 * Support            : None!
 * Date               : 29 Sep 2014 (debugging code removed)
 * Version            : 0.2
 * Compatibility      : R744
 * Syntax             : "XironV1 <Par1:Sensor ID>, <Par2:Basis Variabele>"
 *********************************************************************************************
 * Technical data:
 * Decodes signals from Xiron Weatherstation outdoor unit
 * Message Format: (84 pulses, 40 bits of data)
 * AAAAAAAA BB CC DDDDDDDDDDDD EEEE FFFFFFFF GGGG
 * ID       ?? Ch Temperature  ?    Humidity ?
 * A = ID (code changes after battery replacement)
 * B = ?
 * C = Channel (1,2,3)
 * D = Temperature (12 bit value)
 * E = ?
 * F = Humidity
 * G = ?
 *
 * RF Message is repeated 5 times, delay between messages is approx 3600 uSec
 * Default RFLink release cannot receive this because this delay is too short!!!
 * Workaround:
 *   Ajdust SIGNAL_TIMEOUT from default 7 mSec to 3 mSec
 *
 \*********************************************************************************************/

#define XIRON_PULSECOUNT_SINGLE 84 
#define XIRCON_PULSEMID  1250/RAWSIGNAL_SAMPLE_RATE

boolean Plugin_096(byte function, char *string)
{
  boolean success=false;
      if (RawSignal.Number != XIRON_PULSECOUNT_SINGLE) return false;

      unsigned long bitstream1=0;
      unsigned long bitstream2=0;
      byte rc=0;
      int temperature=0;
      byte humidity=0;
      byte basevar=0;
      byte offset=2;
      if (RawSignal.Number == XIRON_PULSECOUNT_SINGLE)
        offset=0;
      char buffer[14]=""; 

      // get first 28 bits
      for(byte x=(2+offset); x<=(56+offset); x=x+2) if(RawSignal.Pulses[x] > XIRCON_PULSEMID) bitstream1 = (bitstream1 << 1) | 0x1; 
      else bitstream1 = (bitstream1 << 1);
      // get remaining 12 bits
      for(byte x=(58+offset); x<=(80+offset); x=x+2) if(RawSignal.Pulses[x] > XIRCON_PULSEMID) bitstream2 = (bitstream2 << 1) | 0x1; 
      else bitstream2 = (bitstream2 << 1);

      rc = (bitstream1 >> 20) & 0xff;
// fix
byte channel=0;
byte bat=1;

      temperature = ((bitstream1 >> 4) & 0x3ff);
      if (temperature > 3000) {
         temperature=4096-temperature;              // fix for minus temperatures
         temperature=temperature | 0x8000;          // turn highest bit on for minus values
      }      
      humidity = ((bitstream2 >> 4) & 0xff);

/*
Serial.print(RepeatingTimer);
Serial.print(" - ");
Serial.print(millis());
Serial.print(" - ");
Serial.print(SignalCRC);
Serial.print(" - ");
Serial.println(bitstream1 + bitstream2);
*/

      //==================================================================================
      // Prevent repeating signals from showing up
      //==================================================================================
      if( (millis() < RepeatingTimer ) && SignalCRC == bitstream1 + bitstream2 )
        return true;
      else
        SignalCRC = bitstream1 + bitstream2;

      // Output
      // ----------------------------------
      sprintf(buffer, "20;%02X;", PKSequenceNumber++); // Node and packet number 
      Serial.print( buffer );
      // ----------------------------------
      Serial.print("Xiron;");                    // Label
      sprintf(buffer, "ID=%02x%02x;", rc, channel); // ID (rc+channel)
      Serial.print( buffer );
      sprintf(buffer, "TEMP=%04x;", temperature);     
      Serial.print( buffer );
      sprintf(buffer, "HUM=%02d;", humidity);     
      Serial.print( buffer );
      if (bat==0) {                                 // battery status
         Serial.print("BAT=LOW;");
      } else {
         Serial.print("BAT=OK;");
      }
      Serial.println();

      RawSignal.Number=0;
      success = true;

  return success;
}
