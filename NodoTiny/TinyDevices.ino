/*********************************************************************************************\
 * Readout Dallas DS18B20 module
\*********************************************************************************************/
#ifdef DALLAS
// macro's
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM asm("r30")
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+2)) |= (mask))

// interval var's
IO_REG_TYPE bitmask;
volatile IO_REG_TYPE *baseReg;

float ReadDallas(void)
{
      int DSTemp;                           // Temperature in 16-bit Dallas format.
      byte ScratchPad[12];                  // Scratchpad buffer Dallas sensor.
      
      bitmask = PIN_TO_BITMASK(DALLAS_PIN);
      baseReg = PIN_TO_BASEREG(DALLAS_PIN);
      
      boolean present=DS_reset();DS_write(0xCC); DS_write(0x44);
              
      if(present)
        {
        delay(1000);     // time needed to read temperature
    
        DS_reset(); DS_write(0xCC); DS_write(0xBE);
    
        for (byte i = 0; i < 9; i++)            // copy 8 bytes
          ScratchPad[i] = DS_read();

        DSTemp = (ScratchPad[1] << 8) + ScratchPad[0];  
    
        return (float)(DSTemp)*0.0625;
        }
}

uint8_t DS_read(void)
{
  uint8_t bitMask;
  uint8_t r = 0;

  for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
	if ( DS_read_bit()) r |= bitMask;
    }
  return r;
}

uint8_t DS_read_bit(void)
{
  IO_REG_TYPE mask=bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
  uint8_t r;

  noInterrupts();
  DIRECT_MODE_OUTPUT(reg, mask);
  DIRECT_WRITE_LOW(reg, mask);
  delayMicroseconds(3);
  DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
  delayMicroseconds(10);
  r = DIRECT_READ(reg, mask);
  interrupts();
  delayMicroseconds(53);
  return r;
}

void DS_write(uint8_t v)
{
  uint8_t bitMask;
  for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
      DS_write_bit( (bitMask & v)?1:0);
    }
}

void DS_write_bit(uint8_t v)
{
  IO_REG_TYPE mask=bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

  if (v & 1)
    {
      noInterrupts();
      DIRECT_WRITE_LOW(reg, mask);
      DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
      delayMicroseconds(10);
      DIRECT_WRITE_HIGH(reg, mask);	// drive output high
      interrupts();
      delayMicroseconds(55);
    }
  else
    {
      noInterrupts();
      DIRECT_WRITE_LOW(reg, mask);
      DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
      delayMicroseconds(65);
      DIRECT_WRITE_HIGH(reg, mask);	// drive output high
      interrupts();
      delayMicroseconds(5);
    }
}

uint8_t DS_reset()
{
  IO_REG_TYPE mask = bitmask;
  volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
  uint8_t r;
  uint8_t retries = 125;

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  interrupts();

  // wait until the wire is high... just in case
  do
    {
      if (--retries == 0) return 0;
      delayMicroseconds(2);
    } while ( !DIRECT_READ(reg, mask));

  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
  interrupts();
  delayMicroseconds(500);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);	// allow it to float
  delayMicroseconds(80);
  r = !DIRECT_READ(reg, mask);
  interrupts();
  delayMicroseconds(420);
  return r;
}
#endif


/*********************************************************************************************\
 * Readout DHT11 module
\*********************************************************************************************/
#ifdef DHT11
byte dhtread(byte &temperature, byte &humidity)
{
  byte dht11_dat[5];
  byte dht11_in;
  byte i;

  pinMode(DHT11_PIN,OUTPUT);
  
  // DHT11 start condition, pull-down i/o pin for 18ms
  digitalWrite(DHT11_PIN,LOW);               // Pull low
  delay(18);
  digitalWrite(DHT11_PIN,HIGH);              // Pull high
  delayMicroseconds(40);
  pinMode(DHT11_PIN,INPUT);                  // change pin to input
  delayMicroseconds(40);

  dht11_in = digitalRead(DHT11_PIN);
  if(dht11_in) return 255;
  
  delayMicroseconds(80);
  dht11_in = digitalRead(DHT11_PIN);
  if(!dht11_in) return 254;
  
  delayMicroseconds(40);                     // now ready for data reception
  for (i=0; i<5; i++)
    dht11_dat[i] = read_dht11_dat();
  byte dht11_check_sum = dht11_dat[0]+dht11_dat[1]+dht11_dat[2]+dht11_dat[3];// check check_sum
  if(dht11_dat[4]!= dht11_check_sum) return 253;

  temperature = dht11_dat[2];
  humidity = dht11_dat[0];
  return 0;
}

/*********************************************************************************************\
 * DHT11 sub to get an 8 bit value from the receiving bitstream
\*********************************************************************************************/
byte read_dht11_dat()
{
  byte i = 0;
  byte result=0;
  for(i=0; i< 8; i++)
  {
    while(!digitalRead(DHT11_PIN));  // wait for 50us
    delayMicroseconds(30);
    if(digitalRead(DHT11_PIN)) 
      result |=(1<<(7-i));
    while(digitalRead(DHT11_PIN));  // wait '1' finish
  }
  return result;
}
#endif

