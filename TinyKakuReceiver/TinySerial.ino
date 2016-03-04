#define _SS_MAX_RX_BUFF         16 // RX buffer size
// Intrabit timing slightly adjusted, because program code is reduced
#define RX_DELAY_CENTERING      50
#define RX_DELAY_INTRABIT       120 // original 114
#define RX_DELAY_STOPBIT        114
#define TX_DELAY                112
#define XMIT_START_ADJUSTMENT     4

uint8_t _receivePin;
uint8_t _receiveBitMask;
volatile uint8_t *_receivePortRegister;
char _receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t _receive_buffer_tail = 0;
volatile uint8_t _receive_buffer_head = 0;

uint8_t _transmitBitMask;
volatile uint8_t *_transmitPortRegister;

void SoftwareSerial_print(char* string)
{
  byte x=0;
  while (string[x] != 0)
  {
    SoftwareSerial_write(string[x]);
    x++;
  }
}

void SoftwareSerial_println(char* string)
{
  SoftwareSerial_print(string);
  SoftwareSerial_write(13);
  SoftwareSerial_write(10);
}

void SoftwareSerial_init(byte receivePin, byte transmitPin)
{
  SoftwareSerial_setTX(transmitPin);
  SoftwareSerial_setRX(receivePin);
  if (digitalPinToPCICR(_receivePin))
  {
    *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
  }
  SoftwareSerial_tunedDelay(TX_DELAY); // if we were low this establishes the end
}

inline void SoftwareSerial_tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

void SoftwareSerial_recv()
{
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (!SoftwareSerial_rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    SoftwareSerial_tunedDelay(RX_DELAY_CENTERING);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      SoftwareSerial_tunedDelay(RX_DELAY_INTRABIT);
      uint8_t noti = ~i;
      if (SoftwareSerial_rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    SoftwareSerial_tunedDelay(RX_DELAY_STOPBIT);

    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
  }
}

void SoftwareSerial_tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}

uint8_t SoftwareSerial_rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

ISR(PCINT0_vect)
{
  #ifdef NEWKAKUIRQ
    // RF interrupt handling
    byte portstate= *_receivePortRegister & _BV(RF_RX_PIN);
    if (RFState != portstate)
      {
       RFState = portstate;
       NewKaku_interruptHandler();
      }
  #endif
  
  // Software Serial interrupt handling
  SoftwareSerial_recv();
  #ifdef DIGITAL
  byte portstate= *_receivePortRegister & 0xf;
  if ((portstate & 0x8) == 0)
    if (pulsstate == 0)
      {
        pulsstate=1;
        pulscounter++;
      }
    else
      {
        pulsstate=0;
      }
  #endif
}

void SoftwareSerial_setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial_setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

void SoftwareSerial_end()
{
  SoftwareSerial = false;
  if (digitalPinToPCMSK(_receivePin))
    *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
}

int SoftwareSerial_read()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial_available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial_write(uint8_t b)
{
  
  if (!SoftwareSerial) return 0;
  
  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  SoftwareSerial_tx_pin_write(LOW);
  SoftwareSerial_tunedDelay(TX_DELAY + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      SoftwareSerial_tx_pin_write(HIGH); // send 1
    else
      SoftwareSerial_tx_pin_write(LOW); // send 0
  
    SoftwareSerial_tunedDelay(TX_DELAY);
  }
  SoftwareSerial_tx_pin_write(HIGH); // restore pin to natural state

  SREG = oldSREG; // turn interrupts back on
  SoftwareSerial_tunedDelay(TX_DELAY);
  
  return 1;
}

int SoftwareSerial_peek()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

void SoftwareSerial_flush()
{
  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  SREG = oldSREG;
}
