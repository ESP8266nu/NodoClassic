// Code based on "NewRemoteSwitch library" made by Randy Simons http://randysimons.nl/

void NewKaku_interruptHandler()
{
  static unsigned short receivedBit;		// Contains "bit" currently receiving
  static struct NewRemoteCode receivedCode;		// Contains received code
  static struct NewRemoteCode previousCode;		// Contains previous received code
  static unsigned short repeats = 0;		// The number of times the an identical code is received in a row.
  static unsigned long edgeTimeStamp[3] = {    0,   };	// Timestamp of edges
  static unsigned int min1Period, max1Period, min5Period, max5Period;
  static bool skip;

  // Filter out too short pulses. This method works as a low pass filter.
  edgeTimeStamp[1] = edgeTimeStamp[2];
  edgeTimeStamp[2] = micros();

  if (_state >= 0 && edgeTimeStamp[2]-edgeTimeStamp[1] < min1Period) {
    // Last edge was too short.
    _state = -1;
    return;
  }

  unsigned int duration = edgeTimeStamp[1] - edgeTimeStamp[0];
  edgeTimeStamp[0] = edgeTimeStamp[1];

  // Note that if state>=0, duration is always >= 1 period.

  if (_state == -1) {
    // wait for the long low part of a start bit.
    // Startbit: 1T high, 40T low
    // By default 1T is 260us, but for maximum compatiblity go as low as 120us
    if (duration > 4800) { // =40*120us, minimal time between two edges before decoding starts.
      // Sync signal received.. Preparing for decoding
      repeats = 0;

      receivedCode.period = duration / 40; // Measured signal is 40T, so 1T (period) is measured signal / 40.

      // Allow for large error-margin. ElCheapo-hardware :(
      min1Period = receivedCode.period * 3 / 10; // Lower limit for 1 period is 0.3 times measured period; high signals can "linger" a bit sometimes, making low signals quite short.
      max1Period = receivedCode.period * 3; // Upper limit for 1 period is 3 times measured period
      min5Period = receivedCode.period * 3; // Lower limit for 5 periods is 3 times measured period
      max5Period = receivedCode.period * 8; // Upper limit for 5 periods is 8 times measured period
    }
    else {
      return;
    }
  } 
  else if (_state == 0) { // Verify start bit part 1 of 2
    // Duration must be ~1T
    if (duration > max1Period) {
      _state = -1;
      return;
    }
    // Start-bit passed. Do some clean-up.
    receivedCode.address = receivedCode.unit = receivedCode.dimLevel = 0;
  } 
  else if (_state == 1) { // Verify start bit part 2 of 2
    // Duration must be ~10.44T
    if (duration < 7 * receivedCode.period || duration > 15 * receivedCode.period) {
      _state = -1;
      return;
    }
  } 
  else if (_state < 146) { // state 146 is first edge of stop-sequence. All bits before that adhere to default protocol, with exception of dim-bit
    receivedBit <<= 1;

    // One bit consists out of 4 bit parts.
    // bit part durations can ONLY be 1 or 5 periods.
    if (duration <= max1Period) {
      receivedBit &= B1110; // Clear LSB of receivedBit
    }
    else if (duration >= min5Period && duration <= max5Period) {
      receivedBit |= B1; // Set LSB of receivedBit
    }
    else { // Otherwise the entire sequence is invalid
      _state = -1;
      return;
    }

    if (_state % 4 == 1) { // Last bit part? Note: this is the short version of "if ( (_state-2) % 4 == 3 )"
      // There are 3 valid options for receivedBit:
      // 0, indicated by short short short long == B0001.
      // 1, short long shot short == B0100.
      // dim, short shot short shot == B0000.
      // Everything else: inconsistent data, trash the whole sequence.


      if (_state < 106) {
        // States 2 - 105 are address bit states

        receivedCode.address <<= 1;

        // Decode bit. Only 4 LSB's of receivedBit are used; trim the rest.
        switch (receivedBit & B1111) {
        case B0001: // Bit "0" received.
          // receivedCode.address |= 0; But let's not do that, as it is wasteful.
          break;
        case B0100: // Bit "1" received.
          receivedCode.address |= 1;
          break;
        default: // Bit was invalid. Abort.
          _state = -1;
          return;
        }
      } 
      else if (_state < 110) {
        // States 106 - 109 are group bit states.
        switch (receivedBit & B1111) {
        case B0001: // Bit "0" received.
          receivedCode.groupBit = false;
          break;
        case B0100: // Bit "1" received.
          receivedCode.groupBit = true;
          break;
        default: // Bit was invalid. Abort.
          _state = -1;
          return;
        }
      } 
      else if (_state < 114) {
        // States 110 - 113 are switch bit states.
        switch (receivedBit & B1111) {
        case B0001: // Bit "0" received.
          receivedCode.switchType = 0;
          break;
        case B0100: // Bit "1" received.
          receivedCode.switchType = 1;
          break;
        case B0000: // Bit "dim" receivd.
          receivedCode.switchType = 2;
          break;
        default: // Bit was invalid. Abort.
          _state = -1;
          return;
        }
      } 
      else if (_state < 130){
        // States 114 - 129 are unit bit states.
        receivedCode.unit <<= 1;

        // Decode bit.
        switch (receivedBit & B1111) {
        case B0001: // Bit "0" received.
          // receivedCode.unit |= 0; But let's not do that, as it is wasteful.
          break;
        case B0100: // Bit "1" received.
          receivedCode.unit |= 1;
          break;
        default: // Bit was invalid. Abort.
          _state = -1;
          return;
        }

        // Only if there is a dim-action chosen the dim-bits are present in the signal.
        // Thus, if switchType is on or off, skip these dim-bits.
        if (_state == 129 && receivedCode.switchType != 2) {
          _state = 145; // 4 bits x 4 states = 16 states to skip.
        }
      } 
      else if (_state < 146){
        // States 130 - 145 are dim bit states. Note that these are skipped unless switchType == 2.
        receivedCode.dimLevel <<= 1;
        // Decode bit.
        switch (receivedBit) {
        case B0001: // Bit "0" received.
          // receivedCode.dimLevel |= 0; But let's not do that, as it is wasteful.
          break;
        case B0100: // Bit "1" receivd.
          receivedCode.dimLevel |= 1;
          break;
        //default: // Bit was invalid. Abort.
          //_state = -1;
          //return;
        }
      }
    }
  } 
  else if (_state == 146) { // Verify stop bit part 1 of 2
    // Duration must be ~1T
    if (duration < min1Period || duration > max1Period) {
      _state = -1;
      return;
    }
  } 
  else if (_state == 147) { // Verify stop bit part 2 of 2
    // Duration must be ~40T
    if (duration < 20 * receivedCode.period || duration > 80 * receivedCode.period) {
      _state = -1;
      return;
    }

    // receivedCode is a valid code!

    if (
    receivedCode.address != previousCode.address ||
      receivedCode.unit != previousCode.unit ||
      receivedCode.dimLevel != previousCode.dimLevel ||
      receivedCode.groupBit != previousCode.groupBit ||
      receivedCode.switchType != previousCode.switchType
      ) { // memcmp isn't deemed safe
      repeats=0;
      previousCode = receivedCode;
    }

    repeats++;

    if (repeats>=_minRepeats)
    {
      NewKaku = receivedCode;
      _state=-1;
      return;
    }

    // Reset for next round
    _state=0; // no need to wait for another sync-bit!
    return;
  }

  _state++;
  return;
}

