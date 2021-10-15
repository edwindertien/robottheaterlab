//*****************************************************************************
// DimmerBoardTimeController project
// RS485 control board software for Dancing Man or other time based control
//
// The dimmer cards use ATmega88 controllers at 16 MHz (early choice, economical)
// They can be programmed using arduino hardware defintion for Ottantotto
// programming using an AVR-ISP such as AVR-ISP MKII or USBasp unit
//
// The baudrate can be set to 57600 but also works at 1000000 selected using dip switch 4 (OFF: 57600, ON: 1000000)
// mode (dimmer or time) can be set using dip switch 3 (OFF: dimmer, OND: time)
// address [0..3] is set with switches 1 and 2
//
// This software can be put into a Dimmerboard. It will control each of the
// eight outputs as follows:
// + Normally the output is turned off.
// + When a message (see below) is sent, the respective channel will be on for
//     a short period of time, being value*10 ms. Because this has been designed
//     to work with MIDI signals, it is considered that the value lies between
//     0 and 127 (including).
// + If a new command is sent while the output is still on, the old command
//   is discarded and the new one will be executed (and the output will be
//   on for value*10 ms, counting from the time the new command was sent)
// + Value 0 will immediately turn off the output (this is a direct consequence
//   of the previous rule).
// + Value 127 is special: it will leave the output on until a new command is sent.
//
// This software expects a dynamixel-compatible protocol. A message is composed as follows:
// 0xFF   'header'
// 0xFF
// 0x??   'ID'         : board address (NOT <<4, as in earlier versions)
// 0x04   'Data length': number of bytes to follow
// 0x03   'Write'      : command
// 0x??   'Parameter 1': channel; 0..7: channel, >7: broadcast (all channels of the dimmer board)
// 0x??   'Parameter 2': value (turn-on time, in 10ms/increment (so value=31 gives a pulse of 310 ms)
// 0x??   'Checksum'   : ~(ID + 0x04 + 0x03 + channel + value) (~=bitwise NOT)
//
// Example: FF FF 01 04 03 05 08 DA will turn on output 5 on board 04 for 80 ms.
//
// address and mode jumpers are only checked at startup.
//
// Pinning Atmel Controller     |Arduino
// PD0 Serial RXD   (RS485)     |0
// PD1 Serial TXD   (RS485)     |1
// PD2 Send/Receive (RS485)     |2
// PD3 LED D1                   |3
// PD4 Adr 1                    |4
// PD5 Adr 2                    |5
// PD6 Adr 3                    |6
// PD7 Adr 4                    |7
// PB0 channel0                 |8
// PB1 channel1                 |9
// PB2 channel2                 |10
// PB3 channel3                 |11
// PB4 LED D2                   |12
// PB5 LED D3                   |13 (LED)
// PC0 channel4                 |A0
// PC1 channel5                 |A1
// PC2 channel6                 |A2
// PC3 channel7                 |A3
// PC4 I2C SDA                  |A4 (SDA)
// PC5 I2C SCL                  |A5 (SCL)
//*****************************************************************************

// mode 0: time control (valves)
// mode 1: pwm  control (LED dimmer)
int mode = 0; // but changed in the void setup()

// Channel definitions:
#define FET0PORT PORTB
#define FET1PORT PORTB
#define FET2PORT PORTB
#define FET3PORT PORTB
#define FET4PORT PORTC
#define FET5PORT PORTC
#define FET6PORT PORTC
#define FET7PORT PORTC
#define FET0PIN 0
#define FET1PIN 1
#define FET2PIN 2
#define FET3PIN 3
#define FET4PIN 0
#define FET5PIN 1
#define FET6PIN 2
#define FET7PIN 3
///////////////////////////////////////////////////////////////////////////////////////////
// Usefull utility functions to work with ports outside the standard Arduino functions
// outb(), inb(), inw(), outw(), BV(), sbi(), cbi()
// as inheritance from WinAVR style programming
#define outb(addr, data)  addr = (data)
#define inb(addr)     (addr)
#define outw(addr, data)  addr = (data)
#define inw(addr)     (addr)
#define BV(bit)     (1<<(bit))
#define cbi(reg,bit)  reg &= ~(BV(bit))
#define sbi(reg,bit)  reg |= (BV(bit))
#define limit(number,boundary) ( ((number)>(boundary))?(boundary):( ((number)<(-boundary))?(-boundary):(number)) )
#define limit2(number,min,max) ( ((number)>(max))?(max):( ((number)<(min))?(min):(number)) )
#define toggle(port,nr) if(port & 1<<nr) cbi(port,nr); else sbi(port,nr)
#define FALSE (0)
#define TRUE (1)
///////////////////////////////////////////////////////////////////////////////////////////
//
#define DYNAMIXEL_BUFFER_SIZE (128) // make sure it never overflows.
#define N_CHANNELS (8) // number of LED/PWM/Valve channels

/* dim curve taken from http://www.kasperkamperman.com */
const unsigned char dim_curve[] PROGMEM = {
  0,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
  3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,
  4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,
  6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
  8,   8,   9,   9,   9,   9,   9,   9,   10,  10,  10,  10,  10,  11,  11,  11,
  11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  14,  15,
  15,  15,  16,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,
  20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  25,  26,  26,
  27,  27,  28,  28,  29,  29,  30,  30,  31,  32,  32,  33,  33,  34,  35,  35,
  36,  36,  37,  38,  38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,
  48,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,
  63,  64,  65,  66,  68,  69,  70,  71,  73,  74,  75,  76,  78,  79,  81,  82,
  83,  85,  86,  88,  90,  91,  93,  94,  96,  98,  99,  101, 103, 105, 107, 109,
  110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
  146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
  193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

#define LED_GREEN 13
#define LED_YELLOW 3
#define LED_RED 12
/* PWM_FREQ
  Ideally, choose a multiple of ONE_OVER_TIME_UNIT; this will increas accuracy (because we
  are doing an integer division PWM_FREQ/ONE_OVER_TIME_UNIT somewhere).
*/
#define PWM_FREQ (200)

/* PWM_RESOLUTION
  How many steps do we have in the pwm?
  Higher number: more resolution in intensity.
  Lower number: faster PWM possible.
  Don't make this one larger than 128, because that will overflow
  our signed ints! (128 itself is allowed, though)
*/
#define PWM_RESOLUTION (125)

/* ONE_OVER_TIME_UNIT
  how many 'time units' go in 1 second? i.e. if we give 10 here, the variable _time in
  ProcessNewCommand denotes the rise time in 10'ths of a seconds. If we put 1000 here, it
  denotes the rise time in ms.
*/
#define ONE_OVER_TIME_UNIT (10)

/* This struct contains everything we need to know about a channel. Using this, we can,
  each time step, calculate the new current_intensity. We use signed ints here as
  'fixed point' variables so that we have a higher accuracy/resolution in the ramp-up. */
typedef struct{
  //////////////////////////////////////
  // 'current state'; The two variables below are needed to keep the current
  // state of the PWM channel

  /* current_intensity: We use the highest byte as the desired duty-cycle,
     i.e. DutyCycle = highbyte(current_intensity) / RESOLUTION. */
  signed int current_intensity;

  /* counter: Each channel has its own counter. The value of counter is
     always compard with current_intensity; if counter < current_intensity,
     the LED is turned on. The reaason why we use different counters for
     each channel is that it makes that not PWM-channels are reset at the
     same time. This minimizes calculation jitter (we don't need to re-calculate
     the current_intensity for all the channels during one pulse), and it also
     distributes the current because not all channels are on at the same time.
     counter runs from 0 till (PWM_RESOLUTION-1).
  */
  unsigned char counter;


  /////////////////////////////////////
  // 'Command memory'; the two variables below store stuff that is needed
  // to process the ramp-command
  /* final_intensity:  We use the highest byte as the desired duty-cycle,
     i.e. DutyCycle = highbyte(current_intensity) / RESOLUTION. */
  signed int final_intensity;

  /* increment: How much the current_intensity should be incremented each PWM_FREQ 'th
     second. May be negative. */
  signed int increment;

  /* used to store lookup-table corrected intensity value */
  unsigned char corrected_intensity;
} LedChannelInfo;

typedef enum {
  WaitingForFirstHeaderByte,
  WaitingForSecondHeaderByte,
  WaitingForIDByte,
  WaitingForDataLengthByte,
  WaitingForRestOfMessage
} e_receive_state;

e_receive_state ReceiveState = WaitingForFirstHeaderByte;

/* This struct contains everything we need to know about a channel. Using this, we can,
  each time step, calculate the new current_intensity. We use signed ints here as
  'fixed point' variables so that we have a higher accuracy/resolution in the ramp-up. */
typedef struct {
  //////////////////////////////////////
  // 'current state'; The two variables below are needed to keep the current
  // state of the time controlled channel

  /* duration: the number of samples (typically with a sample time of 10 ms = 100 Hz)
     that the channel should be on. This should be set by the ProcessNewCommand function. */
  int duration;

  /* timeOn: The number of samples that the channel has been on. This is read&set by
     the update function. */
  int timeOn;

  /* newCommand: Is set to true when a new command is issued. This allows the CommandUpdate()
     to properly handle new commands (sometimes, it can be unclear that a new command was
     issued). */
  unsigned char newCommand; // actually a bool

  /* on: Boolean that tells if the FET is on (if so, CommandUpdate() should process it) */
  unsigned char on; // actually a bool
} ChannelInfo;

ChannelInfo channel[N_CHANNELS]; // The information
LedChannelInfo ledChannel[N_CHANNELS]; // The information

//////////////////////////////////////////////////
// global variables
unsigned char c;
unsigned char addressBuffer = 0;
unsigned char DataLengthBuffer = 0;
unsigned char NumberOfDataBytesReceived = 0;
unsigned char Data[DYNAMIXEL_BUFFER_SIZE];
unsigned char checksumBuffer = 0;
unsigned char boardAddress;

void setup() {
  pinMode(LED_YELLOW,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  
  outb(DDRB, 0x3F);
  outb(PORTB, 0x3F); // Output fets off

  outb(DDRC, 0x0F); // General purpose input / ADC
  outb(PORTC, 0x0F);

  outb(DDRD, 0x0C); // only SR as output -- and LED yellow
  outb(PORTD, 0xF8);

  sbi(PORTD, 3);
  cbi(PORTD, 2); // set RS485 to listen
  sbi(PORTD, 0); //pullup?

  //
  // now checking dip switches:
  boardAddress = digitalRead(4) + 2 * digitalRead(5);
  mode = digitalRead(6);
  if (digitalRead(7)) Serial.begin(57600); else Serial.begin(1000000);
  //
  if (mode)LedControlInit(); else ControlInit();

  TCCR2A = 0; // no ocr or pwm
  TCCR2B = 4; //1<<CS22 | 0<<CS21 | 0<<CS20 -- for clock div 64
  TIMSK2 = 1 << TOIE2; // enable timer 1 interrupt
  TCNT2 = 0xce; // 5 kHz
}

void loop() {
  if (Serial.available() > 0) {
    c = Serial.read();
    switch (ReceiveState)
    {
      case WaitingForFirstHeaderByte:
        if (c == 0xFF) // first byte of header
        {
          ReceiveState = WaitingForSecondHeaderByte;
        }
        break;
      case WaitingForSecondHeaderByte:
        if (c == 0xFF)
        {
          if(digitalRead(LED_YELLOW)) digitalWrite(LED_YELLOW,LOW); else digitalWrite(LED_YELLOW,HIGH);
         // toggle(PORTB, 5); // yellow LED to show bus communication
          ReceiveState = WaitingForIDByte;
        } else ReceiveState = WaitingForFirstHeaderByte;
        break;
      case WaitingForIDByte:
        // store byte
        addressBuffer = c;
        checksumBuffer = c;
        ReceiveState = WaitingForDataLengthByte;
        break;
      case WaitingForDataLengthByte:
        DataLengthBuffer = c;
        NumberOfDataBytesReceived = 0;
        checksumBuffer += c;
        ReceiveState = WaitingForRestOfMessage;
        break;
      case WaitingForRestOfMessage:
        if (NumberOfDataBytesReceived < DYNAMIXEL_BUFFER_SIZE)
        {
          Data[NumberOfDataBytesReceived] = c;
        } // otherwise, we would have buffer overflow
        checksumBuffer += c;
        NumberOfDataBytesReceived++;
        if (NumberOfDataBytesReceived == DataLengthBuffer)
        {
          // We're finished receiving the message, so process it
          if (addressBuffer == boardAddress)
          {
            // Then it is meant for this board
            // Check if the data is correct by means of checksum.
            // Also, if we had a buffer overflow, we know for sure that the
            // data is invalid (because we only stored part of it, so in that case we
            // should not process it either. It would have been a faulty command anyway).
            if ( (checksumBuffer & 0xFF) == 0xFF && NumberOfDataBytesReceived < DYNAMIXEL_BUFFER_SIZE)
            {
              // Then the data is correct
              if(digitalRead(LED_GREEN)) digitalWrite(LED_GREEN,LOW); else digitalWrite(LED_GREEN,HIGH);
              ProcessData(DataLengthBuffer, Data);
            } else
            {
              //toggle(PORTB, 4); // checksum error
              if(digitalRead(LED_RED)) digitalWrite(LED_RED,LOW); else digitalWrite(LED_RED,HIGH);
            }
          }

          ReceiveState = WaitingForFirstHeaderByte;
        }
        break;
    }
  }
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = 0xf6; // reload timer at 5kHz
  static unsigned char counter = 0;
  if (mode) LedControlUpdate();
  // 100 Hz loop
  else {
    if ( ++counter == 250)
    {
      counter = 0;
      ControlUpdate();
    }
  }
}

/*********************************************************
  Checks what data is in the message and acts accordingly.
*/
void ProcessData(unsigned char DataLengthBuffer, unsigned char* Data)
{
  // We only support one type of message, being a 'write' message (command 0x03) for a
  // channel to turn itself on for some time.
  // DataLengthBuffer should be 4.
  // Command  is in Data[0];
  // Channel  is in Data[1];
  // Time     is in Data[2];
  // Checksum is in Data[3]; (and it is valid).

  if (DataLengthBuffer != 0x04) return; // Todo: should we light a led to indicate this??
  if (Data[0] != 0x03) return;

  if (mode) ProcessDimmerCommand(Data[1], Data[2], 1);
  else ProcessNewCommand(Data[1], Data[2]);
}
/******************************************************************
  void ControlInit(void)

  Initializes the ChannelInfo variables
*/
void ControlInit(void)
{
  int i;
  ChannelInfo *c;
  for (i = 0; i < N_CHANNELS; i++)
  {
    c = &(channel[i]);
    c->duration = 0;
    c->timeOn = 0;
  }
}


/******************************************************************
  void ProcessNewCommand( unsigned char _channel, unsigned char _time)

  Sets up the system such that this command is handled.
  - _channel should be 0..(N_CHANNELS-1).
    Any number higher than that will cause ALL channels to do
    this command. This allows for a general OFF. As a convention,
    use channel = 0x0F = 15 for this.
  - _time is in samples. If the function ControlUpdate() is called
    100 times per second (ie, ever 10 ms), the channel will be turned
    on for _time*10 ms.
    Exception: a _time of 127 will turn the FET on forever (until a new command is issued)

    If a new command is sent while the channel was still on, the old
    command is canceled and the new command is executed (so, from that time
    on, the FET will be turned on for _time * 10 ms).
    Use a _time = 0 to instantaneously turn off the FET.
*/
void ProcessNewCommand( unsigned char _channel, unsigned char _time)
{
  if (_channel < N_CHANNELS)
  {
    ChannelInfo *c;
    c = &(channel[_channel]);

    c->duration = _time;
    c->newCommand = TRUE;
  } else
  {

    // Then it is a broadcast
    for (int i = 0; i < N_CHANNELS; i++) ProcessNewCommand(i, _time);
  }
}



/******************************************************************
  void ControlUpdate(void)

  This function should be called regularly, for example 100 times per second.
*/
void ControlUpdate(void)
{
  unsigned char i;
  ChannelInfo *c;

  for (i = 0; i < N_CHANNELS; i++)
  {
    c = &(channel[i]);
    if ( c -> newCommand ) // Then we should instantiate a command
    {
      c -> newCommand = FALSE;

      if (c->duration == 0) // special case, we need to turn off the FET
      {
        TurnOff(i);
        c -> on = FALSE;
      } else
      {
        // Turn it on
        TurnOn(i);
        c -> timeOn = 0;
        c -> on = TRUE;
      }
    } else if (c -> on)
    {
      // Then the channel is on, so process it

      if (c->duration != 127) // if it _is_ 127, we should never turn off the led
      {
        if (++(c->timeOn) >= c->duration)
        {
          // Then turn it off
          TurnOff(i);
          c -> duration = 0; // Not really needed, but more beautiful
          c -> timeOn = 0;   // Not really needed, but more beautiful
          c -> on = FALSE;
        }
      }
    }
  }
}


/******************************************************************
  Turns on the specific channel
*/
void TurnOn(unsigned char channel)
{
  switch (channel)
  {
    case 0: cbi(FET0PORT, FET0PIN); break;
    case 1: cbi(FET1PORT, FET1PIN); break;
    case 2: cbi(FET2PORT, FET2PIN); break;
    case 3: cbi(FET3PORT, FET3PIN); break;
    case 4: cbi(FET4PORT, FET4PIN); break;
    case 5: cbi(FET5PORT, FET5PIN); break;
    case 6: cbi(FET6PORT, FET6PIN); break;
    case 7: cbi(FET7PORT, FET7PIN); break;
    default: break;// ERROR
  }
}

/******************************************************************
  Turns off the specific channel
*/
void TurnOff(unsigned char channel)
{
  switch (channel)
  {
    case 0: sbi(FET0PORT, FET0PIN); break;
    case 1: sbi(FET1PORT, FET1PIN); break;
    case 2: sbi(FET2PORT, FET2PIN); break;
    case 3: sbi(FET3PORT, FET3PIN); break;
    case 4: sbi(FET4PORT, FET4PIN); break;
    case 5: sbi(FET5PORT, FET5PIN); break;
    case 6: sbi(FET6PORT, FET6PIN); break;
    case 7: sbi(FET7PORT, FET7PIN); break;
    default: break;// ERROR
  }
}

/******************************************************************
  void LedControlInit(void)

  Initializes the ChannelInfo variables
*/
void LedControlInit(void)
{
  int i;
  LedChannelInfo *c;
  for (i = 0; i < N_CHANNELS; i++)
  {
    c = &(ledChannel[i]);
    c->current_intensity = 0;
    c->counter = i * (PWM_RESOLUTION / N_CHANNELS); // Make sure that the counters are all out-of-sync (see .h file for explanation)
    c->final_intensity = 0;
    c->increment = 0;
    c->corrected_intensity = 0;
  }
}


/******************************************************************
  void ProcessNewCommand( unsigned char _channel, unsigned char _intensity, unsigned char _time)

  Sets up the system such that this command is handled.
  - _channel should be 0..(N_CHANNELS-1).
    Any number higher than that will cause ALL channels to do
    this command. This allows for a general OFF. As a convention,
    use channel = 0x0F = 15 for this.
  - _intensity = 0 (off) .. 255 (fully on). Note that this is scaled
    down directly to the internal resolution, so the value you find
    in the ChannelInfo variable (final_intenisty) is different!
  - _time is in 1/ONE_OVER_TIME_UNIT th of seconds.
    So, if ONE_OVER_TIME_UNIT=10, _time denotes the rise time in tenths
    of a seconds; which means we have a maximum rise time of
    25.5 seconds. Use time=0 for instantaneous change.
    Note: maximum rise time is always 255/ONE_OVER_TIME_UNIT seconds.

*/
void ProcessDimmerCommand( unsigned char _channel, unsigned char _intensity, unsigned char _time)
{
  LedChannelInfo *c;
  int _final_intensity; // temporary variable
  int temp;

  if (_channel >= N_CHANNELS) // Then it is a command for all channels
  {
    int i;
    for (i = 0; i < N_CHANNELS; i++)
    {
      ProcessDimmerCommand(i, _intensity, _time);
    }
    return;
  }

  // If we're here, it is a command for one channel only
  c = &(ledChannel[_channel]);

  _final_intensity = _intensity << 7;

  /* About the increment:
    We need to go from current_intensity (cu) to final_intensity (fi) in
    time (t) tenth seconds -->
    increment-per-second = (fi-cu) / (t/ ONE_OVER_TIME_UNIT)  -->
    increment-per-PWM_FREQ = ((fi-cu) / (t/ONE_OVER_TIME_UNIT)) / PWM_FREQ) -->
    increment-per-PWM_FREQ = (fi-cu) / ( (PWM_FREQ/ONE_OVER_TIME_UNIT) * t)
    The order of calculation is important; the way described above will make sure that
    no overflow can occur in any intermediate result.

    Note that we do an integer division on purpose; we want an int as result...
    The error that we make here will result in a slightly longer rise
    (max 1/PWM_FREQ second) time, but no-one will notice that...
  */

  temp =  (( PWM_FREQ / ONE_OVER_TIME_UNIT) * _time); // We do this now already so that the interrupt disabling is as shortly as possible

  // Update (should be done with interrupts off)
  if (_time == 0)
  {
    cli();
    c->final_intensity = _final_intensity;
    c->increment = (_final_intensity  - c->current_intensity);
    sei();
  } else
  {
    cli();
    c->final_intensity = _final_intensity;
    c->increment = ( _final_intensity  - c->current_intensity) / temp;
    sei();
  }
}

/******************************************************************
  void LedControlUpdate(void)

  This function should be called exactly PWM_RESOLUTION * PWM_FREQ times
  per second, preferably from inside an interrupt. It may be interrupted
  by other interrupts, as long as the function ProcessNewCommand() is not
  executed from that interrupt.

  This function handles all PWM generation and intensity ramp generation.
*/
void LedControlUpdate(void)
{
  unsigned char i;
  LedChannelInfo *c;

  for (i = 0; i < N_CHANNELS; i++)
  {
    c = &(ledChannel[i]);

    c->counter++;

    if (c->counter >= PWM_RESOLUTION)
    {
      // Then counter has overflown; calculate new intensity
      c->counter = 0;


      // We need to do this differently for positive and negative
      // increments because we don't want the variables to overflow.
      // Note that current_intensity can be negative for a while, but
      // we cannot make it higher than the maximum intensity (not even
      // as an intermediate result during these calculations!)
      if ( c->increment > 0 )
      {
        // We want to do:
        //   if (cu+inc >= fi) then
        //       cu = fi; inc=0
        //   else
        //       cu = cu + inc;
        //   end
        // But we cannot do cu+inc directly because it may overflow. Hence:
        //    if (cu >= fi-inc) which can never overflow.
        if (c->current_intensity >= (c->final_intensity - c->increment))
        {
          c->current_intensity = c->final_intensity;
          c->increment = 0;
        } else
        {
          c->current_intensity += c->increment;
        }
      } else if (c->increment < 0)
      {
        // We want to do:
        //   if (cu+inc <= fi) then
        //      cu = fi; inc = 0;
        //    else
        //      cu = cu + inc;
        //    end;
        // and can do that directly because the numbers may be negative.
        if (c->current_intensity + c->increment <= c->final_intensity)
        {
          c->current_intensity = c->final_intensity;
          c->increment = 0;
        } else
        {
          c->current_intensity += c->increment;
        }
      } // else c->increment=0 -> don't do anything with current_intensity

      c->corrected_intensity = (PWM_RESOLUTION * pgm_read_byte(&(dim_curve[c->current_intensity >> 7]))) >> 8;
      //    c->corrected_intensity = ((c->current_intensity>>7)*PWM_RESOLUTION)>>8;
      // Turn on port (make output low)
      TurnOn(i);
    }

    if (c->counter == c->corrected_intensity)
      // Note that this never fails because current_intesnity is only set when counter==0
      // So there is no possiblilty that counter>currentintensity while the port is still on
    {
      // Turn off port (make output high)
      TurnOff(i);

    }
  }
}
