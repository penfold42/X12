/* X12.cpp

  An X12.017 Micro Stepper Library

  Written by penfold42

  Greatly inspired by:
  - Brett Hagman's timer library https://github.com/bhagman
  - Guy Carpenter's work https://github.com/clearwater/SwitecX25

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*************************************************/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"
#include "X12.h"

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
#define TCCR2A TCCR2
#define TCCR2B TCCR2
#define COM2A1 COM21
#define COM2A0 COM20
#define OCR2A OCR2
#define TIMSK2 TIMSK
#define OCIE2A OCIE2
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#define TIMSK1 TIMSK
#endif

// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)

#if !defined(__AVR_ATmega8__)
volatile long timer0_toggle_count;
#endif

volatile long timer1_toggle_count;
volatile long timer2_toggle_count;

#if defined(TIMSK3)
volatile long timer3_toggle_count;
#endif

#if defined(TIMSK4)
volatile long timer4_toggle_count;
#endif

#if defined(TIMSK5)
volatile long timer5_toggle_count;
#endif


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define AVAILABLE_TONE_PINS 1
#define USE_TIMER2

const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 3, 4, 5, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255, 255, 255, 255 */ };

#elif defined(__AVR_ATmega8__)

#define AVAILABLE_TONE_PINS 1
#define USE_TIMER2

const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 1 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255 */ };

#elif defined(__AVR_ATmega32U4__)
 
#define AVAILABLE_TONE_PINS 1
#define USE_TIMER3
 
const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 3 /*, 1 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255 */ };
 
#elif defined(__AVR_ATmega328P__)

#define AVAILABLE_TONE_PINS 1
#define USE_TIMER2

// Leave timer 0 to last.
const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255 */ };

#else
#error unknown CPU architecture

#endif

    bool debug = false;
    bool reverse = true;           // is the stepper motor wired backwards?
    unsigned char pinStep;
    unsigned char pinDir;
    unsigned char pinReset;
    volatile uint8_t *step_pin_port;
    volatile uint8_t step_pin_mask;
    volatile uint8_t *dir_pin_port;
    volatile uint8_t dir_pin_mask;

    unsigned int currentStep;      // step we are currently at
    unsigned int steps = 315*12;            // total steps available
    unsigned short (*accelTable)[2]; // accel table can be modified.
    unsigned int maxVel;           // fastest vel allowed
    unsigned int vel;              // steps travelled under acceleration
    int dir;                       // direction -1,0,1

    volatile bool manual = false;          // if true interrupts are ignored
    volatile bool stopped;         // true if stopped
    volatile unsigned int targetStep;       // target we are moving to


// This table defines the acceleration curve.
// 1st value is the speed step, 2nd value is delay in interrupt counts (~70uSec)
// 1st value in each row must be > 1st value in subsequent row
// 1st value in last row is used for maxVel

#define FREQ 14300
#define PERIOD (1000000/FREQ)
/*
static unsigned short olddefaultAccelTable[][2] = {
  {    80, 10}, // less than 80 steps, count 6 interrupts per step
  {   100,  8},
  {   160,  5},
  {   240,  3},
  {   320,  2},
};

// y = 0.0067x2 - 0.7025x + 104.92
static unsigned short gooddefaultAccelTable[][2] = {
  {	40,	16},
  {	60,	15},
  {	80,	14},
  {	120,	10},
  {	160,	7},
  {	200,	5},
  {	240,	4},
  {	280,	3},
  {	320,	2},
};
*/

static unsigned short defaultAccelTable[][2] = {
  {	40,	14},
  {	60,	12},
  {	80,	10},
  {	120,	8},
  {	160,	7},
  {	200,	5},
  {	240,	4},
  {	280,	3},
  {	320,	2},
};


const int stepPulseMicrosec = 1;
const int resetStepMicrosec = 400;
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))


static int8_t findTimer(uint8_t _pin)
{
  int8_t _timer = -1;

  // if we're already using the pin, the timer should be configured.  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      return pgm_read_byte(tone_pin_to_timer_PGM + i);
    }
  }
  
  // search for an unused timer.
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == 255) {
      tone_pins[i] = _pin;
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      break;
    }
  }
  
  if (_timer != -1)
  {
    // Set timer specific stuff
    // All timers in CTC mode
    // 8 bit timers will require changing prescalar values,
    // whereas 16 bit timers are set to either ck/1 or ck/64 prescalar
    switch (_timer)
    {
      #if defined(TCCR0A) && defined(TCCR0B) && defined(WGM01)
      case 0:
        // 8 bit timer
        TCCR0A = 0;
        TCCR0B = 0;
        bitWrite(TCCR0A, WGM01, 1);
        bitWrite(TCCR0B, CS00, 1);
        break;
      #endif

      #if defined(TCCR1A) && defined(TCCR1B) && defined(WGM12)
      case 1:
        // 16 bit timer
        TCCR1A = 0;
        TCCR1B = 0;
        bitWrite(TCCR1B, WGM12, 1);
        bitWrite(TCCR1B, CS10, 1);
        break;
      #endif

      #if defined(TCCR2A) && defined(TCCR2B)
      case 2:
        // 8 bit timer
        TCCR2A = 0;
        TCCR2B = 0;
        bitWrite(TCCR2A, WGM21, 1);
        bitWrite(TCCR2B, CS20, 1);
        break;
      #endif

      #if defined(TCCR3A) && defined(TCCR3B) &&  defined(TIMSK3)
      case 3:
        // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        bitWrite(TCCR3B, WGM32, 1);
        bitWrite(TCCR3B, CS30, 1);
        break;
      #endif

      #if defined(TCCR4A) && defined(TCCR4B) &&  defined(TIMSK4)
      case 4:
        // 16 bit timer
        TCCR4A = 0;
        TCCR4B = 0;
        #if defined(WGM42)
          bitWrite(TCCR4B, WGM42, 1);
        #elif defined(CS43)
          // TODO this may not be correct
          // atmega32u4
          bitWrite(TCCR4B, CS43, 1);
        #endif
        bitWrite(TCCR4B, CS40, 1);
        break;
      #endif

      #if defined(TCCR5A) && defined(TCCR5B) &&  defined(TIMSK5)
      case 5:
        // 16 bit timer
        TCCR5A = 0;
        TCCR5B = 0;
        bitWrite(TCCR5B, WGM52, 1);
        bitWrite(TCCR5B, CS50, 1);
        break;
      #endif
    }
  }

  return _timer;
}



void X12begin( unsigned int _steps, unsigned char _pinStep, unsigned char _pinDir, unsigned char _pinReset)
{
  steps = _steps;
  pinStep = _pinStep;
  step_pin_port = portOutputRegister(digitalPinToPort(pinStep));
  step_pin_mask = digitalPinToBitMask(pinStep);

  pinDir = _pinDir;
  dir_pin_port = portOutputRegister(digitalPinToPort(pinDir));
  dir_pin_mask = digitalPinToBitMask(pinDir);

  pinReset = _pinReset;

  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(pinReset, OUTPUT);

  digitalWrite(pinReset, LOW);
  *step_pin_port &= ~(step_pin_mask); //digitalWrite(pinStep, LOW);
  *dir_pin_port &= ~(dir_pin_mask); //digitalWrite(pinDir, LOW);

  dir = 0;
  vel = 0;
  stopped = true;
  manual = false;
  currentStep = 0;
  targetStep = 0;

  accelTable = defaultAccelTable;
  maxVel = accelTable[DEFAULT_ACCEL_TABLE_SIZE-1][0]; // last value in table.

  digitalWrite(pinReset, HIGH);

  setupTimer (_pinStep);
}



// frequency (in hertz) 7246 == 138uSec micro step pulse
//void setupTimer(uint8_t _pin, unsigned int _frequency )
void setupTimer(uint8_t _pin)
{
  uint8_t prescalarbits = 0b001;
  uint32_t ocr = 0;
  int8_t _timer;

  int frequency = FREQ;
  _timer = findTimer(_pin);

//Serial.print("using timer: ");
//Serial.println(_timer);
  if (_timer >= 0)
  {

    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    if (_timer == 0 || _timer == 2)
    {
      ocr = F_CPU / frequency - 1;
      prescalarbits = 0b001;  // ck/1: same for both timers
      if (ocr > 255)
      {
        ocr = F_CPU / frequency / 8 - 1;
        prescalarbits = 0b010;  // ck/8: same for both timers

        if (_timer == 2 && ocr > 255)
        {
          ocr = F_CPU / frequency / 32 - 1;
          prescalarbits = 0b011;
        }

        if (ocr > 255)
        {
          ocr = F_CPU / frequency / 64 - 1;
          prescalarbits = _timer == 0 ? 0b011 : 0b100;

          if (_timer == 2 && ocr > 255)
          {
            ocr = F_CPU / frequency / 128 - 1;
            prescalarbits = 0b101;
          }

          if (ocr > 255)
          {
            ocr = F_CPU / frequency / 256 - 1;
            prescalarbits = _timer == 0 ? 0b100 : 0b110;
            if (ocr > 255)
            {
              // can't do any better than /1024
              ocr = F_CPU / frequency / 1024 - 1;
              prescalarbits = _timer == 0 ? 0b101 : 0b111;
            }
          }
        }
      }

#if defined(TCCR0B)
      if (_timer == 0)
      {
        TCCR0B = (TCCR0B & 0b11111000) | prescalarbits;
      }
      else
#endif
#if defined(TCCR2B)
      {
        TCCR2B = (TCCR2B & 0b11111000) | prescalarbits;
      }
#else
      {
        // dummy place holder to make the above ifdefs work
      }
#endif
    }
    else
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency - 1;

      prescalarbits = 0b001;
      if (ocr > 0xffff)
      {
        ocr = F_CPU / frequency / 64 - 1;
        prescalarbits = 0b011;
      }

      if (_timer == 1)
      {
#if defined(TCCR1B)
        TCCR1B = (TCCR1B & 0b11111000) | prescalarbits;
#endif
      }
#if defined(TCCR3B)
      else if (_timer == 3) {
        TCCR3B = (TCCR3B & 0b11111000) | prescalarbits;
//Serial.print("TCCR3B: ");
//Serial.println(TCCR3B, BIN);
      }
#endif
#if defined(TCCR4B)
      else if (_timer == 4)
        TCCR4B = (TCCR4B & 0b11111000) | prescalarbits;
#endif
#if defined(TCCR5B)
      else if (_timer == 5)
        TCCR5B = (TCCR5B & 0b11111000) | prescalarbits;
#endif

    }
    

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts
    switch (_timer)
    {

#if defined(OCR0A) && defined(TIMSK0) && defined(OCIE0A)
      case 0:
        OCR0A = ocr;
        bitWrite(TIMSK0, OCIE0A, 1);
        break;
#endif

      case 1:
#if defined(OCR1A) && defined(TIMSK1) && defined(OCIE1A)
        OCR1A = ocr;
        bitWrite(TIMSK1, OCIE1A, 1);
#elif defined(OCR1A) && defined(TIMSK) && defined(OCIE1A)
        // this combination is for at least the ATmega32
        OCR1A = ocr;
        bitWrite(TIMSK, OCIE1A, 1);
#endif
        break;

#if defined(OCR2A) && defined(TIMSK2) && defined(OCIE2A)
      case 2:
        OCR2A = ocr;
        bitWrite(TIMSK2, OCIE2A, 1);
        break;
#endif

#if defined(OCR3A) && defined(TIMSK3) && defined(OCIE3A)
      case 3:
        OCR3A = ocr;
//Serial.print("OCR3A: ");
//Serial.println(OCR3A, DEC);
        bitWrite(TIMSK3, OCIE3A, 1);
        break;
#endif

#if defined(OCR4A) && defined(TIMSK4) && defined(OCIE4A)
      case 4:
        OCR4A = ocr;
        bitWrite(TIMSK4, OCIE4A, 1);
        break;
#endif

#if defined(OCR5A) && defined(TIMSK5) && defined(OCIE5A)
      case 5:
        OCR5A = ocr;
        bitWrite(TIMSK5, OCIE5A, 1);
        break;
#endif

    }
  }
}


// XXX: this function only works properly for timer 2 (the only one we use
// currently).  for the others, it should end the tone, but won't restore
// proper PWM functionality for the timer.
void disableTimer(uint8_t _timer)
{
  switch (_timer)
  {
    case 0:
      #if defined(TIMSK0)
        TIMSK0 = 0;
      #elif defined(TIMSK)
        TIMSK = 0; // atmega32
      #endif
      break;

#if defined(TIMSK1) && defined(OCIE1A)
    case 1:
      bitWrite(TIMSK1, OCIE1A, 0);
      break;
#endif

    case 2:
      #if defined(TIMSK2) && defined(OCIE2A)
        bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt
      #endif
      #if defined(TCCR2A) && defined(WGM20)
        TCCR2A = (1 << WGM20);
      #endif
      #if defined(TCCR2B) && defined(CS22)
        TCCR2B = (TCCR2B & 0b11111000) | (1 << CS22);
      #endif
      #if defined(OCR2A)
        OCR2A = 0;
      #endif
      break;

#if defined(TIMSK3) && defined(OCIE3A)
    case 3:
      bitWrite(TIMSK3, OCIE3A, 0);
      break;
#endif

#if defined(TIMSK4) && defined(OCIE4A)
    case 4:
      bitWrite(TIMSK4, OCIE4A, 0);
      break;
#endif

#if defined(TIMSK5) && defined(OCIE5A)
    case 5:
      bitWrite(TIMSK5, OCIE5A, 0);
      break;
#endif
  }
}


void noTone(uint8_t _pin)
{
  int8_t _timer = -1;
  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      tone_pins[i] = 255;
      break;
    }
  }
  
  disableTimer(_timer);

}


#ifdef USE_TIMER0
ISR(TIMER0_COMPA_vect)
{
  if (--timer0_toggle_count <= 0)
  {
    timer0_toggle_count = advance();
  }
}
#endif


#ifdef USE_TIMER1
ISR(TIMER1_COMPA_vect)
{
  if (--timer1_toggle_count <= 0)
  {
    timer1_toggle_count = advance();
  }
}
#endif


#ifdef USE_TIMER2
ISR(TIMER2_COMPA_vect)
{
  if (--timer2_toggle_count <= 0)
  {
    timer2_toggle_count = advance();
  }
}
#endif


#ifdef USE_TIMER3
ISR(TIMER3_COMPA_vect)
{
  if (--timer3_toggle_count <= 0)
  {
    timer3_toggle_count = advance();
  }
}
#endif


#ifdef USE_TIMER4
ISR(TIMER4_COMPA_vect)
{
  if (--timer4_toggle_count <= 0)
  {
    timer4_toggle_count = advance();
  }
}
#endif


#ifdef USE_TIMER5
ISR(TIMER5_COMPA_vect)
{
  if (--timer5_toggle_count <= 0)
  {
    timer5_toggle_count = advance();
  }
}
#endif

/* do one step based on direction */
void step(int dir)
{
  if ( (reverse && (dir > 0)) ||  (!reverse && (dir <= 0)) )
  {
    *dir_pin_port |= dir_pin_mask;
    *dir_pin_port |= dir_pin_mask;	// do it twice to ensure step setup time is met (100nS)
  } else {
    *dir_pin_port &= ~(dir_pin_mask);
    *dir_pin_port &= ~(dir_pin_mask);	// do it twice to ensure step setup time is met (100nS)
  }
  
  *step_pin_port |= step_pin_mask;
  delayMicroseconds(stepPulseMicrosec);
  *step_pin_port &= ~(step_pin_mask);

  currentStep += dir;
  if (currentStep > 65000) {
    currentStep = 0;
  }
}

/* BLOCKING! step to this position now */
void stepTo(unsigned int position, unsigned int _delay)
{
  int count;
  int dir;

  if (position > currentStep) {
    dir = 1;
    count = position - currentStep;
  } else {
    dir = -1;
    count = currentStep - position;
  }
  for (int i=0;i<count;i++) {
    step(dir);
//    delayMicroseconds(resetStepMicrosec);
    delayMicroseconds(_delay);
  }
}

/* BLOCKING! step to zero position now */
void X12zero()
{
  manual = true;
  currentStep = steps - 1;
  stepTo(0, resetStepMicrosec);
  currentStep = targetStep = 0;
  vel = 0;
  dir = 0;
  manual = false;
}


/* BLOCKING! step to full scale now */
void X12full()
{
  manual = true;
  currentStep = 0;
  stepTo(steps-1, resetStepMicrosec);
  currentStep = targetStep = steps-1;
  vel = 0;
  dir = 0;
  manual = false;
}


/* called from interrupt handler 
 * calculates acceleration and calls step()
 * return next interrupt count, 1 is immediate
 */
int advance()
{
  // detect stopped state
  if ( (currentStep==targetStep && vel==0) || (manual) ) {
    stopped = true;
    dir = 0;
//    return 6;
    // return the first = slowest interrupt counter
    return ( accelTable[0][1] );
  }

  // if stopped, determine direction
  if (vel==0) {
    dir = currentStep<targetStep ? 1 : -1;
    // do not set to 0 or it could go negative in case 2 below
    vel = 1;
  }

  step(dir);

  // determine delta, number of steps in current direction to target.
  // may be negative if we are headed away from target
  int delta = dir>0 ? targetStep-currentStep : currentStep-targetStep;
 
  if (delta>0) {
    // case 1 : moving towards target (maybe under accel or decel)
    if (abs(delta) < vel) {
      // time to declerate
      vel--;
    } else if (vel < maxVel) {
      // accelerating
      vel++;
    } else {
      // at full speed - stay there
    }
  } else {
    // case 2 : at or moving away from target (slow down!)
    vel--;
  }

  // use VELocity to lookup up the delay
  unsigned char i = 0;
  // this is why vel must not be greater than the last vel in the table.
  while (accelTable[i][0] < vel) {
    i++;
  }
  return ( accelTable[i][1] );
}

void X12setPosition(unsigned int pos)
{
  // pos is unsigned so don't need to check for <0
  if (pos >= steps) pos = steps-1;
  targetStep = pos;
  if (stopped) {
    stopped = false;
  }
}

unsigned int X12getPosition() {
  return targetStep;
}



