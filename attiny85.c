#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/wdt.h>
#include <avr/eeprom.h> 


#define RED    (1<<PB0)
#define GREEN  (1<<PB1)
#define BLUE   (1<<PB2)
#define SWITCH (1<<PB3)
#define SENSOR (1<<PB4)


static uint8_t red;
static uint8_t green;
static uint8_t blue;

// It seems as if the version of the Arduino toolchain that can
// deal with attiny85s can't deal with EEMEM directives. So we hardcode 
// all of the addresses...
#define RING_BUFFER_LEN 8
#define INIT_FLAG 0
#define BLUE_BASE 1
#define RED_BASE (1+RING_BUFFER_LEN)
#define GREEN_BASE (RED_BASE+RING_BUFFER_LEN)

// Checks if a status byte has been previously set, and if it hasn't, initializes the memory.
void checkAndInitialize(){
  if(0xCC==eeprom_read_byte((uint8_t*)INIT_FLAG))
    return;
  uint8_t* ptr = (uint8_t*) BLUE_BASE;
  char size = 3*RING_BUFFER_LEN;
  while(size-->0)// 0 out red, blue, green
    eeprom_write_byte(ptr++,0);
  // Set the status byte to 0xCC
  eeprom_write_byte((uint8_t*) INIT_FLAG,0xCC);
}
void writeColor(uint8_t color, uint8_t* base){
  uint8_t i = RING_BUFFER_LEN;
  uint8_t* loc = base;
  while(i-->1){// Not going to bother to check if we can skip writing.
    if(0 != eeprom_read_byte(loc)){
      eeprom_write_byte(loc++,0);
      if(color != 0)
	eeprom_write_byte(loc,color);
      return;
    }
    loc++;
  }
  // Otherwise, nuke the last byte with 0, and write the new value to the beginning
  if(0 != eeprom_read_byte(loc))
    eeprom_write_byte(loc,0);
  if(color != 0)
    eeprom_write_byte(base,color);
}

uint8_t readColor(uint8_t* base){
  uint8_t i = RING_BUFFER_LEN;
  while(i-->0){
    uint8_t color = eeprom_read_byte(base++);
    if(color != 0)
      return color;
  }
  return 0;
}

/* Color observation... This is, of course, meh and ultimately rather subjective.
Based on "Very Low-Cost Sensing and Communication Using Bidirectional LEDs" - Mitsubishi Electric Research Laboratories
Fortunately, we don't have to worry about the relative intensities of the different LEDs.
*/
uint32_t relative_intensity(uint8_t color){
  // Illuminate the sample with the relevant color
  if(color > 0) PORTB |= color;
  // Charge the LED-qua-capacitor 
  DDRB  |= SENSOR;
  PORTB |= SENSOR;  
  /* Set the cathode to high-impedance input mode, and wait for the photocurrent to
     discharge the device. */
  DDRB  &= ~SENSOR;
  PORTB &= ~SENSOR;
  uint32_t start = micros();
  while(PINB & SENSOR);
  uint32_t stop = micros();
  if(color > 0) PORTB &= ~color;
  return stop-start;
}

void observe_color(void){
  uint32_t baseline = relative_intensity(0);
  uint32_t r        = relative_intensity(RED)-baseline;
  uint32_t g        = relative_intensity(GREEN)-baseline;
  uint32_t b        = relative_intensity(BLUE)-baseline;
  baseline  = r*r;
  baseline += g*g;
  baseline += b*b;
  // Divide everything by the aproximate square root of baseline. Make sure to overshoot.
  uint8_t log = 1; // base two integer log of baseline, plus one.
  while(baseline > 0){
    baseline >>= 1;
    log++;
  }
  // Apparently
  r <<= 8;
  r >>= log;
  g <<= 8;
  g >>= log;
  b <<= 8;
  b >>= log;
  // Then clamp everything to 0..255
  red   = (r > 255)? 255 : r;
  green = (g > 255)? 255 : g;
  blue  = (b > 255)? 255 : b;
}

#define PORTB_MASK 0b00000111

ISR (TIMER0_COMPA_vect)
{
  static uint8_t output=PORTB_MASK;
  static uint8_t icount=0xFF;
  // Output the (potentially new) values
  PORTB = output;
  // Every 256 interrupts, put all of the pins back to their initial state.
  if(++icount == 0)
    output = PORTB_MASK;    
  // Periodically turn them off
  if(red   == icount) output &= ~RED;
  if(green == icount) output &= ~GREEN;
  if(blue  == icount) output &= ~BLUE;
}

void setup()
{
  // Initialize pin modes: set pin 0, 1 and 2 to outputs.
  DDRB = 0b00000111;
  PRR = _BV(PRADC); // Turn off the ADC
  checkAndInitialize();
  if(PINB & SWITCH){
    // Enter the rather complex color observation mode, write the new ones out to the EPROM.
    observe_color();
    writeColor(red, (uint8_t*) RED_BASE);
    writeColor(green, (uint8_t*) GREEN_BASE);
    writeColor(blue, (uint8_t*) BLUE_BASE);
  }else{// Read the old values from the eeprom, initializing it if needed.
    red   = readColor((uint8_t*) RED_BASE);
    green = readColor((uint8_t*) GREEN_BASE);
    blue  = readColor((uint8_t*) BLUE_BASE);
  }
  /* Setup timer0 to fire an interrupt every 75 cycles, the enter an infinite loop.
     All the work is done on the interrupts - once we've figured out what colors to use, the
     device simply has to PWM them until it's shutoff.
     Based on Atmel's "AVR136: Low-jitter Multi-channel Software PWM" */
  cli(); // This is probably not needed. Disable interrupts.
  /* Interesting design: before altering any other bits in CLKPR, CLKPCE must be set.
     The processor then resets CLKPCE within four cycles. After we've enabled updating
     CLKPR, set the clock speed to its maximum. */
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;
  TCCR0B = _BV(CS00);  // No prescaler
  TCCR0A = _BV(WGM01);// Clear the timer on compare
  OCR0A = 75;
  TIMSK |= _BV(OCIE0A);// interrupt on compare with register A, which just happens to be 75. 
  sei(); // Turn everything on, and loop!
  while(1);
}
void loop(){;}

void debug(char c){
  for(char i=0;i<8;i++){
    if(c &1){
      PORTB |= BLUE;
      delay(500);
      PORTB &= ~BLUE;
    }else{
      PORTB |= RED;
      delay(500);
      PORTB &= ~RED;
    }
    delay(1000);
    c = c>>1;
  }  
  delay(2000);
}


