#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/wdt.h>
#include <avr/eeprom.h> 


#define high(pin) (PORTB |=  (1 << pin))
#define low(pin)  (PORTB &= ~(1 << pin))

#define RED   PB0
#define GREEN PB1
#define BLUE  PB2

static uint8_t red;
static uint8_t green;
static uint8_t blue;

// It seems as if the version of the arduino toolchain that can
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
  while(size>0){// 0 out red, blue, green
    eeprom_write_byte(ptr++,0);
    size--;
  }// 0 out the status byte
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
  // Otherwise, nuke the last bit with 0, and write the new value to the begining
  if(0 != eeprom_read_byte(loc))
    eeprom_write_byte(loc,0);
  if(color != 0)
    eeprom_write_byte(base,color);
}

uint8_t readColor(uint8_t* base){
  uint8_t i = RING_BUFFER_LEN;
  while(i-->0){
    uint8_t color = eeprom_read_byte(base++);
    if(color != 0){
      return color;
    }
  }
  return 0;
}



#define F_CPU 16000000UL
#define Clear(pin) (pinlevelB &= ~_BV(pin))
#define PORTB_DDR  0b00000111
#define PORTB_MASK 0b00000111

ISR (TIMER0_COMPA_vect)
{
  static uint8_t pinlevelB=PORTB_MASK;
  static uint8_t softcount=0xFF;

  PORTB = pinlevelB;

  if(++softcount == 0){         /* increment modulo 256 counter and update the compare values only when counter = 0. */
    pinlevelB = PORTB_MASK;     /* set all affected port pins high */
  }
  
  if(red   == softcount) Clear(RED);
  if(green == softcount) Clear(GREEN);
  if(blue  == softcount) Clear(BLUE);
}

void setup()
{
  // Initialize pinmodes: Turn pin 0, 1 and 2 to outputs.
  DDRB = PORTB_MASK;
  PRR = _BV(PRADC); // Turn off the ADC
  if(PINB & (1<<PB3)){
    // Enter the rather complex color observation mode, write the new ones out to the EPROM.
    red = 10; green = 10; blue = 0;
  }else{
    // Read the old values from the eeprom, initializing it if needed.
    checkAndInitialize();
    red   = readColor((uint8_t*) RED_BASE);
    green = readColor((uint8_t*) GREEN_BASE);
    blue  = readColor((uint8_t*) BLUE_BASE);
  }
  // Setup the timer for PWM, then loop
  cli();
  // Setup time0 to fire an interrupt every 75 cycles
  CLKPR = _BV(CLKPCE); // enable clock prescaler update - this bit must be set before
  CLKPR = 0;           // altering that register, then set clock speed to max
  TCCR0B = _BV(CS00);  // No prescaler
  TCCR0A = _BV(WGM01);// Clear the timer on compare
  // interrupt on compare register A
  OCR0A = 75;
  TIMSK |= _BV(OCIE0A);
  sei();// enable interrupts, enter an infinite loop - all work is done on interrupts.
  while(1);
}

void loop(){;}

void debug(char c){
  for(char i=0;i<8;i++){
    if(c &1){
      high(BLUE);
      delay(500);
      low(BLUE);
    }else{
      high(RED);
      delay(500);
      low(RED);
    }
    delay(1000);
    c = c>>1;
  }  
  delay(2000);
}


