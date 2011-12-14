#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/wdt.h>

#define high(pin) (PORTB |=  (1 << pin))
#define low(pin)  (PORTB &= ~(1 << pin))

#define RED   PB0
#define GREEN PB1
#define BLUE  PB2

#define SWITCH_STATE (PINB & 0b1000)

static unsigned char red;
static unsigned char green;
static unsigned char blue;

#define F_CPU 16000000UL
#define Clear(pin) (pinlevelB &= ~_BV(pin))
#define PORTB_DDR  0b00000111
#define PORTB_MASK 0b00000111
#define PWM_ISR_CYCLES 100

void start_pwm(){
  CLKPR = _BV(CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)
  /*
    Set up timer 0 to fire an int every PWM_ISR_CYCLES clocks
  */
  /* No prescaler */
  TCCR0B = _BV(CS00);
  /* Clear timer on compare mode */
  TCCR0A = _BV(WGM01);
  /* Fire every PWM_ISR_CYCLES clock ticks */
  OCR0A = PWM_ISR_CYCLES;
  /* Int on compare A */
  TIMSK |= _BV(OCIE0A);
  /* Set LED pins as outputs */
  DDRB = PORTB_DDR;
  PRR = _BV(PRADC); // Turn off the ADC
}

ISR (TIMER0_COMPA_vect)
{
  static unsigned char pinlevelB=PORTB_MASK;
  static unsigned char softcount=0xFF;

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
  //DDRB |= 0b1000;
  if(PINB & (1<<PB3)){
    // Enter the rather complex color observation mode, write the new ones out to the EPROM.
    red = 10; green = 10; blue = 0;
  }else{
    // Read the old values from the EPROM. Use a ring-buffer, if we're clever.
    red = 0; green = 0; blue = 10;
  }
  // Setup the timer for PWM, then loop
  cli();
  start_pwm();
  sei();         // enable interrupts
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


