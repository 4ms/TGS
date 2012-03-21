#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <ctype.h>

/** MODES **/


/** SETTINGS **/



/** PINS **/

#define CLOCK_IN_pin PD2
#define CLOCK_IN_init DDRD &= ~(1<<CLOCK_IN_pin)
#define CLOCK_IN (PIND & (1<<CLOCK_IN_pin))

#define CLOCK_LED_pin PD3
#define CLOCK_LED_init DDRD |=(1<<CLOCK_LED_pin)
#define CLOCK_LED_PORT PORTD

#define DOWNBEATMODE_pin PD5
#define DOWNBEATMODE_init DDRD &= ~(1<<DOWNBEATMODE_pin)
#define DOWNBEATMODE_init_pullup PORTD |= (1<<DOWNBEATMODE_pin) 
#define DOWNBEATMODE_JUMPER (!(PIND & (1<<DOWNBEATMODE_pin)))

#define GATEMODE_pin PD4
#define GATEMODE_init DDRD &= ~(1<<GATEMODE_pin)
#define GATEMODE_init_pullup PORTD |= (1<<GATEMODE_pin) 
#define GATEMODE_JUMPER (!(PIND & (1<<GATEMODE_pin)))


#define OUT_PORT1 PORTB
#define OUT_DDR1 DDRB
#define OUT_MASK1 0b00111111
#define OUT_init1 OUT_DDR1 |= OUT_MASK1

#define OUT_PORT2 PORTD
#define OUT_DDR2 DDRD
#define OUT_MASK2 0b11000000
#define OUT_init2 OUT_DDR2 |= OUT_MASK2

#define JUMPER_MASK 0b111100
#define JUMPER_PULLUP PORTC
#define JUMPER_PINS PINC
#define JUMPER_DDR DDRC
#define JUMPER_init JUMPER_DDR &= ~(JUMPER_MASK)

#define ADC_DDR DDRC
#define ADC_PORT PORTC
#define ADC_pin PC0

#define RANGE_JUMPER1 PC2
#define RANGE_JUMPER2 PC3
#define RESET_CV ((JUMPER_PINS & (1<<PC1)))
#define RANGE_JUMPERS (JUMPER_PINS & ((1<<RANGE_JUMPER1) | (1<<RANGE_JUMPER2)))
#define AUTO_RESET1_JUMPER (!(JUMPER_PINS & (1<<PC4)))
#define AUTO_RESET2_JUMPER (!(JUMPER_PINS & (1<<PC5)))


/** MACROS **/

#define ALLON(p,x) p &= ~(x)
#define ALLOFF(p,x) p |= (x)
#define ON(p,x) p &= ~(1<<(x))
#define OFF(p,x) p |= (1<<(x))


/** set_step() routine **/

inline void set_step(uint8_t step){

	if (step<=5) {
		ALLOFF(OUT_PORT2,OUT_MASK2);
		OUT_PORT1 |= (OUT_MASK1) & ~(1<<step); 	//turn off all unmasked pins except the one we want to keep on
		ON(OUT_PORT1,step);			//make sure the one we want is on
	} else if (step==6){
		ALLOFF(OUT_PORT1,OUT_MASK1);
		OFF(OUT_PORT2,6); 	//turn off #6 which is step==7
		ON(OUT_PORT2,7);	//make sure #7 is on (step==6)
	} else if (step==7){
		ALLOFF(OUT_PORT1,OUT_MASK1);
		OFF(OUT_PORT2,7); 	//turn off #7 which is step==6
		ON(OUT_PORT2,6);	//make sure #6 is on (step==7)
	}
	

}

/** MAIN **/


int main(void){

	unsigned char clock_up=0,clock_down=0, reset2_up=0;
	unsigned char o0=7,o1=0,clkdiv=0;
	unsigned char reset_step=0, old_reset_step=0;

	unsigned char adc=0;
	unsigned char switchread;


	CLOCK_IN_init; 
	CLOCK_LED_init;

	DOWNBEATMODE_init;
	DOWNBEATMODE_init_pullup;
	GATEMODE_init;
	GATEMODE_init_pullup;


	OUT_init1;
	OUT_init2;

	ALLOFF(OUT_PORT1,OUT_MASK1);
	ALLOFF(OUT_PORT2,OUT_MASK2);

	JUMPER_init; 
	JUMPER_PULLUP |= JUMPER_MASK; //turn pullups on

	//init the ADC:
	ADC_DDR &= ~(1<<ADC_pin); //adc input
	ADC_PORT &= ~(1<<ADC_pin); //disable pullup
	ADCSRA = (1<<ADEN);	//Enable ADC
	ADMUX = (1<<ADLAR);	//Left-Adjust
	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

	ADCSRA |= (1<<ADSC);		//set the Start Conversion Flag in the ADC Status Register
	

	while(1){


		/** READ ADC **/
		while( !(ADCSRA & (1<<ADIF)) );
		ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"
		adc=ADCH;
		
		ADCSRA |= (1<<ADSC);		//set the Start Conversion Flag in the ADC Status Register

		
		reset_step=adc >> 5; //0..255 -> 0..7
		if (old_reset_step != reset_step){
			o0=reset_step;
			set_step(o0);
			old_reset_step=reset_step;
		}

		/** READ JUMPERS **/
		switchread = RANGE_JUMPERS; //jumpers that set the rotate length
		
		//Jumper IN is Low (L), Jumper Out is High (H)

		if (switchread==0) {
			;}		//LL
		else if (switchread==(1<<RANGE_JUMPER2)) {
			;} 		//LH
		else if (switchread==(1<<RANGE_JUMPER1)) {
			;}		//HL
		else {
			;}		//HH


		if (RESET_CV) {
			if (reset2_up==0){
				reset2_up=1;
				o0=reset_step;
				set_step(o0);

			}
		} else {
			reset2_up=0;
		}



		if (CLOCK_IN){
			clock_down=0;

			if (!clock_up){
				clock_up=1;//rising edge only						

				ON(CLOCK_LED_PORT,CLOCK_LED_pin);

				if (++o0>7) o0=0;
				set_step(o0);


				if (AUTO_RESET1_JUMPER){
					if (AUTO_RESET2_JUMPER){
						//auto reset switches 1 and 2
					} else {
						//auto reset switch 1 only
					}
				} else {
					if (AUTO_RESET2_JUMPER){
					}
				}
			}
		}else{
			clock_up=0;

			if (!clock_down){
				clock_down=1;//rising edge only						

				OFF(CLOCK_LED_PORT,CLOCK_LED_pin);

				if (!GATEMODE_JUMPER){
					ALLOFF(OUT_PORT1,OUT_MASK1);
					ALLOFF(OUT_PORT2,OUT_MASK2);
				}
			}
		}
	
	}	//endless loop


	return(1);
}
