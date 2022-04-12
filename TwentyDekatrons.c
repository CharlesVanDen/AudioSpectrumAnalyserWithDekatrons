/*
 * TwentyDekatrons.c
 *
 * Created: 9-9-2020
 *  Author: CCT
 ATMEGA328 <-- check uctrler type before compiling!
 20MHz
Abstract
Seven dekatrons are used as indicators in a stereo 7 channel (14 channel in total) frequency analyzer for audio application.
The left and right audio signals are fed to two MSGEQ7's. Each of these contains 7 band pass filters (60Hz, 150Hz etc.) and the signal level is sampled.
The AVR gets the audio levels as analog signals, digitizes them and uses the dekatrons as display devices.
The Dekatrons are high voltage devices. Each one is powered by a number of MPSA42 transistors, connected to a 74HC595 shift register. The HC595's are 
chained together microcontroller's SPI output (MOSI).

The dekatrons work as follows. Each dekatron (in the view of this program) has 60 states. Why 60, isn't a dekatron supposed to have 10 states, hence 'deka'?
Yes originally, the dekatron is supposed to have ten stable states. But in between each stable states are two transitional states. In total, the dekatron has
30 cathodes, 10 for each stable state and between each stable state there are two cathodes for the transition. Originally the dekatron was used as a counting 
tube. By applying a shifted pulse on the first and then then the second transitional cathode, it could count "one up". In this application however, we
use all 30 cathodes. Furthermore, there is the possibility to energize two cathodes (lying next to each other) at the same time and this is used as transition 
between one cathode and the next. In this way we arrive at 60 states.

This application relies on rapid switching between states. So rapid that it can fool the eye. The current state of dekatron i (a number between 0 and 59) is 
stored in values[i]. Like a windscreen wiper, this is rapidly varied between two extreme states. The extreme states are stored in minima[i] and maxima[i]. The 
current direction of movement (towards minima or towards maxima) is stored in orient[]. The state is changed every 150us. When 150us have passed, interrupt 
service routine TIMER0_COMPA_vect() is called. At this time, the new values to be sent out to the dekatrons are already prepared in buffer[]. The first value 
out of buffer[] is sent out over the SPI bus at a rate of 5Mb/s. When this SPI transfer is finished, interrupt service routine SPI_STC_vect() is called, which
sends out the next byte over the SPI bus. When all 7 bytes are sent out in this way, SPI_STC_vect() calculates the new values[]. The value of buffer[] is 
calculated, it must contain the actual bit pattern to be sent to the SPI bus. The map[] array contains the translation from values[i] to buffer[i].
If necessary (when values[i] equals minima[i] or maxima[i]), the orient[i] is also inverted.

Example. For dekatron #3, the values are as follows:
value[3]=10
buffer[3]=2 (this is the bit pattern that corresponds to state 10)
minima[3]=2
maxima[3]=11
orient[3]=1 (move to the right, towards 11)

Because orient[3] is 1, we must move to the right, so value[s] must become 11. Therefore the new values are:
value[3]=11
buffer[3]=6 (this is the bit pattern that corresponds to state 11)
minima[3]=2
maxima[3]=11
orient[3]=1 (move to the right, towards 11)

In the next cycle, value[3] has become equal to maxima[3], the orientation must change to "left" and the new values are:

value[3]=11
buffer[3]=6 (this is the bit pattern that corresponds to state 11)
minima[3]=2
maxima[3]=11
orient[3]=0 (move to the right, towards 2)

In the next cycle the values will be:
value[3]=10
buffer[3]=2 (this is the bit pattern that corresponds to state 10)
minima[3]=2
maxima[3]=11
orient[3]=0 (move to the right, towards 2)


Be aware that a dekatron is a circular thing and the state model is also circular. You would maybe expect that for each dekatron i, minima[i] should always 
be numerically lower than maxima[i], but in a circular model, this is not required. Values like the following are perfectly valid:

value[3]=0
buffer[3]=64 (this is the bit pattern that corresponds to state 0)
minima[3]=58
maxima[3]=2
orient[3]=1 (move to the left, towards 58)

In the next cycle, the values would be:
value[3]=59
buffer[3]=66 (this is the bit pattern that corresponds to state 59)
minima[3]=58
maxima[3]=2
orient[3]=1 (move to the left, towards 58)

The main loop is in charge of changing the values of minima[] and maxima[] according to the sampled analogue values and the interrupt routines will handle the rest. 
However there is one catch. Normally, values[i] is supposed to lie between minima[i] and maxima[i]**. If this is no longer the case, values[i] must return to the range 
between minima[i] and maxima[i] a.s.a.p! This is not done abruptly but  step by step and the function that is in charge is checkRange(). It checks whether this 
condition is the case. If that is so, the only thing it will do is change the value of orient[i] and only if that helps to move within bounds asap.

** this is easier said than it really is, because in a circular model values[i] is always "between" minima[i] and maxima[i]. We also have to take orient[i] into account. 
If orient[i]==0, we are moving to the right and values[i] should arrive at maxima[i] before arriving at minima[i]. If this is not the case, values is no longer "between" 
minima[i] and maxima[i]. Similar when orient[i]==1.

 connect:
 ATMEGA pin         74HC575 pin
 MOSI PB3 pin 17 to SI pin 14
 SCK  PB5 pin 19 to SCK pin 11
 SS   PB2 pin 16 to RCK pin 12 
	                ~G  pin 13 to ground
					~SCLR pin 10 to VCC
 
 Connections to MSGEQ7:
 
 PC0 analog in left channel
 PC1 STROBE
 PC2 RESET
 PC3 analog in right channel
 
 The connections of the 6802 dekatron
 In order to control any dekatron, we must have access to the common kathode, the two intermediate cathode and at least one reset kathode. The 6802 can be reset 
 to states 0, 1, 5, 8 and 9. The connections are as follows: 
 
 pin number | function                    | connected to bit n of the HC595
 -----------+-----------------------------+-----------------------------------
 1          | common kathode K1/2/3/4/6/7 | 2
 2          | K5                          | 3
 3          | intermediate kathode G1     | 0
 4          | anode                       |
 5          | intermediate kathode G2     | 1
 6          | K9                          | 5
 7          | K0                          | 6
 8          | K8                          | 4
 
 (bit 7 is not used)
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#define NUM_DEKATRONS 7
#define NUM_STATES 60
#define NUM_CHANNELS 7
#define NUM_MEASUREMENTS 5

void checkRange(uint8_t i);

void delay_ms(uint16_t x)
{
	uint8_t y, z;
	for ( ; x > 0 ; x--){
		for ( y = 0 ; y < 90 ; y++){
			for ( z = 0 ; z < 6 ; z++){
				asm volatile ("nop");
			}
		}
	}
}

void delay_dms(uint16_t x)
{
	uint8_t y, z;
	for ( ; x > 0 ; x--){
		for ( y = 0 ; y < 9 ; y++){
			for ( z = 0 ; z < 6 ; z++){
				asm volatile ("nop");
			}
		}
	}
}

//                    0   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28  29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46  47  48  49 50 51 52  53  54  55 56 57 58  59
const uint8_t map[]={64, 65, 1, 3, 2, 6, 4, 5, 1, 3, 2, 6, 4, 5, 1, 3, 2, 6, 4, 5, 1, 3, 2, 6, 4, 5, 1, 3, 2, 10, 8, 9, 1, 3, 2, 6, 4, 5, 1, 3, 2, 6, 4, 5, 1, 3, 2, 18, 16, 17, 1, 3, 2, 34, 32, 33, 1, 3, 2, 66};

volatile uint8_t buffer[NUM_DEKATRONS+1];
volatile uint8_t values[NUM_DEKATRONS+1]; //current position of the dot
volatile uint8_t orient[NUM_DEKATRONS+1]; //direction of movement: 1=right, towards max. 0=left, towards min
volatile uint8_t minima[NUM_DEKATRONS+1]; //minimum value of the dot
volatile uint8_t maxima[NUM_DEKATRONS+1]; //maximum value of the dot
volatile uint8_t decnum=0;

int main(void)
{
	DDRB=0b00101100;//Set PB2, 3 and 5 as output
	DDRC=0b00000110;//SET pc1 and pc2 as output
	DDRD=(1<<2);

	//setup SPI
	SPCR=(1<<SPIE)|(1<<SPE)|(1<<MSTR); // Fspi = fcpu/4 = 20/4 = 5MHz
	//SPSR|=(1<<SPI2X); //SPI2X double spi speed
	sei(); //global interrupt enable
	
	//setup timercounter0 to run an interrupt routine every 150us, that will update all the tubes
	//f=20MHz/64/47 Mode 7 fast pwm
	TCCR0A=(1<<WGM00)|(1<<WGM01);
	TCCR0B=(1<<CS00)|(1<<CS01)|(1<<WGM02);
	OCR0A=46;
	TIMSK0=(1<<OCIE0A);
	
	//setup timercounter1 normal mode at f=fcpu/8=20MHz/8=2.5 MHz
	// No PWM, no Interrupt Service Routine, just to be used in the main loop.
	TCCR1B=(1<<CS11);
	
	//setup ADC
	//ref voltage =Vcc=5V
	//PRESCALER=128-->20MHz/128=156kHz ADPS0, 1 and 2
	//PRESCALER=64-->20MHz/64=312.5kHz ADPS1 and 2
	//CHANNEL 0
	ADMUX=(1<<REFS0);
	//ADCSRA=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
	ADCSRA=(1<<ADEN)|(1<<ADPS1)|(1<<ADPS2);
	ADCSRB=0;
	DIDR0=0b00111111;
	
	//uint16_t speed=2;

	for(uint8_t i=0; i<NUM_DEKATRONS; i++) orient[i]=0;
	for(uint8_t i=0; i<NUM_DEKATRONS; i++) maxima[i]=10;
	for(uint8_t i=0; i<NUM_DEKATRONS; i++) minima[i]=NUM_STATES-1-maxima[i];
	for(uint8_t i=0; i<NUM_DEKATRONS; i++) values[i]=minima[i];
	for(uint8_t i=0; i<NUM_DEKATRONS; i++) buffer[i]=map[values[i]];
	
    while(1){
		static uint16_t measurements[NUM_CHANNELS][2];//storage for 7 frequencies times 2 channels (left and right)
		for(uint8_t ch=0; ch<NUM_CHANNELS; ch++) {
			measurements[ch][0]=0xffff;
			measurements[ch][1]=0xffff;
		}

		//give reset pulse
		PORTC|=(1<<2); //RESET HIGH
		TCNT1=0;//reset timer counter 1
		while(TCNT1<250){}	//do nothing; just wait for 100us to elapse
		PORTC&= ~(1<<2); //RESET LOW

		for (uint8_t strobe=0; strobe<NUM_MEASUREMENTS; strobe++){
			for(uint8_t ch=0; ch<NUM_CHANNELS; ch++){
				PORTC|=(1<<1); //STROBE HIGH
				TCNT1=0;//reset timer counter 1
				while(TCNT1<250){}	//do nothing; just wait for 100us to elapse
		
				PORTC&= ~(1<<1); //STROBE LOW
				TCNT1=0;//reset timer counter 1
				PORTD&= ~(1<<2);//debug

				//do three measurements and discard the highest and lowest value
				uint16_t sumL=0;
				uint16_t maxL=0;
				uint16_t minL=0xffff;
				uint16_t sumR=0;
				uint16_t maxR=0;
				uint16_t minR=0xffff;
				for(uint8_t i=0; i<3; i++){
					while(TCNT1<195){}	//do nothing; just wait for TCNT1 x 0.4us us to elapse
					TCNT1=0;//reset timer counter 1
					PIND|=(1<<2);//toggle pd2 for debug
					
					//LEFT CHANNEL
					ADMUX&= 0b11110000; //select analog input PC0
					ADCSRA|=(1<<ADSC); //start ADC CONVERSION
					while(ADCSRA&(1<<ADSC)){};//wait for complete adc
					uint16_t result=ADC;
					sumL+=result;
					if(result>maxL)maxL=result;
					if(result<minL)minL=result;
					
					//RIGHT CHANNEL
					ADMUX|= 0b00000011; //select analog input PC3
					ADCSRA|=(1<<ADSC); //start ADC CONVERSION
					while(ADCSRA&(1<<ADSC)){};//wait for complete adc
					result=ADC;
					sumR+=result;
					if(result>maxR)maxR=result;
					if(result<minR)minR=result;
				}
				sumL=sumL-maxL-minL;//take the middle value of three values
				if(sumL<measurements[ch][0])measurements[ch][0]=sumL;//take the lowest value of the middle values
				sumR=sumR-maxR-minR;//take the middle value of three values
				if(sumR<measurements[ch][1])measurements[ch][1]=sumR;//take the lowest value of the middle values
			}
		}
		for(uint8_t ch=0; ch<NUM_CHANNELS; ch++){
			//right channel
			if(measurements[ch][1]>4)measurements[ch][1]-=4;
			maxima[ch]=(measurements[ch][1]/69)*2; //convert value 0..1023 to 0..28, only use even states

			//left channel
			if(measurements[ch][0]>4)measurements[ch][0]-=4;
			uint8_t lef=(measurements[ch][0]/69)*2;
			if(lef!=0) lef=60-lef;
			minima[ch]=lef;

			checkRange(ch);
		}
		
		delay_ms(1500);
	}
}

void checkRange(uint8_t i){
	if((values[i]-minima[i]+NUM_STATES)%NUM_STATES > (maxima[i]-minima[i]+NUM_STATES)%NUM_STATES){
		//alarm! current value is outside the range between min and max!
		//find out which direction is the fastest way to get back between min and max
		if((minima[i]-values[i]+NUM_STATES)%NUM_STATES > (values[i]-maxima[i]+NUM_STATES)%NUM_STATES){
			//cli();
			orient[i]=0;//to the left towards max
			//sei();
		}
		else {
			//cli();
			orient[i]=1;
			//sei();
		}
	}
}

ISR(SPI_STC_vect){//this interrupt is called when a SPI transfer is complete (8 bit sent)
	uint8_t i=decnum;
	decnum--;
	if(i==0){
		PORTB|=0b100;//latch the contents of the shift register to output by rising RCK
		//moveDots();
	}else{
		SPDR=buffer[decnum];//send next byte to spi
	}
	
	if(orient[i]){//moving to the right
		if(values[i]==maxima[i]) orient[i]=0;//at the end turn the direction of movement to the left
		else {//move the dot one position to the right, towards max
			values[i]++;
			if(values[i]>=NUM_STATES) values[i]-=NUM_STATES;
			buffer[i]=map[values[i]];
		}
	}else{//moving to the left
		if(values[i]==minima[i]) orient[i]=1;//at the end turn the direction of movement to the right
		else {//move the dot one position to the left, towards min
			values[i]--;
			if(values[i]>=NUM_STATES) values[i]+=NUM_STATES;
			buffer[i]=map[values[i]];
		}
	}
}

ISR(TIMER0_COMPA_vect){
//	PORTD|=(1<<2); //debug
	PORTB&= ~0b100; //lower PB2=RCK
	decnum=NUM_DEKATRONS-1;
	SPDR=buffer[decnum]; //send out the first one over the spi
//	PORTD&= ~(1<<2);//debug
}
