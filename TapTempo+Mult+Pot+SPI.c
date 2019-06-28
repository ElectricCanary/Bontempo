/*
 * TapTempo+Multiplier.c
 *
 * Created: 20/06/2019
 *  Author: Electric Canary
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LEDDDR DDRB			//Define every pin, DDR and Ports
#define LEDPIN PINB0
#define LEDPORT PORTB
#define BUTTONDDR DDRC
#define BUTTONPIN PINC5
#define BUTTON PINC
#define BPIN 5
#define BUTTONPORT PORTC
#define TOGGLEPIN PINC3
#define TOGGLEPORT PORTC
#define TOGGLEDDR DDRC
#define MOSIPIN PINB3
#define SCKPIN PINB5
#define SSPIN PINB2
#define SPIDDR DDRB
#define SPIPORT PORTB
#define DEBOUNCE_TIME 100		//Define debounce Time in microseconds
#define MAXDELAY 1183.4928		//Define max & min delay time in ms
#define MINDELAY 37.4928
#define R 0.86					//the resistor in series with the time pot (in kOhms) echo dream 2: 0.86, disaster transport jr. : 1, Deep Blue Delay : 2.7 

volatile float mstempo;				//Volatile for variable shared by main and ADC interrupt
volatile uint8_t potvalue = 0;
volatile uint8_t previouspotvalue = 1;
volatile float mult;
volatile float previousmult;


uint8_t debounce(void)			//Debounce code, tells with certainty if the button is pushed (1) or released (0)
{
	if(bit_is_clear(BUTTON,BPIN))
	{
		_delay_us(DEBOUNCE_TIME);
		
		if(bit_is_clear(BUTTON,BPIN))
		{
			return(1);
		}
	}
	return (0);
}

uint8_t potdebounce(void)
{
	if (previouspotvalue != potvalue && previouspotvalue != potvalue + 1 && previouspotvalue != potvalue -1 && previouspotvalue != potvalue +2 && previouspotvalue != potvalue -2 && previouspotvalue != potvalue -3 && previouspotvalue != potvalue +3)
	{
			return(1);
	}
	return(0);
}

void SPI_MasterTransmit(uint8_t command,uint8_t data) //SPI transmission from data-sheet
{
/* Start transmission */
SPIPORT &= ~(1<<SSPIN);
SPDR = command;
/* Wait for transmission complete */
while(!(SPSR & (1<<SPIF)));
/* Start transmission */
SPDR = data;
/* Wait for transmission complete */
while(!(SPSR & (1<<SPIF)));
SPIPORT |= (1<<SSPIN);
}

int main(void)
{
	LEDDDR |= (1<< LEDPIN);			//LED pin output and set to low
	LEDPORT &= ~(1<<LEDPIN);
	BUTTONDDR &= ~(1<<BUTTONPIN);	//Button pin input and set to high
	BUTTONPORT |= (1<<BUTTONPIN);
	   
	sei();							//Activate interrupts
	
	ADMUX |= (1<<REFS0);			//ADC reference to Vcc 
	ADMUX |= (1<<MUX1);				//Selecting AD2 (PINC2)
	ADMUX |= (1<<ADLAR);			//8bit ADC
	ADCSRA |= (1<<ADIE);			//Activate ADC interrupts
	ADCSRA |= (1<<ADPS2);			//Setting ADC prescaler to 16
	ADCSRA |= (1<<ADEN);			//Turning ADC on
	ADCSRA |= (1<<ADSC);			//Starting Conversion
	
	TCCR1B |= 1<<CS10;				// Counter reference set to clock (1MHz no prescaler)
	
	SPIDDR |= (1<<MOSIPIN) | (1<<SCKPIN) | (1<<SSPIN);			//Setting MOSI, SCK & SS| as Output
	SPIPORT |= (1<<SSPIN);										//Setting SS| high
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); //Turning SPI on, as master, clock rate with 16 prescaler, LSB first

	
	float ms;
	
	unsigned int turns = 0;
	unsigned int ledturns = 0;
	
	unsigned int nbpress = 0;
	uint8_t laststate = 0;
	uint8_t tap = 0;
	
	uint8_t commandbyte = 0b00010001;
	uint8_t databyte;
	
    while(1)
    {
		
		if(potdebounce()==1)	//if the pot moves, pot overrides tap 
		{
			mstempo = (potvalue*((MAXDELAY-MINDELAY)/255)+MINDELAY); //Adjusting the pot range (0-255) to delay range
			databyte = potvalue;			 						//Adjusting the pot range to digital pot range
			SPI_MasterTransmit(commandbyte,databyte);				//Transmit 16bit word
			previouspotvalue = potvalue;
			tap = 0;
		}
		
		if(previousmult != mult && tap == 1)					//If toggle changes with no tap (in tap mode), update the digital pot value
		{
			databyte = ((((mstempo*mult)-29.7)/11.46)-R)*(255/100);
			SPI_MasterTransmit(commandbyte,databyte);
			previousmult = mult;
		}
		
		if(debounce() == 1 && laststate == 0 && nbpress == 0) // first time pressed = start counting
		{
			nbpress++;
			laststate = 1;
			TCNT1 = 0;
			turns = 0;	
		}
		
		if (TCNT1 > 1000)			// Counts turns, 1 turn = 1ms, limits counter to 1000
		{
			TCNT1 =0;
			turns ++;
			ledturns ++;
		}
		
		if (turns > 2000)			//if time out exceeded (2s), resets
		{
			nbpress = 0;
			turns = 0;
			laststate = 0;
		}
		
		if(debounce() == 0 && laststate == 1) // button released
		{
			laststate = 0;
		}
		
		if(debounce() == 1 && laststate == 0 && nbpress != 0) //second, third press...
		{
			if (TCNT1 > 500)		// rounds up to next turn if counter passed half its course
			{
				ms = turns + 1;
			}
			if (TCNT1 < 500) 
			{
				ms = turns;
			}
			if(nbpress == 1)				//If second press : tempo = time measured
			{
				mstempo = ms;
			}
			if(nbpress != 1)
			{
				mstempo = (mstempo + ms)/2; //average tempo if not second tap
			}
			if(mstempo > MAXDELAY)				//Clipping tempo to min & max delay
			{
				mstempo = MAXDELAY;
			}
		
			if(mstempo < MINDELAY)
			{
				mstempo = MINDELAY;
			}
			
			//databyte = ((mstempo - MINDELAY)*(255/(MAXDELAY-MINDELAY)))+ MINDELAY; //Adjusting tempo to digital pot's range (0-255)
			databyte = ((((mstempo*mult)-29.7)/11.46)-R)*(255/100);
			SPI_MasterTransmit(commandbyte,databyte);			//Transmit 16bit word
			nbpress++;						//restarts counting, for next tap
			laststate = 1;
			tap = 1;
			TCNT1 = 0;
			turns = 0;
		}
		
		
		
		if (ledturns > (mstempo*mult))				//Toggle Led every downbeat, multiply with toggle value
		{
			LEDPORT |= (1 << LEDPIN);
			_delay_ms(5);
			LEDPORT &= ~(1<<LEDPIN);
			ledturns = 0;
			
		}			
    }
	return(0);
}

ISR(ADC_vect)								// Interrupts when conversion is finished
{
	uint8_t togglevalue;
	switch(ADMUX)
	{
		case 0x62 :
		potvalue = ADCH;						//Stocks final ADC result
		ADMUX |= (1<<MUX0);
		break;
		
		case 0x63 :
		togglevalue = ADCH;
		if(togglevalue <= 127)
		{
			mult = 0.5;
		}
		//if(togglevalue >= 200)
		//{
		//	mult = 1;
		//}
		else
		{
			mult = 1;
		}
		ADMUX &= ~(1<<MUX0);
		break;
		
	}
	
	ADCSRA |= (1<<ADSC);					//Starts new conversion since previous one is finished
}