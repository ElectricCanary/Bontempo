/*
 * AVR TapTempo
 *
 * Created: 20/06/2019
 *  Author: Antoine Ricoux
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

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
#define DEBOUNCE_TIME 200		//Define debounce Time in microseconds
#define MAXDELAY 1138			//Define max & min delay time in ms : maxdelay = (11.46 * (R+maxpot))+29.7; mindelay = (11.46 * R)+29.7
#define MINDELAY 40

volatile float mstempo;				//Volatile for variable shared by main and other functions
volatile uint8_t potvalue = 0;
volatile uint8_t previouspotvalue = 1;
volatile uint8_t togglevalue;
volatile uint8_t multvar = 0;
volatile uint8_t speedvalue;
volatile uint8_t depthvalue;
volatile uint8_t wavetype;
volatile uint8_t wavevar;

void startup(void)
{
	SPI_MasterTransmit(0b00010001,0x7F);
	_delay_ms(400);
}

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

uint8_t potdebounce(void)			// Very goofy way to debounce the pot, for not interrupting the tap tempo randomly
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
SPIPORT &= ~(1<<SSPIN);		//Slave Select low to activate the digital pot
SPDR = command;				//Sending the command byte first (write to pot 1)
/* Wait for transmission complete */
while(!(SPSR & (1<<SPIF)));
/* Start transmission */
SPDR = data;				//Sending the data byte (wiper position between 0 and 255)
/* Wait for transmission complete */
while(!(SPSR & (1<<SPIF)));
SPIPORT |= (1<<SSPIN);		//Slave Select High to indicate we are finished (after 16clock counts)
}

void mod(uint8_t databyte, int speedturns)
{
	uint8_t datamod;
	uint8_t polarity = 0;
	uint8_t depth = round((depthvalue*127)/255); //calculating depth in digital pot values (max = 127 = approx. +-500ms)
	int speed = round(((speedvalue*1999)/255)+1);		//adjusting speed pot to ms value (1ms to 2s)
	
	if (wavetype == 0 && wavevar == 0)		//Square wave
	{
		if(speedturns > (speed/2))
		{
			switch (polarity)
			{
			case 0x00 :
			datamod = databyte + depth;
			SPI_MasterTransmit(0b00010001,datamod);
			polarity =1;								//next time around square down
			break;
			
			case 0x01 :
			datamod = databyte - depth;
			SPI_MasterTransmit(0b00010001,datamod);
			polarity =0;								//next time around square up
			break;
			}
			speedturns = 0;								//reinitialize turns every half period		
		}
	}
	
	if (wavetype == 1 && wavevar == 0)		//Sin wave
	{
		datamod = round( databyte + (depth * sin(((2*M_PI)/speed)*speedturns)));
		SPI_MasterTransmit(0b00010001,datamod);
		
		if(speedturns > speed)
		{
			speedturns = 0;
		}
	}
	
	if (wavetype == 2 && wavevar == 0)		//Triangle wave
	{
		if(speedturns <= (speed/2))
		{
			datamod = round((databyte-depth) + (speedturns*((depth*2)/(speed/2)))); //for the first half increase time gradually (respecting depth and speed values)
			SPI_MasterTransmit(0b00010001,datamod);
		}
		
		if(speedturns > (speed/2))
		{
			datamod = round((databyte+depth) - (speedturns*((depth*2)/(speed/2)))); //for the second half decrease time gradually 
			SPI_MasterTransmit(0b00010001,datamod);
		}
		
		if(speedturns > speed)
		{
			speedturns = 0;				//not forgetting to reinitialize the ms turns once a phase is done
		}
	}
	
	if(wavetype == 0 && wavevar == 1)	//Ramp up
	{
		if (speedturns <= speed)
		{
			datamod = round((databyte-depth) + (speedturns * ((depth*2)/speed)));
			SPI_MasterTransmit(0b00010001,datamod);
		}
		
		if(speedturns > speed)
		{
			speedturns = 0;
		}
	}
	
	if (wavetype == 1 && wavevar == 1)	//Ramp Down
	{
		if (speedturns <= speed)
		{
			datamod = round((databyte+depth) - (speedturns * ((depth*2)/speed)));
			SPI_MasterTransmit(0b00010001,datamod);
		}
		
		if(speedturns > speed)
		{
			speedturns = 0;
		}
	}
	
	if (wavetype == 2 && wavevar == 1)	//Random
	{
		datamod = round (databyte + ((rand() % ((depth*2)+1)) - depth));
		SPI_MasterTransmit(0b00010001,datamod);
	}
}

float mult(void)
{
	if(togglevalue <= 84 && multvar == 0)
	{
		return(0.5);						//eighth
	}
	
	if(togglevalue >= 174 && multvar == 0)
	{
		return(1);							//fourth
	}
	
	if(togglevalue > 84 && togglevalue < 174 && multvar == 0)
	{
		return(0.75);						//dotted eighth
	}
	
	if(togglevalue <= 84 && multvar == 1)
	{
		return(1/3);						//triplet
	}
	
	if(togglevalue >= 174 && multvar == 1)
	{
		return(1/4);						//sixteenth
	}
	
	if(togglevalue > 84 && togglevalue < 174 && multvar == 1)
	{
		return(1/6);						//sextuplet
	}
}


int main()
{
	SPIDDR |= (1<<MOSIPIN) | (1<<SCKPIN) | (1<<SSPIN);			//Setting MOSI, SCK & SS| as Output
	SPIPORT |= (1<<SSPIN);										//Setting SS| high
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); //Turning SPI on, as master, clock rate with 16 prescaler, LSB first
	
	startup();						//Waiting 400ms with pot at 50k to avoid boot problem with PT2399
	
	LEDDDR |= (1<< LEDPIN);			//LED pin output and set to low
	LEDPORT &= ~(1<<LEDPIN);
	BUTTONDDR &= ~(1<<BUTTONPIN);	//Button pin input and set to high
	BUTTONPORT |= (1<<BUTTONPIN);
	
	ADMUX |= (1<<REFS0);			//ADC reference to Vcc 
	ADMUX |= (1<<ADLAR);			//for 8bit ADC
	ADCSRA |= (1<<ADIE);			//Activate ADC interrupts
	ADCSRA |= (1<<ADPS2);			//Setting ADC prescaler to 16
	ADCSRA |= (1<<ADEN);			//Turning ADC on
	
	TCCR1B |= 1<<CS10;				// Counter reference set to clock (1MHz)
	
	float ms;
	float data;
	
	unsigned int turns = 0;		//1turn = 1ms = 1000 TCNT1
	unsigned int ledturns = 0;
	unsigned int speedturns = 0;
	
	unsigned int nbpress = 0;	// Number of presses during the tap tempo sequence
	uint8_t laststate = 0;		//Previous state of the button
	uint8_t tap = 0;			//Buton control (0) or Tap tempo (1) ? 
	
	uint8_t commandbyte = 0b00010001;
	uint8_t databyte;
	
	float previousmult;
	uint8_t previouswave;
	
	startup();					//Waiting 400ms with pot at 50k to avoid boot problem with PT2399
	
	sei();							//Activate interrupts
	ADCSRA |= (1<<ADSC);			//Starting Conversion
	
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
			speedturns++;
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
				mstempo = (mstempo + ms)/2; //average tempo if not second time pressing the button
			}
			if(mstempo > MAXDELAY)				//Clipping tempo to min & max delay
			{
				mstempo = MAXDELAY;
			}
		
			if(mstempo < MINDELAY)
			{
				mstempo = MINDELAY;
			}
			
			data = (((mstempo*mult())-MINDELAY)/((MAXDELAY-MINDELAY)/100))*2.55;		//this calculates the digital pot value (between 0 and 255) from the tempo
			databyte = round(data);													//rounding the result
			SPI_MasterTransmit(commandbyte,databyte);			//Transmits 16bit word
			nbpress++;						//restarts counting, for next tap
			laststate = 1;
			tap = 1;
			TCNT1 = 0;
			turns = 0;
			ms=0;
		}
		
		if(previousmult != mult() && tap == 1)					//If toggle changes (in tap mode), update the digital pot value
		{
			if(debounce() == 1)									//If toggle position changes while button pressed, use the other 3 divisions
			{
				multvar = 1;
				nbpress = 0;									//Not counting the press as a tap
				laststate = 0;
			}
			
			if(debounce == 0)
			{
				multvar = 0;									//Button not pressed = not using alternative divisions
			}
			data = (((mstempo*mult())-MINDELAY)/((MAXDELAY-MINDELAY)/100))*2.55;	//Sending new division to digital pot
			databyte = round(data);
			SPI_MasterTransmit(commandbyte,databyte);
			previousmult = mult();								//Updating the previous division detected		
		}
		
		if(previouswave != wavetype)		//taking the wave variation in count if the wave toggle is moved
		{
			if (debounce() ==1)				//If button pressed while the wave toggle changed = use alternative waveforms
			{
				wavevar = 1;
				nbpress = 0;				//don't count press as tap for tap tempo
				laststate =0;
			}
			if (debounce()==0)				//if button not pressed while wave toggle changed = don't use alternative waveforms
			{
				wavevar = 0;
			}
			previouswave = wavetype;
		}
		
		mod(databyte,speedturns);			//engaging modulation
		
		if(tap==0)
		{
			LEDPORT |= (1 << LEDPIN);					// LED constantly on if the delay is pot-controlled 
		}
		if (ledturns > mstempo && tap==1)				//Toggles Led every downbeat if delay tap tempo controlled
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
	uint8_t wavevalue;
	switch(ADMUX)							//Switch each time between every ADC input necessary
	{
		case 0x60 :							//Get Speed pot value
		speedvalue = ADCH;
		ADMUX |= (1<<MUX0);					//Change to ADC1 for next conversion
		break;
		
		case 0x61 :
		depthvalue = ADCH;					//Get Depth Pot value
		ADMUX &= ~(1<<MUX0);				//Change to ADC2
		ADMUX |= (1<<MUX1);
		break;
		
		case 0x62 :
		potvalue = ADCH;					//Get Time pot value
		ADMUX |= (1<<MUX0);					//Changing to ADC3
		break;
		
		case 0x63 :
		togglevalue = ADCH;					//get time division toggle value
		ADMUX &= ~(1<<MUX0);				//Changing to ADC4
		ADMUX &= ~(1<<MUX1);
		ADMUX |= (1<<MUX2);
		break;
		
		case 0x64 :
		wavevalue = ADCH;					//Get waveform toggle value
		if (wavevalue <= 84)
		{
			wavetype = 0;
		}
		if (wavevalue >= 171)
		{
			wavetype = 1;
		}
		else
		{
			wavetype = 2;
		}
		ADMUX &= ~(1<<MUX2);				//Changing to ADC0
		break;
	}
	
	ADCSRA |= (1<<ADSC);					//Starts new conversion since previous one is finished
}