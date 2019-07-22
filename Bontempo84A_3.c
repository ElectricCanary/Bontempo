/*
 * Bontempo84_2.c
 *
 * Created: 15/07/2019 21:30:51
 * Author : Antoine
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define LEDPORT PORTB		//Defining every pin, DDR & port used
#define LEDPIN	PINB0
#define LEDDDR	DDRB
#define BUTTONPIN PINB1
#define BUTTONPORT PORTB
#define BUTTONSFR PINB
#define BUTTONBV 1
#define SPIDDR DDRA
#define SPIPORT PORTA
#define CSPORT PORTB
#define CSDDR DDRB
#define CSPIN PINB2
#define CLKPIN PINA4
#define DATAPIN PINA5
#define N_ARRAY 255
#define DEBOUNCE_TIME 500	//Tap button debounce time in microseconds
#define DELAY_MIN 41

volatile uint8_t timevalue;
volatile uint8_t div;
volatile uint16_t msturns = 0;
volatile uint16_t ledturns = 0;

PROGMEM prog_uint16_t tempo[] ={41,42,44,47,52,58,63,68,73,77,82,86,90,95,99,104,108,112,117,121,			//The tempo values are separated by class to save precious SRAM space
			126,130,135,140,145,149,154,159,164,168,173,177,182,186,191,195,200,204,209,213,
			217,221,225,230,234,238,243,247,252,256,260,265,269,274,278,282,287,292,296,301,
			305,310,315,319,323,328,332,337,341,345,350,354,359,363,367,372,376,380,384,388,
			393,397,402,406,410,415,419,423,427,431,436,440,444,449,453,458,462,466,470,475,
			479,483,488,492,496,501,505,510,514,518,523,527,531,535,540,544,548,553,557,561,
			566,570,574,579,583,587,591,595,600,604,608,613,617,621,626,630,634,638,642,646,
			651,655,659,663,668,672,676,681,685,688,693,697,701,706,710,718,723,727,731,735,
			740,744,748,752,756,761,765,770,773,778,782,786,791,795,799,803,807,812,816,820,
			824,829,833,837,841,846,850,854,858,862,866,871,875,879,883,888,892,896,900,905,
			909,913,917,921,925,929,933,938,942,946,951,955,959,963,967,971,976,980,984,988,
			992,996,1000,1005,1009,1013,1018,1022,1027,1028,1033,1038,1042,1046,1051,1055,1059,1063,1068,1071,
			1076,1080,1084,1088,1093,1097,1101,1105,1109,1113,1118,1122,1126,1130,1134};

uint8_t debounce(void)
{
	if (bit_is_clear(BUTTONSFR,BUTTONBV))		//if button pressed
	{
		_delay_us(DEBOUNCE_TIME);				//wait debounce time
		if (bit_is_clear(BUTTONSFR,BUTTONBV))	//if still pressed return 1
		{
			return(1);
		}
		else									//if not return 0
		{
			return(0);
		}
	}
	else
	{
		return(0);
	}
}

//this is a function I found on Internet, it searches for the closest number in a array, I modified it so it returns position in array (which will be the wiper position) instead of value

int getClosest(int, int, int, int, int); 
  
// Returns element closest to target in arr[] 
int findClosest(int target) 
{ 
    // Corner cases 
    if (target <= pgm_read_word_near(tempo + 0)) 
        return 0; 
    if (target >= pgm_read_word_near(tempo + (N_ARRAY-1))) 
        return N_ARRAY - 1; 
  
    // Doing binary search 
    int i = 0, j = N_ARRAY, mid = 0; 
    while (i < j) { 
        mid = (i + j) / 2; 
  
        if (pgm_read_word_near(tempo + mid) == target) 
            return mid; 
  
        /* If target is less than array element, 
            then search in left */
        if (target < pgm_read_word_near(tempo + mid)) { 
  
            // If target is greater than previous 
            // to mid, return closest of two 
            if (mid > 0 && target > pgm_read_word_near(tempo + (mid - 1))) 
                return getClosest(pgm_read_word_near(tempo + (mid - 1)), 
                                  pgm_read_word_near(tempo + mid), target, mid-1, mid); 
  
            /* Repeat for left half */
            j = mid; 
        } 
  
        // If target is greater than mid 
        else { 
            if (mid < N_ARRAY - 1 && target < pgm_read_word_near(tempo + (mid + 1))) 
                return getClosest(pgm_read_word_near(tempo + mid), 
                                  pgm_read_word_near(tempo + (mid + 1)), target, mid, mid+1); 
            // update i 
            i = mid + 1;  
        } 
    } 
  
    // Only single element left after search 
    return mid; 
} 
  
// Method to compare which one is the more close. 
// We find the closest by taking the difference 
// between the target and both values. It assumes 
// that val2 is greater than val1 and target lies 
// between these two. 
int getClosest(int val1, int val2, 
               int target, int n1, int n2) 
{ 
    if (target - val1 >= val2 - target) 
        return n2; 
    else
        return n1; 
} 

void SPI_Transmit(uint8_t data)			//function to transmit 8 bit digital pot wiper position via the SPI
{
	USIDR = 0b00010011;					//command byte  : write to all pots
	CSPORT &= ~(1<<CSPIN);				//Chip select pin set low : chip is selected
	USISR = _BV(USIOIF);
	while((USISR &_BV(USIOIF))==0)
	{
		USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	}
	USIDR = data;						//wiper position in USI data register
	USISR = _BV(USIOIF);
	while((USISR &_BV(USIOIF))==0)
	{
		USICR = (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC);
	}
	CSPORT |= (1<<CSPIN);				//Chip select pin set high after 16 clock cycles: transmission complete
}

ISR(ADC_vect)					//ADC interrupt
{
	uint8_t divtogglevalue;
	switch(ADMUX)
	{
		case 0x00:
		timevalue = ADCH;		//Stocking 8-bit value
		ADMUX |= (1<<MUX0);
		break;
		
		case 0x01:
		divtogglevalue = ADCH;
		
		if (divtogglevalue < 20)
		{
			div = 0;
		}
		
		if (divtogglevalue > 20 && divtogglevalue < 230)
		{
			div = 1;
		}
		
		if (divtogglevalue > 230)
		{
			div = 2;
		}
		
		ADMUX &= ~(1<<MUX0);
		break;
	}
		
		ADCSRA |= (1<<ADSC);	//Restarting conversion
}

void blink1(void)		//toggles led to verify interactions
{
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
}

ISR(TIM0_OVF_vect)
{
	msturns++;
	ledturns++;
}

int main(void)
{
	SPIDDR |= (1<<CLKPIN) | (1<<DATAPIN); //SPI pins as output
	CSDDR |= (1<<CSPIN);
	CSPORT |= (1<<CSPIN);	//Chip select pin high (not selected)
	
	LEDDDR |= (1<<LEDPIN);	//LED pin as output
	BUTTONPORT |= (1<<BUTTONPIN);	//Tap Button pin set to high 
	
	OCR0A = 125;
	TCNT0 = 0;
	TCCR0A |= (1<<WGM00) | (1<<WGM01);
	TIMSK0 |= (1<<TOIE0);
	TCCR0B |= (1<<WGM02) | (1<<CS01);
	
	ADCSRA |= (1<<ADEN) | (1<<ADIE); //Enabling ADC, 64 prescaler, ADC interrupts enable
	ADCSRB |= (1<<ADLAR); //8bit conversion
	
	uint16_t mstempo;		//The current tempo tapped
	uint16_t delaymax = 1184;
	
	uint16_t nbtap = 0;		//number of times tapped during the current sequence
	uint8_t laststate = 0;	//last state of the button
	uint8_t tap = 0;		//tap controlled (1) or pot controlled (0)
	
	uint8_t data;	//to store digital pot wiper position
	
	uint8_t previoustimevalue;	//used to know if the time pot moved
	uint8_t previousdiv = 3;	//used to know if the div toggle moved
	float divmult;				//the div tempo multiplicand
	
	uint8_t cleanmode = eeprom_read_byte((uint8_t*)0);	//read clean mode status from eeprom
	uint8_t dual2399 = eeprom_read_byte((uint8_t*)1);	//read dual mode status from eeprom
	
		if (debounce()==1)				//if button pressed at startup
		{
			LEDPORT |= (1<<LEDPIN);
			_delay_ms(2500);
			
			if (debounce() == 1)
			{
				_delay_ms(1500);
				if (debounce()==1)
				{
					blink1();			//if pressed long enough blink twice
					blink1();
					
					for (uint16_t c = 0; c < 2000; c++ )	//if button released during the 2s following the blinking
					{
						_delay_ms(1);
						if (debounce() == 0)
						{
							switch (cleanmode)
							{
								case 0x01:					//toggle the clean mode state and store it to eeprom
								cleanmode = 0;
								eeprom_update_byte((uint8_t*)0,0);
								break;
								
								default:
								cleanmode = 1;
								eeprom_update_byte((uint8_t*)0,1);
								break;
							}
							blink1();
							blink1();
							break;
						}
					}
				}
			}
			
			if (debounce() == 1)						//if button still pressed 2s after blinking
			{
				_delay_ms(4500);						//wait 4.5s
				if (debounce()==1)
				{
					blink1();							//blink 4 times
					blink1();
					blink1();
					blink1();
					
					for (uint16_t b = 0; b < 2000; b++ )	//if button released during the 2s after blinking
					{
						_delay_ms(1);
						if (debounce() == 0)
						{
							switch (dual2399)				//toggle dual2399 state and stock new state to eeprom
							{
								case 0x01:
								dual2399 = 0;
								eeprom_update_byte((uint8_t*)1,0);
								break;
								
								default:
								dual2399 = 1;
								eeprom_update_byte((uint8_t*)1,1);
								break;
							}
							blink1();
							blink1();
							blink1();
							blink1();
							break;
						}
					}
				}
			}
		}
	
	if (cleanmode == 1)		//if in clean mode the maximum delay is now 600ms
	{
		delaymax = 600;
	}
	
	sei();			//activating interrupts
	
	ADCSRA |= (1<<ADSC); //Starting AD conversion
	
    while (1) 
    {
		
		if (tap == 0 || abs(previoustimevalue-timevalue) >= 13)	//if pot move of more than 5%, changing to pot control
		{
			if (cleanmode == 1)		//if clean mode active time pot course divided per 2
			{
				timevalue = (timevalue*127)/255;
			}
			
			SPI_Transmit(timevalue);
			previoustimevalue = timevalue;
			tap = 0;
		}
		
		if (previousdiv != div)		//if first time or if div toggle changed position : update div
		{
			if (debounce() == 0)	//if tap button not pressed while changing 3 first div
			{
				if (div == 0)
				{
					divmult = 1;	//fourth
				}
			
				if (div == 1)
				{
					divmult = 0.5;	//eighth
				}
				
				if (div == 2)
				{
					divmult = 0.75;	//dotted eighth
				}
			}
			
			if (debounce() == 1)	//if tap button pressed while changing 3 last div
			{
				if (div == 0)
				{
					divmult = 0.333333;	//triplet
				}
				
				if (div == 1)
				{
					divmult = 0.1666666;	//sextuplet
				}
				
				if (div == 2)
				{
					divmult = 0.25;	//sixteenth
				}
				
				nbtap = 0;			//don't count press as tap
				laststate = 1;
			}
			
			if (dual2399 == 1)		//if dual2399 mode is active = divide any div per 2
			{
				divmult = divmult / 2;
			}
			
			if (tap == 1)	//if in tap control, update digital pot value
			{
				data = round(mstempo * divmult);
				SPI_Transmit(data);
			}
			
			previousdiv = div;	//reseting previousdiv to detect next move
		}
		
		if (debounce()==1 && laststate==0 && nbtap==0) //first tap
		{
			TCNT1 = 0;			//starts counting
			msturns=0;
			nbtap++;
			laststate = 1;
		}
		
		if (msturns > ((delaymax/divmult)+800)) //if too long between taps (2s) : resets
		{
			msturns = 0;
			nbtap = 0;
		}
		
		if (debounce()==0 && laststate ==1)	//release tap button
		{
			laststate = 0;
		}
		
		if (debounce()==1 && laststate==0 && nbtap!=0) //not first tap
		{
			if (TCNT1 > 500)	//if timer1 passed half its course add a ms
			{
				msturns = msturns +1;
			}
			
			if (nbtap==1)		//if second tap, tempo = time elapsed between the 2 button press
			{
				mstempo = msturns * divmult;
			}
			
			if (nbtap!=1)		//if not second tap, average every tap
			{
				mstempo = round((mstempo + (msturns*divmult)) / 2);
			}
			
			if (mstempo < DELAY_MIN)		//clipping current tempo to min and max tempo
			{
				mstempo = DELAY_MIN;
			}
			
			if (mstempo > delaymax)
			{
				mstempo = delaymax;
			}
			
			data = findClosest(mstempo);			//wiper position returned by position in array
			
			SPI_Transmit(data);		//sending wiper position to digital pot
			
			nbtap++;			//updating number of tap and last state of tap button
			laststate = 1;
			TCNT1 = 0;			//reseting counter and ms
			msturns = 0;
			tap = 1;			//now in tap control mode
		}
		
		if (tap == 0)
		{
			LEDPORT |= (1<<LEDPIN);
		}
		
		if (ledturns > mstempo && tap==1)	//turns LED on for 8ms every downbeat
		{
			ledturns = 0;
			LEDPORT |= (1<<LEDPIN);
			_delay_ms(8);
			LEDPORT &= ~(1<<LEDPIN);
		}
		
    }
}
