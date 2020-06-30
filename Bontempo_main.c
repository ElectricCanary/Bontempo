/*
 * Bontempo
 * June 2020
 * Author : Antoine Ricoux for Electric Canary
 https://electric-canary.com/bontempo
 support@electric-canary.com
 
 
 This is a tap tempo with modulation for PT2399-based delay
 This code is created for the ATtiny84A and digital potentiometer MCP41100 (or MCP42100)
 
 Pin configuration :
 
 1: VCC
 2: LED +
 3: Tap Momentary Button
 4: Double Time
 5: SPI Chip Select
 6: Waveform Toggle (On-Off-On)
 7: Mod PWM Out
 8: SPI Data Out
 9: SPI Clock Out
 10: Mod Depth Potentiometer
 11: Mod Speed Potentiometer 
 12: Time Div Toggle (On-Off-On)
 13: Time Potentiometer
 14: GND
 
 Recommended fuses : 
 Reset Pin Enabled, Double Time Disabled: Low Fuse = 0xd2	High Fuse = 0xdc
 (!)Reset Pin Disabled(!), Double Time Enabled: Low Fuse = 0xd2	High Fuse = 0x5c
 
 Thanks to Florian Dupeyron for helping me debug some stuff
 
 This code is shared shared under a BY-NC-SA Creative Commons License
 Go here for complete license : https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
 
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define LEDPORT PORTB		//Defining every pin, DDR & port used
#define LEDPIN	PINB0
#define LEDDDR	DDRB
#define DOUBLEPIN PINB3
#define DOUBLEPORT PORTB
#define DOUBLESFR PINB
#define DOUBLEBV 3
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
#define PWMPIN PINA6
#define PWMBV 6
#define PWMDDR DDRA
#define DIVPORT PORTA
#define DIVPIN PINA1
#define MODPORT PORTA
#define MODSFR PINA
#define WAVEBV 7
#define WAVEPIN PINA7
#define SPEEDPIN PINA2
#define SPEEDBV 2
#define DEPTHPIN PINA3
#define DEPTHBV 3
#define N_ARRAY 256
#define DEBOUNCE_TIME 800	//Tap button debounce time in microseconds
#define RAND_MAX 0x7fff

volatile unsigned long timevalue;
volatile uint8_t divtogglevalue;
volatile uint16_t msturns = 0;
volatile uint16_t ledturns = 0;
volatile uint8_t inc = 0;
volatile uint8_t speedvalue;
volatile unsigned long depthvalue;
volatile uint8_t wavevalue;
volatile unsigned long currentinc = 500;
volatile uint16_t pwm;
volatile int16_t offset;
volatile uint16_t speed;

PROGMEM const uint16_t sine[] = {					//Sine wavetable
			200,205,210,215,220,224,229,234,239,244,249,253,258,263,267,272,
			277,281,286,290,294,299,303,307,311,315,319,323,327,331,334,338,
			341,345,348,351,355,358,361,364,366,369,372,374,376,379,381,383,
			385,387,388,390,391,393,394,395,396,397,398,398,399,399,400,400,
			400,400,400,399,399,398,398,397,396,395,394,393,391,390,388,387,
			385,383,381,379,376,374,372,369,366,364,361,358,355,351,348,345,
			341,338,334,331,327,323,319,315,311,307,303,299,294,290,286,281,
			277,272,267,263,258
};

PROGMEM const uint8_t smallsine[] = { 
			253,249,244,239,234,229,224,220,215,210,205,
			200,195,190,185,180,176,171,166,161,156,151,147,142,137,133,128,
			123,119,114,110,106,101,97,93,89,85,81,77,73,69,66,62,
			59,55,52,49,45,42,39,36,34,31,28,26,24,21,19,17,
			15,13,12,10,9,7,6,5,4,3,2,2,1,1,0,0,
			0,0,0,1,1,2,2,3,4,5,6,7,9,10,12,13,
			15,17,19,21,24,26,28,31,34,36,39,42,45,49,52,55,
			59,62,66,69,73,77,81,85,89,93,97,101,106,110,114,119,
			123,128,133,137,142,17,151,156,161,166,171,176,180,185,190,195
	};

const uint16_t triangle[] = {			//triangle wavetable
			256,259,263,266,269,272,275,278,281,284,288,291,294,297,300,
			303,306,309,313,316,319,322,325,328,331,334,338,341,344,347,350,
			353,356,359,363,366,369,372,375,378,381,384,388,391,394,397,400,
			397,394,391,388,384,381,378,375,372,369,366,363,359,356,353,350,
			347,344,341,338,334,331,328,325,322,319,316,313,309,306,303,300,
			297,294,291,288,284,281,278,275,272,269,266,263,259,256
};

PROGMEM const uint8_t smalltriangle[] = {			//triangle wavetable
			3,6,9,13,16,19,22,25,28,31,34,38,41,44,47,50,
			53,56,59,63,66,69,72,75,78,81,84,88,91,94,97,100,
			103,106,109,113,116,119,122,125,128,131,134,138,141,144,147,150,
			153,156,159,163,166,169,172,175,178,181,184,188,191,194,197,200,
			203,206,209,213,216,219,222,225,228,231,234,238,241,244,247,250,
			253
};

PROGMEM const uint8_t smallsaw[] = {				//sawtooth wavetable
			0,2,3,5,6,8,9,11,13,14,16,17,19,20,22,24,
			25,27,28,30,31,33,35,36,38,39,41,42,44,45,47,49,
			50,52,53,55,56,58,60,61,63,64,66,67,69,71,72,74,
			75,77,78,80,82,83,85,86,88,89,91,93,94,96,97,99,
			100,102,104,105,107,108,110,111,113,115,116,118,119,121,122,124,
			125,127,129,130,132,133,135,136,138,140,141,143,144,146,147,149,
			151,152,154,155,157,158,160,162,163,165,166,168,169,171,173,174,
			176,177,179,180,182,184,185,187,188,190,191,193,195,196,198,199,
			201,202,204,205,207,209,210,212,213,215,216,218,220,221,223,224,
			226,227,229,231,232,234,235,237,238,240,242,243,245,246,248,249,
			251,253,254
};

PROGMEM const uint16_t saw[] = {				//sawtooth wavetable
			256,257,259,260,262,264,265,267,268,270,271,273,275,
			276,278,279,281,282,284,285,287,289,290,292,293,295,296,298,300,
			301,303,304,306,307,309,311,312,314,315,317,318,320,322,323,325,
			326,328,329,331,333,334,336,337,339,340,342,344,345,347,348,350,
			351,353,355,356,358,359,361,362,364,365,367,369,370,372,373,375,
			376,378,380,381,383,384,386,387,389,391,392,394,395,397,398,400
};

PROGMEM const uint16_t tempo[] = {	//wiper position to tempo conversion chart
			51,52,54,59,65,71,77,83,89,94,99,105,110,115,120,125,131,136,141,146,
			152,157,162,168,173,179,185,190,196,201,206,211,217,222,227,232,237,242,247,252,
			257,262,267,272,277,282,287,292,298,302,307,312,317,322,327,332,338,343,348,353,
			359,364,369,374,379,384,389,394,399,405,410,415,420,425,430,435,439,444,449,454,
			459,464,469,474,479,484,488,493,498,503,507,512,517,522,527,532,537,542,547,551,
			556,561,566,571,576,581,586,591,596,601,606,611,615,620,625,630,635,640,645,650,
			655,660,664,669,674,679,684,688,693,698,703,708,713,718,722,727,732,737,741,746,
			750,755,760,765,770,775,779,784,789,794,798,803,808,812,817,822,827,831,836,841,
			846,850,855,860,865,869,874,879,884,888,893,898,903,908,912,917,922,926,931,936,
			941,945,950,955,960,965,969,974,979,983,988,993,997,1002,1007,1012,1017,1021,1026,1031,
			1036,1040,1045,1049,1054,1058,1063,1068,1073,1078,1082,1087,1092,1097,1101,1106,1110,1115,1120,1124,
			1129,1133,1138,1142,1147,1152,1157,1162,1166,1171,1175,1180,1185,1189,1194,1199,1203,1208,1212,1217,
			1222,1226,1231,1236,1241,1245,1250,1254,1259,1264,1268,1273,1277,1282,1287,1291};

uint8_t debounce(void)					//tells with certainty if button is pressed
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

int getClosest(int, int, int, int, int); 
  
// Returns element closest to target in arr[] and return wiper position
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

void blink1(void)		//toggles led to verify interactions
{
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
}

void fastblink1(void)		//toggles led to verify interactions
{
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(100);
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(100);
}

uint8_t doubletime(void)
{
	return(bit_is_set(DOUBLESFR, DOUBLEBV));
}

void Timerinit(void)
{
	TCNT0 = 0;				//timer0, fast PWM, OCRA as TOP, enable compare A interrupt, 64 prescaler
	OCR0A = 100;
	TCCR0A |= (1<<WGM00) | (1<<WGM01);
	TIMSK0 |= (1<<OCIE0A);
	TCCR0B |= (1<<WGM02) | (1<<CS02) | (1<<CS00);
	
	ICR1 = 999;	//timer1, fast PWM, ICR1 as TOP, enable overflow interrupt and compare A interrupt, no prescaler
	TCNT1 = 0;
	
	OCR1A = 300;
	TCCR1A |= (1<<COM1A1);
	TIMSK1 |= (1<<OCIE1A);
	
	TCCR1A |= (1<<WGM11);
	TIMSK1 |= (1<<TOIE1);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11);
}

void IOinit(void)
{
	LEDDDR |= (1<<LEDPIN);	//LED pins as output
	PWMDDR |= (1<<PWMPIN);	//PWM pin as output
	SPIDDR |= (1<<CLKPIN) | (1<<DATAPIN); //SPI pins as output
	CSDDR |= (1<<CSPIN);
	
	BUTTONPORT |= (1<<BUTTONPIN); //Connecting Pull-Up Resistors
	MODPORT |= (1<<WAVEPIN);
	DIVPORT |= (1<<DIVPIN);
	CSPORT |= (1<<CSPIN);	//Chip select pin high (not selected)
}

void ADCinit(void)
{
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enabling ADC, 16 prescaler, ADC interrupts enable
	ADCSRB |= (1<<ADLAR); //8bit conversion
}

ISR(ADC_vect)					//ADC interrupt
{
	switch(ADMUX)
	{
		case 0x00:
		timevalue = ADCH;		//Stocking 8-bit value
		ADMUX |= (1<<MUX0);		//changing to next ADC Channel
		break;
	
		case 0x01:
		divtogglevalue = ADCH;
		ADMUX &= ~(1<<MUX0);
		ADMUX |= (1<<MUX1);
		break;
		
		case 0x02:
		speedvalue = ADCH;
		ADMUX |= (1<<MUX0);
		break;
	
		case 0x03:
		depthvalue = ADCH;
		ADMUX |= (1<<MUX2);
		break;
		
		case 0x07:
		wavevalue = ADCH;
		ADMUX &= ~(1<<MUX0) & ~(1<<MUX1) & ~(1<<MUX2);
		break;
	}
	ADCSRA |= (1<<ADSC);	//Restarting conversion
}

ISR(TIM1_OVF_vect)
{
	msturns++;	//ms increment if timer1 overflow
	ledturns++;
}

ISR(TIM1_COMPA_vect)
{
	cli();		//updates mod pwm duty cycle
	OCR1A = pwm;
	sei();
}

ISR(TIM0_COMPA_vect)
{
	inc++;		//increment position in wavetable
	cli();
	OCR0A = speed;	//update mod speed
	sei();
}

int main(void)
{	
	_delay_ms(1000); //Waiting for PT2399 to start-up
	
	IOinit();
	Timerinit();
	ADCinit();
	
	uint16_t divtempo;		//The current tempo tapped (it will  be multiplied by the tempo div)
	uint16_t mstempo = eeprom_read_word((uint16_t*)2);		//tempo for toggling LED (not influenced by tempo div)
	//uint8_t delaymin = 51;
	uint16_t delaymax = 1291;	//maximum tempo if not in clean mode
	int8_t pwmfine[] = {25,21,16,11,6,0,-7,-11,-15,-19,-23};	//pwm values for +-5ms offset
	uint8_t delta;
	unsigned long cleantimevalue;
	
	uint16_t nbtap = 0;		//number of times tapped during the current sequence
	uint8_t laststate = 0;	//last state of the button
	uint8_t tap = eeprom_read_byte((uint8_t*)1);		//tap controlled (1) or pot controlled (0)
	uint8_t tapping = 0;	//led follow tap (1) or follow tempo (0)
	
	uint8_t data;	//to store digital pot wiper position
	
	uint8_t previoustimevalue;	//to know if the time pot moved
	uint16_t previousdiv = 300;	//to know if the div toggle moved
	float divmult = 1;				//the div tempo multiplicand
	
	uint16_t previouswave = 300;	//to know if waveform toggle moved
	uint8_t wavetype = 0;			//this value selects one of the 6 waveforms
	
	uint8_t cleanmode = eeprom_read_byte((uint8_t*)0);	//read clean mode status from eeprom
	
	uint16_t block = 0; //is used for delaying the time and tap button read at startup, (the eeprom time save is not taken into account otherwise) 
	
	uint8_t speedpresetactive = 0;	//is the speed preset value used?
	uint8_t depthpresetactive = 0;	//is the depth preset value used?
	uint8_t presetspeed;	//for storing preset values
	uint32_t presetdepth;
	uint8_t previousspeed;	//for detecting if speed pot moved
	uint32_t previousdepth;	//for detecting if depth pot moved
	uint8_t previousdoubletime = doubletime(); //for detecting if double time pin changed state

		if (debounce()==1)
		{
			LEDPORT &= ~(1<<LEDPIN);
			blink1();			//if pressed long enough blink twice
			
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
					fastblink1();
					break;
				}
			}
		}
	
	if (cleanmode == 1)		//if in clean mode the maximum delay is now 600ms
	{
		delaymax = 600;
	}
	
	sei();			//activating interrupts
	
	ADCSRA |= (1<<ADSC); //Starting first AD conversion
	
    while (1) 
    {
		//---------PWM OUTPUT
		
		
		if (depthpresetactive == 1)
		{
			offset = (300 - ((presetdepth * 300)/255));	//updating mod pwm value
		}
		if (depthpresetactive == 0 || abs(depthvalue - previousdepth) >= 13)
		{
			depthpresetactive = 0;
			offset = (300 - ((depthvalue * 300)/255));	//updating mod pwm value
		}
		
		if (tap == 1)
		{
			delta = divtempo - tempo[data];	//difference between tempo tapped and closest tempo in conversion chart
			offset = offset + pwmfine[delta + 5];	//compensate with pwm if tap tempo is active
		}
		
		if (depthpresetactive == 1)
		{
			pwm = (((100 + currentinc) * presetdepth)/255) + offset;
		}
		else
		{
			pwm = (((100 + currentinc) * depthvalue)/255) + offset;
		}
		
		
		
		
		//-----------MODULATION SPEED & WAVEFORMS
		
		if (speedpresetactive == 1)
		{
			speed = 255 - presetspeed; //updating mod speed
		}
		if (speedpresetactive == 0 || abs(speedvalue - previousspeed) >= 13)
		{
			speedpresetactive = 0;
			speed = 255 - speedvalue; //updating mod speed
		}
		
		if (speed < 3) //limiting speed, it bugs if lower than 3
		{
			speed = 3;
		}
	
		if (abs(previouswave-wavevalue) > 50)	//if wave toggle moves, update waveform
		{
			if (debounce() == 0)	//if tap button not pressed use first 3 waveforms
			{
				if (wavevalue <= 50)
				{
					wavetype = 0;
				}
			
				if (wavevalue > 50 && wavevalue < 230 )
				{
					wavetype = 1;
				}
			
				if (wavevalue >= 230)
				{
					wavetype = 2;
				}
			}
		
			if (debounce() == 1)		//if tap button pressed while moving waveform toggle use alternate waveforms
			{
				if (wavevalue <= 50)
				{
					wavetype = 3;
				}
			
				if (wavevalue > 50 && wavevalue < 230)
				{
					wavetype = 4;
				}
			
				if (wavevalue >=230)
				{
					wavetype = 5;
				}
				laststate = 1;		//press doesn't count as tap for tap tempo
				nbtap = 0;
				tapping = 0;
				msturns = 0;
			}
		
			previouswave = wavevalue;	//update previouswave for next toggle move
		}
	
		if (wavetype == 0)	//update mod waveform value
		{	//Sine
			if (inc <= 116)
			{
				currentinc = pgm_read_word_near(sine +inc);
			}
			else
			{
				currentinc = pgm_read_byte_near(smallsine + inc - 117);
			}
		}
	
		if (wavetype == 1)	//Square
		{
			if (inc <=127)
			{
				currentinc = 0;
			}
			else
			{
				currentinc = 400;
			}
		}
	
		if (wavetype == 2)	//Triangle
		{
			if (inc <= 80)
			{
				currentinc = pgm_read_byte_near(smalltriangle + inc);
			}
			if (inc > 80 && inc <= 173)
			{
				currentinc = triangle[inc - 81];
			}
			if (inc > 173 && inc < 255)
			{
				currentinc = pgm_read_byte_near(smalltriangle + 174 + 80 - inc);
			}
			if (inc == 255)
			{
				currentinc = 0;
			}
			
		}
		if (wavetype == 3)	//Sawtooth
		{
			if (inc <= 162)
			{
				currentinc = pgm_read_byte_near(smallsaw + inc);
			}
			
			else
			{
				currentinc = pgm_read_word_near(saw + inc - 163);			//Sawtooth
			}
		}
		if (wavetype == 4)	//Reverse Sawtooth
		{
			if (inc <= 92)
			{
				currentinc = pgm_read_word_near(saw + 92 -inc);
			}
			else
			{
				currentinc = pgm_read_byte_near(smallsaw + 162 + 93 - inc);
			}
		}
		if (wavetype == 5 && (inc%50) == 0)
		{
			currentinc = rand() / (RAND_MAX / 401);				//random
		}
		
		
		
		//---------DOUBLE TIME
		
		if (doubletime() != previousdoubletime)
		{
			if (doubletime()==1){divmult /= 2;}
			else{divmult *= 2;}
			divtempo = round(mstempo * divmult);
			if (divtempo > delaymax)
			{
				divtempo = delaymax;
			}
			data = findClosest(divmult);
			SPI_Transmit(data);
			previousdoubletime = doubletime();
		}
		
		
		
		
		//----------DELAY TIME POT
		
		if(block < 200)	//blocks the tap button input and the delay time input for 250 loops. This helps stabilizing the save tempo and preset recall.
		{
			previoustimevalue = timevalue;
			block++;
		}
				
		if (abs(previoustimevalue-timevalue) >= 13)	//if pot move of more than 5%, changing to pot control
		{
			if (cleanmode == 1)		//if clean mode active time pot course divided per 2
			{
				cleantimevalue = (timevalue*127)/255;
				SPI_Transmit(cleantimevalue);
			}
			else{SPI_Transmit(timevalue);}
			
			previoustimevalue = timevalue;
			mstempo = tempo[timevalue];
			tap = 0;
			TCNT1 = 0;
		}
		
		if (tap == 0 && TCNT1 >=800)
		{
			eeprom_update_byte((uint8_t*)1, 0);
		}
		
		
		
		//-------------TIME DIVISION
		
		if (abs(previousdiv - divtogglevalue) > 50)		//if first time or if div toggle changed position : update div
		{
			if (debounce() == 0)	//if tap button not pressed while changing 3 first div
			{
				if (divtogglevalue <= 50)
				{
					divmult = 1;	//fourth
				}
			
				if (divtogglevalue > 50 && divtogglevalue < 230)
				{
					divmult = 0.75;	//dotted eighth
				}
				
				if (divtogglevalue >= 230)
				{
					divmult = 0.5;	//eighth
				}
			}
			
			if (debounce() == 1)	//if tap button pressed while changing 3 last div
			{
				if (divtogglevalue <= 50)
				{
					divmult = 0.333333;	//triplet
				}
				
				if (divtogglevalue > 50 && divtogglevalue < 230)
				{
					divmult = 0.25;	//sixteenth
				}
				
				if (divtogglevalue >= 230)
				{
					divmult = 0.1666666;	//sextuplet
				}
				
				nbtap = 0;			//don't count press as tap
				tapping = 0;
				msturns = 0;
				laststate = 1;
			}
			
			if (doubletime()==1){divmult /= 2;}
			
			if (tap == 1)	//if in tap control, update digital pot value
			{
				divtempo = round(mstempo * divmult);
				if (divtempo >= delaymax)
				{
					divtempo = delaymax;
				}
				data = findClosest(divtempo);
				SPI_Transmit(data);
			}
			
			ledturns = 0;
			previousdiv = divtogglevalue;	//reseting previousdiv to detect next move
		}
		
		
		
		
		
		//---------------TAP TEMPO
		
		if (debounce()==1 && laststate==0 && nbtap==0 && block >= 200) //first tap
		{
			TCNT1 = 0;			//starts counting
			msturns=0;
			nbtap++;
			laststate = 1;
			tapping = 1;
		}
		
		if (msturns > ((delaymax/divmult)+800) && debounce()==0 && laststate==0) //if too long between taps  : resets
		{
			msturns = 0;
			nbtap = 0;
			tapping = 0;
			eeprom_update_word((uint16_t*)2, mstempo);
			eeprom_update_byte((uint8_t*)1, 1);
		}
		
		if (debounce()==0 && laststate ==1)	//release tap button
		{
			laststate = 0;
			LEDPORT &= ~(1<<LEDPIN);
		}
		
		if (debounce()==1 && laststate==0 && nbtap!=0) //not first tap
		{
			if (TCNT1 >= 500)
			{
				msturns++;
			}
			
			if (nbtap == 1)		//if second tap, tempo = time elapsed between the 2 button press
			{
				divtempo = msturns * divmult;
				mstempo = msturns;
			}
			
			else		//if not second tap, average every tap
			{
				divtempo = round((divtempo + (msturns*divmult)) / 2);
				mstempo = round((mstempo + msturns)/2);
			}
			
			if (divtempo > delaymax)
			{
				divtempo = delaymax;
			}

			data = findClosest(divtempo);			//wiper position returned by position in array
			SPI_Transmit(data);		//sending wiper position to digital pot

			nbtap++;			//updating number of tap and last state of tap button
			laststate = 1;
			TCNT1 = 0;			//reseting counter and ms
			msturns = 0;
			ledturns = 0;
			tap = 1;			//now in tap control mode
			LEDPORT |= (1<<LEDPIN);
		}
		
		
		
		//-----------PRESETS RECALL & SAVE
		
		if (debounce()==1 && msturns >= 3000 && laststate==1)	//if button pressed more than 3s
		{
			LEDPORT &= ~(1<<LEDPIN);
			blink1();
			for (uint16_t y=0; y<=1300;y++)	//wait for button release
			{
				_delay_ms(1);
				if (debounce()==0)
				{
					if (eeprom_read_byte((uint8_t *)32)==1)	//if button released recall preset one (if it has already been saved)
					{
						tap = eeprom_read_byte((uint8_t *)33);
						wavetype = eeprom_read_byte((uint8_t *)34);
						divmult = eeprom_read_float((float *)35);
						mstempo = eeprom_read_word((uint16_t *)39);
						presetdepth = eeprom_read_dword((uint32_t *)41);
						presetspeed = eeprom_read_byte((uint8_t *)45);
						if (doubletime()==1){divmult /= 2;}
						divtempo = (uint16_t) round(mstempo * divmult);
						if (divtempo > delaymax){divtempo = delaymax;}
						data = findClosest(divtempo);
						SPI_Transmit(data);
						speedpresetactive = 1;
						depthpresetactive = 1;
						previousdepth = depthvalue;
						previousspeed = speedvalue;
						block=0;	//Blocking delay time pot and tap button to make it stable
					}
					fastblink1();
					_delay_ms(300);
					break;
				}
			}
			if (debounce()==1)	//if button still not released
			{
				blink1();
				blink1();
				for (uint16_t y=0; y<=1300;y++)	//wait for button release
				{
					_delay_ms(1);
					if (debounce()==0)	//if button released recall preset 2
					{
						if (eeprom_read_byte((uint8_t *)64)==1)	//(recall only if preset 2 has previously been saved)
						{
							tap = eeprom_read_byte((uint8_t *)65);
							wavetype = eeprom_read_byte((uint8_t *)66);
							divmult = eeprom_read_float((float *)67);
							mstempo = eeprom_read_word((uint16_t *)71);
							presetdepth = eeprom_read_dword((uint32_t *)73);
							presetspeed = eeprom_read_byte((uint8_t *)77);
							divtempo = (uint16_t) round(mstempo * divmult);
							if (doubletime()==1){divmult /= 2;}
							if (divtempo > delaymax){divtempo = delaymax;}
							data = findClosest(divtempo);
							SPI_Transmit(data);
							speedpresetactive = 1;
							depthpresetactive = 1;
							previousdepth = depthvalue;
							previousspeed = speedvalue;
							block=0;
						}
						fastblink1();
						fastblink1();
						_delay_ms(300);
						break;
						}
					}
					if (debounce()==1)	//if button still pressed
					{
						LEDPORT |= (1<<LEDPIN);	//reverse blink (writing mode)
						_delay_ms(500);
						blink1();
						for (uint16_t y=0; y<=1300;y++)	//wait for button release
						{
							_delay_ms(1);
							if (debounce()==0)	//if button released save preset 1
							{
								eeprom_update_byte((uint8_t *)32, 1);
								eeprom_update_byte((uint8_t *)33, tap);
								eeprom_update_byte((uint8_t *)34, wavetype);
								eeprom_update_float((float *)35, divmult);
								eeprom_update_word((uint16_t *)39, mstempo);
								eeprom_update_dword((uint32_t *)41, depthvalue);
								eeprom_update_byte((uint8_t *)45, speedvalue);
								fastblink1();
								_delay_ms(300);
								break;
							}
						}
						if (debounce()==1)	//if button still not released
						{
							blink1();
							blink1();
							for (uint16_t y=0; y<=1300;y++)	//wait for button release
							{
								_delay_ms(1);
								if (debounce()==0)	//if button released save preset 2
								{
									eeprom_update_byte((uint8_t *)64, 1);
									eeprom_update_byte((uint8_t *)65, tap);
									eeprom_update_byte((uint8_t *)66, wavetype);
									eeprom_update_float((float *)67, divmult);
									eeprom_update_word((uint16_t *)71, mstempo);
									eeprom_update_dword((uint32_t *)73, depthvalue);
									eeprom_update_byte((uint8_t *)77, speedvalue);
									fastblink1();
									fastblink1();
									_delay_ms(300);
									break;
								}
							}
						}
					}
				}
		msturns = 0;	//reset tap sequence since this press is not to set tempo
		nbtap = 0;
		tapping = 0;
		}
		
		
		
		//---------LED CONTROL
		
		if (tap != 1 && tapping == 0)				//if button controlled  : LED on 
		{
				LEDPORT |= (1<<LEDPIN);
		}
		
		if (tapping == 1 && nbtap == 1 && laststate == 1)	//keep the light off when long button press
		{
			LEDPORT &= ~(1<<LEDPIN);
		}
		
		if (tap == 1 && tapping == 0 && debounce()==0)
		{	
			if (ledturns >= mstempo - 4)	//turns LED on every downbeat
			{
				ledturns = 0;
				LEDPORT |= (1<<LEDPIN);
			}
		
			if (ledturns == 8)			//turns LED off 4ms after downbeat
			{
				LEDPORT &= ~(1<<LEDPIN);
			}
		}
    }
}
