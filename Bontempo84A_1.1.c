/*
 * Bontempo84A.c
 *
 * Created: 15/07/2019 21:30:51
 * Author : Antoine Ricoux
 
 This is a tap tempo with modulation for PT2399-based delay
 This code is created for the ATtiny84A and digital potentiometer MCP41100 (or MCP42100)
 
 Pin configuration :
 
 1: VCC
 2: LED +
 3: Tap Momentary Button
 4: Div Tempo LED + (if RESET is disabled)
 5: Waveform Toggle (On-Off-On)
 6: SPI Chip Select
 7: Mod PWM Out
 8: SPI Data Out
 9: SPI Clock Out
 10: Mod Depth Potentiometer
 11: Mod Speed Potentiometer 
 12: Time Div Toggle (On-Off-On)
 13: Time Potentiometer
 14: GND
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
#define LED2PIN PINB3
#define LED2DDR DDRB
#define LED2PORT PORTB
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
#define PWMDDR DDRA
#define N_ARRAY 255
#define DEBOUNCE_TIME 500	//Tap button debounce time in microseconds
#define DELAY_MIN 36
#define RAND_MAX 0x7fff

volatile uint8_t timevalue;
volatile uint8_t divtogglevalue;
volatile uint16_t msturns = 0;
volatile uint16_t ledturns = 0;
volatile uint16_t led2turns = 0;
volatile uint8_t inc = 0;
volatile uint8_t speedvalue;
volatile unsigned long depthvalue;
volatile uint8_t wavevalue;
volatile unsigned long currentinc = 500;
volatile uint16_t pwm;
volatile uint16_t offset;
volatile uint16_t speed;

PROGMEM prog_uint16_t sine[] = {					//Sine wavetable
			0x1f4,0x200,0x20c,0x218,0x224,0x231,0x23d,0x249,0x255,0x261,0x26d,0x279,0x284,0x290,0x29c,0x2a7,
			0x2b3,0x2be,0x2c9,0x2d4,0x2df,0x2ea,0x2f4,0x2ff,0x309,0x313,0x31d,0x327,0x330,0x33a,0x343,0x34c,
			0x355,0x35d,0x366,0x36e,0x376,0x37d,0x385,0x38c,0x393,0x39a,0x3a0,0x3a6,0x3ac,0x3b2,0x3b7,0x3bc,
			0x3c1,0x3c6,0x3ca,0x3ce,0x3d1,0x3d5,0x3d8,0x3db,0x3dd,0x3e0,0x3e2,0x3e3,0x3e5,0x3e6,0x3e6,0x3e7,
			0x3e7,0x3e7,0x3e6,0x3e6,0x3e5,0x3e3,0x3e2,0x3e0,0x3dd,0x3db,0x3d8,0x3d5,0x3d1,0x3ce,0x3ca,0x3c6,
			0x3c1,0x3bc,0x3b7,0x3b2,0x3ac,0x3a6,0x3a0,0x39a,0x393,0x38c,0x385,0x37d,0x376,0x36e,0x366,0x35d,
			0x355,0x34c,0x343,0x33a,0x330,0x327,0x31d,0x313,0x309,0x2ff,0x2f4,0x2ea,0x2df,0x2d4,0x2c9,0x2be,
			0x2b3,0x2a7,0x29c,0x290,0x284,0x279,0x26d,0x261,0x255,0x249,0x23d,0x231,0x224,0x218,0x20c,0x200,
			0x1f4,0x1e7,0x1db,0x1cf,0x1c3,0x1b6,0x1aa,0x19e,0x192,0x186,0x17a,0x16e,0x163,0x157,0x14b,0x140,
			0x134,0x129,0x11e,0x113,0x108,0xfd,0xf3,0xe8,0xde,0xd4,0xca,0xc0,0xb7,0xad,0xa4,0x9b,
			0x92,0x8a,0x81,0x79,0x71,0x6a,0x62,0x5b,0x54,0x4d,0x47,0x41,0x3b,0x35,0x30,0x2b,
			0x26,0x21,0x1d,0x19,0x16,0x12,0xf,0xc,0xa,0x7,0x5,0x4,0x2,0x1,0x1,0x0,
			0x0,0x0,0x1,0x1,0x2,0x4,0x5,0x7,0xa,0xc,0xf,0x12,0x16,0x19,0x1d,0x21,
			0x26,0x2b,0x30,0x35,0x3b,0x41,0x47,0x4d,0x54,0x5b,0x62,0x6a,0x71,0x79,0x81,0x8a,
			0x92,0x9b,0xa4,0xad,0xb7,0xc0,0xca,0xd4,0xde,0xe8,0xf3,0xfd,0x108,0x113,0x11e,0x129,
			0x134,0x140,0x14b,0x157,0x163,0x16e,0x17a,0x186,0x192,0x19e,0x1aa,0x1b6,0x1c3,0x1cf,0x1db,0x1e7};
			
PROGMEM prog_uint16_t squar[] = {					//square wavetable
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
			0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,0x3e7,
};

PROGMEM prog_uint16_t triangle[] = {			//triangle wavetable
			0x8,0x10,0x17,0x1f,0x27,0x2f,0x37,0x3e,0x46,0x4e,0x56,0x5e,0x65,0x6d,0x75,0x7d,
			0x85,0x8c,0x94,0x9c,0xa4,0xac,0xb4,0xbb,0xc3,0xcb,0xd3,0xdb,0xe2,0xea,0xf2,0xfa,
			0x102,0x109,0x111,0x119,0x121,0x129,0x130,0x138,0x140,0x148,0x150,0x157,0x15f,0x167,0x16f,0x177,
			0x17e,0x186,0x18e,0x196,0x19e,0x1a5,0x1ad,0x1b5,0x1bd,0x1c5,0x1cc,0x1d4,0x1dc,0x1e4,0x1ec,0x1f4,
			0x1fb,0x203,0x20b,0x213,0x21b,0x222,0x22a,0x232,0x23a,0x242,0x249,0x251,0x259,0x261,0x269,0x270,
			0x278,0x280,0x288,0x290,0x297,0x29f,0x2a7,0x2af,0x2b7,0x2be,0x2c6,0x2ce,0x2d6,0x2de,0x2e5,0x2ed,
			0x2f5,0x2fd,0x305,0x30c,0x314,0x31c,0x324,0x32c,0x333,0x33b,0x343,0x34b,0x353,0x35b,0x362,0x36a,
			0x372,0x37a,0x382,0x389,0x391,0x399,0x3a1,0x3a9,0x3b0,0x3b8,0x3c0,0x3c8,0x3d0,0x3d7,0x3df,0x3e7,
			0x3df,0x3d7,0x3d0,0x3c8,0x3c0,0x3b8,0x3b0,0x3a9,0x3a1,0x399,0x391,0x389,0x382,0x37a,0x372,0x36a,
			0x362,0x35b,0x353,0x34b,0x343,0x33b,0x333,0x32c,0x324,0x31c,0x314,0x30c,0x305,0x2fd,0x2f5,0x2ed,
			0x2e5,0x2de,0x2d6,0x2ce,0x2c6,0x2be,0x2b7,0x2af,0x2a7,0x29f,0x297,0x290,0x288,0x280,0x278,0x270,
			0x269,0x261,0x259,0x251,0x249,0x242,0x23a,0x232,0x22a,0x222,0x21b,0x213,0x20b,0x203,0x1fb,0x1f4,
			0x1ec,0x1e4,0x1dc,0x1d4,0x1cc,0x1c5,0x1bd,0x1b5,0x1ad,0x1a5,0x19e,0x196,0x18e,0x186,0x17e,0x177,
			0x16f,0x167,0x15f,0x157,0x150,0x148,0x140,0x138,0x130,0x129,0x121,0x119,0x111,0x109,0x102,0xfa,
			0xf2,0xea,0xe2,0xdb,0xd3,0xcb,0xc3,0xbb,0xb4,0xac,0xa4,0x9c,0x94,0x8c,0x85,0x7d,
			0x75,0x6d,0x65,0x5e,0x56,0x4e,0x46,0x3e,0x37,0x2f,0x27,0x1f,0x17,0x10,0x8,0x0
};

PROGMEM prog_int16_t saw[] = {				//sawtooth wavetable
	0,4,8,12,16,20,24,27,31,35,39,43,47,51,55,59,63,67,71,74,78,82,86,90,94,98,102,106,110,114,118,121,125,129,
	133,137,141,145,149,153,157,161,165,168,172,176,180,184,188,192,196,200,204,208,212,215,219,223,227,231,235,
	239,243,247,251,255,259,262,266,270,274,278,282,286,290,294,298,302,306,309,313,317,321,325,329,333,337,341,
	345,349,353,357,360,364,368,372,376,380,384,388,392,396,400,404,407,411,415,419,423,427,431,435,439,443,447,
	451,454,458,462,466,470,474,478,482,456,490,494,498,501,505,509,513,517,521,525,529,533,537,541,545,548,552,
	556,560,564,568,572,576,580,584,588,592,595,599,603,607,611,615,619,623,627,631,635,639,642,646,650,654,658,
	662,666,670,674,678,682,686,690,693,697,701,705,709,713,717,721,725,729,733,737,740,744,748,752,756,760,764,
	768,772,776,780,784,787,791,795,799,803,807,811,815,819,823,827,831,834,838,842,846,850,854,858,862,866,870,
	874,878,881,885,889,893,897,901,905,909,913,917,921,925,928,932,936,940,944,948,952,956,960,964,968,972,975,
	979,983,987,991,995,999
};

PROGMEM prog_uint16_t tempo[] ={			//wiper position to ms tempo conversion chart for PT2399
	36,37,40,44,50,55,60,65,70,74,79,83,88,93,97,102,107,111,116,121,
	125,131,133,140,145,149,154,159,166,166,168,176,181,185,190,196,200,205,208,214,
	222,222,228,231,237,239,246,250,256,258,266,271,277,277,277,282,289,295,296,303,
	308,313,319,331,333,333,333,333,341,351,357,359,363,370,370,375,384,389,389,389,
	391,400,406,416,416,416,417,424,429,437,444,444,452,459,464,466,466,466,473,477,
	486,496,500,500,500,501,513,518,518,518,521,532,535,542,549,555,555,555,556,569,
	578,583,583,583,584,589,598,600,606,608,615,623,627,639,664,666,666,666,666,666,
	666,666,666,666,667,667,667,670,687,697,702,706,713,717,723,729,733,739,740,740,
	747,753,758,767,774,777,777,777,777,777,778,782,786,796,805,810,811,818,829,833,
	833,833,840,854,857,857,857,857,858,864,871,879,888,888,889,894,905,909,913,921,
	930,933,933,937,947,952,952,954,966,986,998,999,999,1000,1000,1000,1000,1000,1000,1000,
	1000,1000,1000,1019,1026,1032,1040,1044,1048,1051,1053,1061,1065,1070,1075,1083,1096,1105,1108,1110,
	1111,1111,1111,1111,1111,1111,1113,1115,1123,1145,1153,1155,1158,1160,1162
	};

uint8_t debounce(void)					//tels with certainty if button is pressed
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

void blink1(void)		//toggles led to verify interactions
{
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
	LEDPORT ^= (1<<LEDPIN);
	_delay_ms(150);
}

ISR(TIM1_OVF_vect)
{
	msturns++;	//ms increment if timer1 overflow
	ledturns++;
	led2turns++;
}

ISR(TIM1_COMPA_vect)
{
	cli();		//updates mod pwm cycle duty
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
	SPIDDR |= (1<<CLKPIN) | (1<<DATAPIN); //SPI pins as output
	CSDDR |= (1<<CSPIN);
	CSPORT |= (1<<CSPIN);	//Chip select pin high (not selected)
	
	SPI_Transmit(200);	//wait for PT2399 startup
	_delay_ms(550);
	
	LEDDDR |= (1<<LEDPIN);	//LED pin as output
	LED2DDR |= (1<<LED2PIN);
	BUTTONPORT |= (1<<BUTTONPIN);	//Tap Button pin set to high 
	
	PWMDDR |= (1<<PWMPIN);	//PWM pin as output
	
	TCNT0 = 0;				//timer0, fast PWM, OCRA as TOP, enable compare A interrupt, 64 prescaler
	OCR0A = 100;
	TCCR0A |= (1<<WGM00) | (1<<WGM01);
	TIMSK0 |= (1<<OCIE0A);
	TCCR0B |= (1<<WGM02) | (1<<CS01) | (1<<CS00);
	
	ICR1 = 999;	//timer1, fast PWM, ICR1 as TOP, enable overflow interrupt and compare A interrupt, no prescaler
	TCNT1 = 0;
	OCR1A = 500;
	TCCR1A |= (1<<WGM11) | (1<<COM1A1);	
	TIMSK1 |= (1<<TOIE1) | (1<<OCIE1A);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2); //Enabling ADC, 16 prescaler, ADC interrupts enable
	ADCSRB |= (1<<ADLAR); //8bit conversion
	
	uint16_t mstempo;		//The current tempo tapped (it will  be multiplied by the tempo div)
	uint16_t ledtempo;		//tempo for toggling LED (not influenced by tempo div)
	uint16_t delaymax = 1162;	//maximum tempo if not in clean mode
	
	uint16_t nbtap = 0;		//number of times tapped during the current sequence
	uint8_t laststate = 0;	//last state of the button
	uint8_t tap = 0;		//tap controlled (1) or pot controlled (0)
	
	uint8_t data;	//to store digital pot wiper position
	
	uint8_t previoustimevalue;	//used to know if the time pot moved
	uint16_t previousdiv = 300;	//used to know if the div toggle moved
	float divmult = 1;				//the div tempo multiplicand
	
	uint16_t previouswave = 300;	//used to know if waveform toggle moved
	uint8_t wavetype = 0;			//
	
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
	
	ADCSRA |= (1<<ADSC); //Starting first AD conversion
	
    while (1) 
    {
		
		offset = 500 - ((depthvalue * 500)/255);	//updating mod pwm value
		pwm = ((currentinc * depthvalue)/255) + offset;
		speed = speedvalue; //updating mod speed
		if (speed < 3) //limiting speed, it bugs if lower than 3
		{
			speed = 3;
		}
		
		if (abs(previouswave-wavevalue) > 50)	//if wave toggle moves update waveform
		{
			if (debounce() == 0)	//if tap button not pressed use first 3 waveforms
			{
				if (wavevalue <= 20)
				{
					wavetype = 0;
				}
				
				if (wavevalue > 20 && wavevalue < 230 )
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
				if (wavevalue <= 20)
				{
					wavetype = 3;
				}
				
				if (wavevalue > 20 && wavevalue < 230)
				{
					wavetype = 4;
				}
				
				if (wavevalue >=230)
				{
					wavetype = 5;
				}
				laststate = 1;		//press doesn't count as tap for tap tempo
				nbtap = 0;
			}
			
			previouswave = wavevalue;	//update previouswave for next toggle move
		}
		
		if (wavetype == 0)	//update mod wavetable value
		{
			currentinc = pgm_read_word_near(sine + inc);	//sine
		}
		
		if (wavetype == 1)
		{
			currentinc = pgm_read_word_near(squar + inc);	//square
		}
		
		if (wavetype == 2)
		{
			currentinc = pgm_read_word_near(triangle + inc);//triangle
		}
		
		if (wavetype == 3)
		{
			currentinc = pgm_read_word_near(saw + inc);		//sawtooth
		}
		
		if (wavetype == 4)
		{
			currentinc = pgm_read_word_near(saw + 255 - inc);//reverse sawtooth
		}
		
		if (wavetype == 5 && (inc%50) == 0)
		{
			currentinc = pgm_read_word_near(saw + (rand() / (RAND_MAX / 256)));	//random
		}
		
		if (tap == 0 || abs(previoustimevalue-timevalue) >= 13)	//if pot move of more than 5%, changing to pot control
		{
			if (cleanmode == 1)		//if clean mode active time pot course divided per 2
			{
				timevalue = (timevalue*127)/255;
			}
			
			SPI_Transmit(timevalue);
			mstempo = tempo[timevalue];
			previoustimevalue = timevalue;
			tap = 0;
		}
		
		if (abs(previousdiv - divtogglevalue) > 50)		//if first time or if div toggle changed position : update div
		{
			if (debounce() == 0)	//if tap button not pressed while changing 3 first div
			{
				if (divtogglevalue <= 20)
				{
					divmult = 1;	//fourth
				}
			
				if (divtogglevalue > 20 && divtogglevalue < 230)
				{
					divmult = 0.5;	//eighth
				}
				
				if (divtogglevalue >= 230)
				{
					divmult = 0.75;	//dotted eighth
				}
			}
			
			if (debounce() == 1)	//if tap button pressed while changing 3 last div
			{
				if (divtogglevalue <= 20)
				{
					divmult = 0.333333;	//triplet
				}
				
				if (divtogglevalue > 20 && divtogglevalue < 230)
				{
					divmult = 0.1666666;	//sextuplet
				}
				
				if (divtogglevalue >= 230)
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
			
			previousdiv = divtogglevalue;	//reseting previousdiv to detect next move
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
				ledtempo = msturns;
			}
			
			if (nbtap!=1)		//if not second tap, average every tap
			{
				mstempo = round((mstempo + (msturns*divmult)) / 2);
				ledtempo = round((ledtempo + msturns)/2);
			}
			
			if (mstempo < DELAY_MIN)		//clipping current tempo to min and max tempo
			{
				mstempo = DELAY_MIN;
			}
			
			if (mstempo > delaymax)
			{
				mstempo = delaymax;
			}
			
			if (ledtempo < DELAY_MIN)	//clipping LED toggling to min and max tempo
			{
				ledtempo = DELAY_MIN;
			}
			
			if (ledtempo > delaymax)
			{
				ledtempo = delaymax;
			}
			
			data = findClosest(mstempo);			//wiper position returned by position in array
			
			SPI_Transmit(data);		//sending wiper position to digital pot
			
			nbtap++;			//updating number of tap and last state of tap button
			laststate = 1;
			TCNT1 = 0;			//reseting counter and ms
			msturns = 0;
			tap = 1;			//now in tap control mode
		}
		
		if (tap == 0)				//if button controlled  : LED on 
		{
			LEDPORT |= (1<<LEDPIN);
		}
		
		if (ledturns >= ledtempo && tap==1)	//turns LED on for 8ms every downbeat
		{
			ledturns = 0;
			LEDPORT |= (1<<LEDPIN);
			_delay_ms(8);
			LEDPORT &= ~(1<<LEDPIN);
		}
		
		if (led2turns >= mstempo && tap == 1) //Turns LED2 on for 8ms every time division (doesn't stop if button controlled)
		{
			led2turns = 0;
			LED2PORT |= (1<<LED2PIN);
			_delay_ms(8);
			LED2PORT &= ~(1<<LED2PIN);
		}
		
    }
}
