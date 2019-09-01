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
 Low Fuse = 0xd2	High Fuse = 0xdc (Reset enabled, led 2 disabled)
 Low Fuse = 0xd2	High Fuse = 0x5c !!!CAREFUL!!! RESET disabled, led2 enabled
 
 Thanks to Florian Dupeyron who helped me debug some stuff
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
volatile uint16_t led2turns = 0;
volatile uint8_t inc = 0;
volatile uint8_t speedvalue;
volatile unsigned long depthvalue;
volatile uint8_t wavevalue;
volatile unsigned long currentinc = 500;
volatile uint16_t pwm;
volatile int16_t offset;
volatile uint16_t speed;
volatile uint8_t modenable = 1;

PROGMEM const uint16_t sine[] = {					//Sine wavetable
			0xc8,0xcd,0xd2,0xd7,0xdc,0xe0,0xe5,0xea,0xef,0xf4,0xf9,0xfd,0x102,0x107,0x10b,0x110,
			0x115,0x119,0x11e,0x122,0x126,0x12b,0x12f,0x133,0x137,0x13b,0x13f,0x143,0x147,0x14b,0x14e,0x152,
			0x155,0x159,0x15c,0x15f,0x163,0x166,0x169,0x16c,0x16e,0x171,0x174,0x176,0x178,0x17b,0x17d,0x17f,
			0x181,0x183,0x184,0x186,0x187,0x189,0x18a,0x18b,0x18c,0x18d,0x18e,0x18e,0x18f,0x18f,0x190,0x190,
			0x190,0x190,0x190,0x18f,0x18f,0x18e,0x18e,0x18d,0x18c,0x18b,0x18a,0x189,0x187,0x186,0x184,0x183,
			0x181,0x17f,0x17d,0x17b,0x178,0x176,0x174,0x171,0x16e,0x16c,0x169,0x166,0x163,0x15f,0x15c,0x159,
			0x155,0x152,0x14e,0x14b,0x147,0x143,0x13f,0x13b,0x137,0x133,0x12f,0x12b,0x126,0x122,0x11e,0x119,
			0x115,0x110,0x10b,0x107,0x102,0xfd,0xf9,0xf4,0xef,0xea,0xe5,0xe0,0xdc,0xd7,0xd2,0xcd,
			0xc8,0xc3,0xbe,0xb9,0xb4,0xb0,0xab,0xa6,0xa1,0x9c,0x97,0x93,0x8e,0x89,0x85,0x80,
			0x7b,0x77,0x72,0x6e,0x6a,0x65,0x61,0x5d,0x59,0x55,0x51,0x4d,0x49,0x45,0x42,0x3e,
			0x3b,0x37,0x34,0x31,0x2d,0x2a,0x27,0x24,0x22,0x1f,0x1c,0x1a,0x18,0x15,0x13,0x11,
			0xf,0xd,0xc,0xa,0x9,0x7,0x6,0x5,0x4,0x3,0x2,0x2,0x1,0x1,0x0,0x0,
			0x0,0x0,0x0,0x1,0x1,0x2,0x2,0x3,0x4,0x5,0x6,0x7,0x9,0xa,0xc,0xd,
			0xf,0x11,0x13,0x15,0x18,0x1a,0x1c,0x1f,0x22,0x24,0x27,0x2a,0x2d,0x31,0x34,0x37,
			0x3b,0x3e,0x42,0x45,0x49,0x4d,0x51,0x55,0x59,0x5d,0x61,0x65,0x6a,0x6e,0x72,0x77,
			0x7b,0x80,0x85,0x89,0x8e,0x93,0x97,0x9c,0xa1,0xa6,0xab,0xb0,0xb4,0xb9,0xbe,0xc3
};

PROGMEM const uint16_t triangle[] = {			//triangle wavetable
			0x3,0x6,0x9,0xd,0x10,0x13,0x16,0x19,0x1c,0x1f,0x22,0x26,0x29,0x2c,0x2f,0x32,
			0x35,0x38,0x3b,0x3f,0x42,0x45,0x48,0x4b,0x4e,0x51,0x54,0x58,0x5b,0x5e,0x61,0x64,
			0x67,0x6a,0x6d,0x71,0x74,0x77,0x7a,0x7d,0x80,0x83,0x86,0x8a,0x8d,0x90,0x93,0x96,
			0x99,0x9c,0x9f,0xa3,0xa6,0xa9,0xac,0xaf,0xb2,0xb5,0xb8,0xbc,0xbf,0xc2,0xc5,0xc8,
			0xcb,0xce,0xd1,0xd5,0xd8,0xdb,0xde,0xe1,0xe4,0xe7,0xea,0xee,0xf1,0xf4,0xf7,0xfa,
			0xfd,0x100,0x103,0x107,0x10a,0x10d,0x110,0x113,0x116,0x119,0x11c,0x120,0x123,0x126,0x129,0x12c,
			0x12f,0x132,0x135,0x139,0x13c,0x13f,0x142,0x145,0x148,0x14b,0x14e,0x152,0x155,0x158,0x15b,0x15e,
			0x161,0x164,0x167,0x16b,0x16e,0x171,0x174,0x177,0x17a,0x17d,0x180,0x184,0x187,0x18a,0x18d,0x190,
			0x18d,0x18a,0x187,0x184,0x180,0x17d,0x17a,0x177,0x174,0x171,0x16e,0x16b,0x167,0x164,0x161,0x15e,
			0x15b,0x158,0x155,0x152,0x14e,0x14b,0x148,0x145,0x142,0x13f,0x13c,0x139,0x135,0x132,0x12f,0x12c,
			0x129,0x126,0x123,0x120,0x11c,0x119,0x116,0x113,0x110,0x10d,0x10a,0x107,0x103,0x100,0xfd,0xfa,
			0xf7,0xf4,0xf1,0xee,0xea,0xe7,0xe4,0xe1,0xde,0xdb,0xd8,0xd5,0xd1,0xce,0xcb,0xc8,
			0xc5,0xc2,0xbf,0xbc,0xb8,0xb5,0xb2,0xaf,0xac,0xa9,0xa6,0xa3,0x9f,0x9c,0x99,0x96,
			0x93,0x90,0x8d,0x8a,0x86,0x83,0x80,0x7d,0x7a,0x77,0x74,0x71,0x6d,0x6a,0x67,0x64,
			0x61,0x5e,0x5b,0x58,0x54,0x51,0x4e,0x4b,0x48,0x45,0x42,0x3f,0x3b,0x38,0x35,0x32,
			0x2f,0x2c,0x29,0x26,0x22,0x1f,0x1c,0x19,0x16,0x13,0x10,0xd,0x9,0x6,0x3,0x0
};

PROGMEM const uint16_t saw[] = {				//sawtooth wavetable
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
			251,253,254,256,257,259,260,262,264,265,267,268,270,271,273,275,
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

PROGMEM const uint16_t nomodtempo[] = {		//Tempo if the Modulation is disabled
			41,42,44,47,52,58,63,68,73,77,82,86,90,95,99,104,108,112,117,121,
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
			1076,1080,1084,1088,1093,1097,1101,1105,1109,1113,1118,1122,1126,1130,1134
};

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
  
  // Returns element closest to target in arr[] and return wiper position
int nomodfindClosest(int target) 
{ 
    // Corner cases 
    if (target <= pgm_read_word_near(nomodtempo + 0)) 
        return 0; 
    if (target >= pgm_read_word_near(nomodtempo + (N_ARRAY-1))) 
        return N_ARRAY - 1; 
  
    // Doing binary search 
    int i = 0, j = N_ARRAY, mid = 0; 
    while (i < j) { 
        mid = (i + j) / 2; 
  
        if (pgm_read_word_near(nomodtempo + mid) == target) 
            return mid; 
  
        /* If target is less than array element, 
            then search in left */
        if (target < pgm_read_word_near(nomodtempo + mid)) { 
  
            // If target is greater than previous 
            // to mid, return closest of two 
            if (mid > 0 && target > pgm_read_word_near(nomodtempo + (mid - 1))) 
                return getClosest(pgm_read_word_near(nomodtempo + (mid - 1)), 
                                  pgm_read_word_near(nomodtempo + mid), target, mid-1, mid); 
  
            /* Repeat for left half */
            j = mid; 
        } 
  
        // If target is greater than mid 
        else { 
            if (mid < N_ARRAY - 1 && target < pgm_read_word_near(nomodtempo + (mid + 1))) 
                return getClosest(pgm_read_word_near(nomodtempo + mid), 
                                  pgm_read_word_near(nomodtempo + (mid + 1)), target, mid, mid+1); 
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

void modtest(void)	//testing if the mod pin are tied to ground at startup, if so : disable modulation
{
	MODPORT |= (1<<WAVEPIN) | (1<<SPEEDPIN) | (1<<DEPTHPIN) | (1<<PWMPIN);	//output Vcc to every mod pin
	
	if (bit_is_clear(MODSFR, WAVEBV) && bit_is_clear(MODSFR, SPEEDBV) && bit_is_clear(MODSFR, DEPTHBV) && bit_is_clear(MODSFR, PWMBV))
	{
		modenable = 0;
	}
	else
	{
		modenable = 1;
	}
	
	MODPORT &= ~(1<<WAVEPIN) & ~(1<<SPEEDPIN) & ~(1<<DEPTHPIN) & ~(1<<PWMPIN);
}

void blink1(void)		//toggles led to verify interactions
{
	LEDPORT ^= (1<<LEDPIN);
	LED2PORT^= (1<<LED2PIN);
	_delay_ms(150);
	LEDPORT ^= (1<<LEDPIN);
	LED2PORT ^= (1<<LED2PIN);
	_delay_ms(150);
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
	led2turns++;
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
	modtest();		//Checking if modulation is disabled
	
	LEDDDR |= (1<<LEDPIN);	//LED pins as output
	LED2DDR |= (1<<LED2PIN);
	BUTTONPORT |= (1<<BUTTONPIN);	//Tap Button pin set to high 
	
	if (modenable == 1) //If Modulation is enabled
	{
		PWMDDR |= (1<<PWMPIN);	//PWM pin as output
		
		TCNT0 = 0;				//timer0, fast PWM, OCRA as TOP, enable compare A interrupt, 64 prescaler
		OCR0A = 100;
		TCCR0A |= (1<<WGM00) | (1<<WGM01);
		TIMSK0 |= (1<<OCIE0A);
		TCCR0B |= (1<<WGM02) | (1<<CS02) | (1<<CS00);
	}
	
	ICR1 = 999;	//timer1, fast PWM, ICR1 as TOP, enable overflow interrupt and compare A interrupt, no prescaler
	TCNT1 = 0;
	if (modenable == 1)
	{
		OCR1A = 250;
		TCCR1A |= (1<<COM1A1);	
		TIMSK1 |= (1<<OCIE1A);
	}
	
	TCCR1A |= (1<<WGM11);	
	TIMSK1 |= (1<<TOIE1);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11);
	
	if (modenable == 1)
	{
		MODPORT |= (1<<WAVEPIN);	//Connecting Pull-Up Resistor of toggle pins
	}
	DIVPORT |= (1<<DIVPIN);
	
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enabling ADC, 16 prescaler, ADC interrupts enable
	ADCSRB |= (1<<ADLAR); //8bit conversion
	
	SPIDDR |= (1<<CLKPIN) | (1<<DATAPIN); //SPI pins as output
	CSDDR |= (1<<CSPIN);
	CSPORT |= (1<<CSPIN);	//Chip select pin high (not selected)
	
	uint16_t mstempo;		//The current tempo tapped (it will  be multiplied by the tempo div)
	uint16_t ledtempo;		//tempo for toggling LED (not influenced by tempo div)
	uint8_t delaymin = 51;
	uint16_t delaymax = 1291;	//maximum tempo if not in clean mode
	int8_t pwmfine[] = {25,21,16,11,6,0,-7,-11,-15,-19,-23};	//pwm values for +-5ms offset
	uint8_t delta;
	unsigned long cleantimevalue;
	if (modenable == 0)	//if mod disabled change max delay
	{
		delaymin = 41;
		delaymax = 1134;
	}
	
	uint16_t nbtap = 0;		//number of times tapped during the current sequence
	uint8_t laststate = 0;	//last state of the button
	uint8_t tap = 0;		//tap controlled (1) or pot controlled (0)
	
	uint8_t data;	//to store digital pot wiper position
	
	uint8_t previoustimevalue;	//used to know if the time pot moved
	uint16_t previousdiv = 300;	//used to know if the div toggle moved
	float divmult = 1;				//the div tempo multiplicand
	
	uint16_t previouswave = 300;	//used to know if waveform toggle moved
	uint8_t wavetype = 0;			//this value selects one of the 6 waveforms
	
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
		if (modenable == 1)
		{
			offset = (300 - ((depthvalue * 300)/255));	//updating mod pwm value
			
			if (tap == 1)
			{
				delta = 5 + mstempo - tempo[data];	//difference between tempo tapped and closest tempo in conversion chart
				offset = offset + pwmfine[delta];	//compensate with pwm if tap tempo is active
			}
			
			pwm = (((100 + currentinc) * depthvalue)/255) + offset;
			
			speed = 255 - speedvalue; //updating mod speed
			
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
				}
			
				previouswave = wavevalue;	//update previouswave for next toggle move
			}
		
			if (wavetype == 0)	//update mod waveform value
			{
				currentinc = pgm_read_word_near(sine + inc);	//sine
			}
		
			if (wavetype == 1)	//Square
			{
				if (inc <=127)
				{
					currentinc = 0;
				}
				if (inc > 127)
				{
					currentinc = 400;
				}
			}
		
			if (wavetype == 2)
			{
				currentinc = pgm_read_word_near(triangle + inc);
			}
		
			if (wavetype == 3)
			{
				currentinc = pgm_read_word_near(saw + inc);
			}
		
			if (wavetype == 4)
			{
				currentinc = pgm_read_word_near(saw + 255 - inc);
			}
		
			if (wavetype == 5 && (inc%50) == 0)
			{
				currentinc = rand() / (RAND_MAX / 401);	//random
			}
		}
		
		
		if (tap == 0 || abs(previoustimevalue-timevalue) >= 13)	//if pot move of more than 5%, changing to pot control
		{
			if (cleanmode == 1)		//if clean mode active time pot course divided per 2
			{
				cleantimevalue = (timevalue*127)/255;
				SPI_Transmit(cleantimevalue);
			}
			if (cleanmode != 1)
			{
				SPI_Transmit(timevalue);
			}
			
			previoustimevalue = timevalue;
			tap = 0;
		}
		
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
				laststate = 1;
			}
			
			if (dual2399 == 1)		//if dual2399 mode is active = divide any div per 2
			{
				divmult = divmult / 2;
			}
			
			if (tap == 1)	//if in tap control, update digital pot value
			{
				mstempo = round(ledtempo * divmult);
				
				if (mstempo < delaymin)		//clipping current tempo to min and max tempo
				{
					mstempo = delaymin;
				}
				
				if (mstempo > delaymax)
				{
					mstempo = delaymax;
				}
				
				if (modenable == 1)
				{
					data = findClosest(mstempo);
				}
				if (modenable == 0)
				{
					data = nomodfindClosest(mstempo);
				}
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
		
		if (msturns > ((delaymax/divmult)+800)) //if too long between taps  : resets
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
			if (TCNT1 >= 500)
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
			
			if (mstempo < delaymin)		//clipping current tempo to min and max tempo
			{
				mstempo = delaymin;
			}
			
			if (mstempo > delaymax)
			{
				mstempo = delaymax;
			}
			
			if (ledtempo < delaymin)	//clipping LED toggling to min and max tempo
			{
				ledtempo = delaymin;
			}
			
			if (ledtempo > delaymax)
			{
				ledtempo = delaymax;
			}
			
			if (modenable == 1)
			{
				data = findClosest(mstempo);			//wiper position returned by position in array
			}
			
			if (modenable == 0)
			{
				data = nomodfindClosest(mstempo);			//wiper position returned by position in array
			}
			
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
			LED2PORT |= (1<<LED2PIN);
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
