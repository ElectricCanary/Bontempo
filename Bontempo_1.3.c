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
 Low Fuse = 0xc2	High Fuse = 0xdd (Reset enabled, led 2 disabled)
 Low Fuse = 0xc2	High Fuse = 0x5d !!!CAREFUL!!! RESET disabled, led2 enabled
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
#define DEBOUNCE_TIME 500	//Tap button debounce time in microseconds
#define DELAY_MIN 37
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

PROGMEM const uint16_t tempo[] ={			//wiper position to ms tempo conversion chart for PT2399, calibrated with PWM at 300 (no modulation)
		37,38,42,48,54,60,66,71,76,82,87,92,97,103,108,113,
		119,125,130,135,141,147,152,157,163,167,172,177,183,188,192,198,
		204,209,213,220,223,228,234,239,244,250,256,260,267,272,278,282,
		285,292,295,301,306,312,318,324,334,334,334,338,347,352,358,361,
		369,373,379,386,390,390,392,400,406,414,417,418,425,431,436,443,
		447,454,461,464,467,468,473,481,489,499,501,501,502,510,517,520,
		522,530,536,541,548,554,557,559,566,573,580,584,584,585,591,599,
		603,608,613,617,622,628,635,645,660,668,668,668,668,668,668,668,
		669,670,682,695,702,708,713,718,723,729,735,740,743,747,751,752,
		756,762,771,777,780,780,783,791,799,803,807,813,818,824,831,835,
		835,836,840,849,857,861,866,873,882,888,891,891,891,893,897,905,
		916,919,923,929,934,936,939,946,952,958,963,970,976,983,991,997,
		1002,1002,1002,1002,1003,1003,1008,1016,1026,1034,1038,1040,1042,1046,1051,1056,
		1066,1073,1079,1085,1094,1103,1110,1113,1114,1114,1114,1114,1118,1124,1132,1139,
		1146,1149,1153,1159,1165,1168,1169,1169,1170,1174,1181,1189,1196,1201,1206,1210,
		1219,1225,1230,1235,1242,1248,1252,1256,1261,1265,1270,1275,1280,1286,1296,1306
	};

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

void modtest(void)
{
	MODPORT |= (1<<WAVEPIN) | (1<<SPEEDPIN) | (1<<DEPTHPIN) | (1<<PWMPIN);
	
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
	_delay_ms(150);
	LEDPORT ^= (1<<LEDPIN);
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
	
	modtest();
	
	LEDDDR |= (1<<LEDPIN);	//LED pin as output
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
	
	uint16_t mstempo;		//The current tempo tapped (it will  be multiplied by the tempo div)
	uint16_t ledtempo;		//tempo for toggling LED (not influenced by tempo div)
	uint16_t delaymax = 1306;	//maximum tempo if not in clean mode
	
	if (modenable == 0)
	{
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
	uint8_t wavetype = 0;
	unsigned long incr;
	
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
			offset = 300 - ((depthvalue * 300)/255);	//updating mod pwm value
			
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
				incr = inc/2;
				if (inc <= 127)
				{
					currentinc = (400 * incr)/127;//triangle
				}
				if (inc > 127)
				{
					currentinc = 400 - ((400*incr)/128);
				}
			}
		
			if (wavetype == 3)
			{
				incr = inc;
				currentinc = (400 * incr)/255;		//sawtooth
			}
		
			if (wavetype == 4)
			{
				incr = inc;
				currentinc = 400 - ((400*incr)/255);//reverse sawtooth
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
				timevalue = (timevalue*127)/255;
			}
			
			SPI_Transmit(timevalue);
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
					divmult = 0.25;	//sextuplet
				}
				
				if (divtogglevalue >= 230)
				{
					divmult = 0.1666666;	//sixteenth
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
