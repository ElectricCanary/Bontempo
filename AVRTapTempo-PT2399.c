/*
 * AVR TapTempo
 *
 * Created: 20/06/2019
 *  Author: Antoine Ricoux
 */ 
#include <avr/io.h>
#define 	F_CPU   8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/eeprom.h>

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
#define DEBOUNCE_TIME 800		//Define debounce Time in microseconds
#define MAXDELAY 1138			//Define max & min delay time in ms : maxdelay = (11.46 * (R+maxpot))+29.7; mindelay = (11.46 * R)+29.7
#define MINDELAY 40

volatile float mstempo;				//Volatile for variable shared by main and other functions
volatile uint8_t potvalue = 0;
volatile uint8_t previouspotvalue = 1;
volatile float mult;

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

uint8_t potdebounce(void)			// Very messy way to debounce the pot, for not interrupting the tap tempo randomly
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

int getClosest(int, int, int, int, int); 
  
// Returns element closest to target in arr[] 
int findClosest(int arr[], int n, int target) 
{ 
    // Corner cases 
    if (target <= arr[0]) 
        return 0; 
    if (target >= arr[n - 1]) 
        return n - 1; 
  
    // Doing binary search 
    int i = 0, j = n, mid = 0; 
    while (i < j) { 
        mid = (i + j) / 2; 
  
        if (arr[mid] == target) 
            return mid; 
  
        /* If target is less than array element, 
            then search in left */
        if (target < arr[mid]) { 
  
            // If target is greater than previous 
            // to mid, return closest of two 
            if (mid > 0 && target > arr[mid - 1]) 
                return getClosest(arr[mid - 1], 
                                  arr[mid], target, mid-1, mid); 
  
            /* Repeat for left half */
            j = mid; 
        } 
  
        // If target is greater than mid 
        else { 
            if (mid < n - 1 && target < arr[mid + 1]) 
                return getClosest(arr[mid], 
                                  arr[mid + 1], target, mid, mid+1); 
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

int main(void)
{
	SPIDDR |= (1<<MOSIPIN) | (1<<SCKPIN) | (1<<SSPIN);			//Setting MOSI, SCK & SS| as Output
	SPIPORT |= (1<<SSPIN);										//Setting SS| high
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); //Turning SPI on, as master, clock rate with 16 prescaler, LSB first
	
	startup();						//Waiting 400ms with pot at 50k to avoid boot problem with PT2399
	
	LEDDDR |= (1<< LEDPIN);			//LED pin output and set to low
	LEDPORT &= ~(1<<LEDPIN);
	BUTTONDDR &= ~(1<<BUTTONPIN);	//Button pin input and set to high
	BUTTONPORT |= (1<<BUTTONPIN);
	//TOGGLEDDR |= (1<<TOGGLEPIN);
	TOGGLEPORT |= (1<<TOGGLEPIN);
	
	ADMUX |= (1<<REFS0);			//ADC reference to Vcc 
	ADMUX |= (1<<MUX1);				//Selecting AD2 (PINC2)
	ADMUX |= (1<<ADLAR);			//for 8bit ADC
	ADCSRA |= (1<<ADIE);			//Activate ADC interrupts
	ADCSRA |= (1<<ADPS2);			//Setting ADC prescaler to 16
	ADCSRA |= (1<<ADEN);			//Turning ADC on
	
	TCCR1B |= 1<<CS11;				// Counter reference set to clock/8 (1MHz)
	
	float ms;
	float data;
	
	unsigned int turns = 0;		//1turn = 1ms = 1000 TCNT1
	unsigned int ledturns = 0;
	
	unsigned int nbpress = 0;	// Number of presses during the tap tempo sequence
	uint8_t laststate = 0;		//Previous state of the button
	uint8_t tap = 0;			//Buton control (0) or Tap tempo (1) ? 
	
	uint8_t commandbyte = 0b00010001;
	uint8_t databyte;
	
	float previousmult;
	
	
	uint16_t tempo[]={41,42,44,47,52,58,63,68,73,77,82,86,90,95,99,104,108,112,117,121,
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
	
	int n = sizeof(tempo) / sizeof(tempo[0]);	//For the findClosest function
	
	sei();							//Activate interrupts
	ADCSRA |= (1<<ADSC);			//Starting Conversion
	
    while(1)
    {
		
		if(potdebounce()==1)	//if the pot moves, pot overrides tap 
		{
			//mstempo = (potvalue*((MAXDELAY-MINDELAY)/255)+MINDELAY); //Adjusting the pot range (0-255) to delay range
			databyte = potvalue;			 						//Adjusting the pot range to digital pot range
			SPI_MasterTransmit(commandbyte,databyte);				//Transmit 16bit word
			previouspotvalue = potvalue;
			tap = 0;
		}
		
		if(previousmult != mult && tap == 1)					//If toggle changes with no tap (in tap mode), update the digital pot value
		{
			data = findClosest(tempo,n,(mstempo*mult));
			databyte = data;
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
			
			data = findClosest(tempo, n, round(mstempo*mult));
			databyte = data;
			mstempo = tempo[findClosest(tempo,n,mstempo)];
			SPI_MasterTransmit(commandbyte,databyte);			//Transmits 16bit word
			
			nbpress++;					//restarts counting, for next tap
			laststate = 1;
			tap = 1;
			TCNT1 = 0;
			turns = 0;
			ms=0;
		}
		
		
		if(tap==0)
		{
			LEDPORT |= (1 << LEDPIN);
		}
		if (ledturns > mstempo && tap ==1)				//Toggles Led every downbeat
		{
			LEDPORT |= (1 << LEDPIN);
			_delay_ms(5);
			LEDPORT &= ~(1<<LEDPIN);
			ledturns = 0;
			
		}			
    }
}

ISR(ADC_vect)								// Interrupts when conversion is finished
{
	uint8_t togglevalue;
	switch(ADMUX)							//Switches between the pot and toggle ADC
	{
		case 0x62 :
		potvalue = ADCH;					//Stocks 8-bit pot value
		ADMUX |= (1<<MUX0);					//Changing to toggle ADC pin for next interrupt
		break;
		
		case 0x63 :
		togglevalue = ADCH;					//Stocks 8-bit toggle value
		
		if(togglevalue <= 20)				//if pin low : eighth
		{
			mult = 0.5;
		}
		if(togglevalue >= 230)				//if pin high : fourth
		{
			mult = 1;
		}
		if(togglevalue > 20 && togglevalue < 130)
		{
			mult = 0.75;					//Anything in between : dotted eighth
		}
		ADMUX &= ~(1<<MUX0);				//Changing to pot ADC pin for next interrupt
		break;
		
	}
	
	ADCSRA |= (1<<ADSC);					//Starts new conversion since previous one is finished
}