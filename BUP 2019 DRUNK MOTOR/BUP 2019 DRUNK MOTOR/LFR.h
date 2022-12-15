#ifndef LFR
#define LFR

#ifndef BAUD
#define BAUD 9600
#endif


#include <string.h>
#include <stdlib.h>
#include <avr/io.h>

void initUSART(void);
void transData(unsigned char data);
void initADC(void);
uint16_t readADC(uint8_t pin);
void initPWM(void);
void analogWrite(int pin, int width);



void initADC(void)
{
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);	//128 Prescalar selection
	ADMUX |= (1 << REFS0) ;					//Capacitor AVCC reference
	ADMUX &= ~(1 << ADLAR);					//10bit right shift
	ADCSRA |= (1 << ADEN);									//turn on adc feature and interupt
	

	
	ADCSRA |= (1 << ADSC);									//first conversion
}


uint16_t readADC(uint8_t pin)
{
	ADMUX &= 0xF0;								//Clearing MUX3-0 bits (MUX4 left unchanged)
	ADMUX |= pin;								//selecting Analog Pin
	ADCSRA |= (1 << ADSC);						//start conversion
	
	while(!( bit_is_clear(ADCSRA, ADSC)));		//waiting until conversion is done
	
	uint8_t low = ADCL;
	
	uint16_t adcVal = (ADCH << 8) | low;
	
	return adcVal;
	
}



void initPWM(void)
{

		//case PIND7:
		TCCR2 = (1 << WGM20 | 1 << WGM21 | 1 << COM21 | 1 << CS20);

		
		//case PINB3:
		TCCR0 = (1 < WGM01 | 1 << WGM00 | 1 << COM01 | 1 << CS00);

		
		//case PIND5:
		//TCCR1A |= (1 << WGM10 | 1 << COM1A1);
		//TCCR1B |= (1 << WGM12 | 1 << CS10);

		//case PIND4:
		//TCCR1A |= (1 << WGM10 | 1 << COM1B1);
		//TCCR1B |= (1 << WGM12 | 1 << CS10);
}
		





void analogWrite(int pin, int width)
{
	switch(pin)
	{
		case PIND7:
		OCR2 = width;
		break;
		
		case PINB3:
		OCR0 = width;
		break;
		
		case PIND5:
		OCR1A = width;
		break;
		
		case PIND4:
		OCR1B = width;
		break;
	}

}






void initUSART(void)
{
	int ubbr = (F_CPU/(16*BAUD))-1;

	UBRRH = (unsigned char)(ubbr>>8); //higher bits of ubbr
	UBRRL = (unsigned char)ubbr;		//lower bits of ubbr


	UCSRB |= (1<<RXEN)|(1<<TXEN); // enabling transimitter and receiver
	
	UCSRC |= (1<<URSEL)|(1<<USBS)|(3<<UCSZ0); //2 stop bits and 8bit data frame setup
	
}

void transData(unsigned char data)
{
	while(! ( UCSRA & (1 << UDRE))); //waiting transmission for buffer to be free
	
	UDR = data;
}


void serialPrintStr(char *text)
{
	for(int i = 0; i < strlen(text); i++)
	{
		transData(text[i]);
	}
}

void serialPrintInt(int num)
{
	char buffer[10];
	itoa(num, buffer, 10);
	serialPrintStr(buffer);
}




#endif