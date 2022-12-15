#ifndef LFR
#define LFR

#ifndef BAUD
#define BAUD 9600
#endif


#include <string.h>
#include <stdlib.h>
#include <avr/io.h>

#define trig1_PORT PORTC
#define trig1_PIN PINC1
#define trig2_PIN PINC0
#define trig3_PIN PINC2

void initUSART(void);
void transData(unsigned char data);
void initADC(void);
uint16_t readADC(uint8_t pin);
void initPWM(void);
void analogWrite(int pin, int width);
uint16_t sonar1Read(void);
uint16_t sonar2Read(void);
uint16_t sonar3Read(void);

//1 sec = 62500

int sonar1_ack = 0;
int sonar2_ack = 0;
int sonar3_ack = 0;

uint16_t pulse1 = 0;
uint16_t pulse2 = 0;
uint16_t pulse3 = 0;



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



uint16_t sonar1Read(void)
{
	sonar1_ack = 0;
	trig1_PORT &= ~(1 << trig1_PIN);
	trig1_PORT |= (1 << trig1_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig1_PIN);
	
	while (sonar1_ack == 0)
	{
		TCCR1B |= (1 << CS11);
		if ((PIND & (1 << PIND2)))
		{
			TCNT1 = 0;
			TCCR1B |= (1 << CS11);
			while ((PIND & (1 << PIND2)))
			{
				if (TCNT1 > 5000)
				{
					TCCR1B = 0;
					TCNT1 = 0;
					return 111;
				}
			}
			TCCR1B = 0;
			pulse1 = TCNT1 / 2;
			TCNT1 = 0;
			sonar1_ack = 1;
			//          Serial_sendInt(pulse1, DEC);
			//          Serial_sendString("\t");
		}
		else if (TCNT1 > 10000)
		{
			TCCR1B = 0;
			TCNT1 = 0;
			return 112;
		}
	}
	return (.035 * pulse1 / 2);
}


uint16_t sonar2Read(void)
{
	sonar2_ack = 0;
	trig1_PORT &= ~(1 << trig2_PIN);
	trig1_PORT |= (1 << trig2_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig2_PIN);
	
	while (sonar2_ack == 0)
	{
		TCCR1B |= (1 << CS11);
		if ((PINB & (1 << PINB2)))
		{
			TCNT1 = 0;
			TCCR1B |= (1 << CS11);
			while ((PINB & (1 << PINB2)))
			{
				if (TCNT1 > 5000)
				{
					TCCR1B = 0;
					TCNT1 = 0;
					return 111;
				}
			}
			TCCR1B = 0;
			pulse2 = TCNT1 / 2;
			TCNT1 = 0;
			sonar2_ack = 1;
			//          Serial_sendInt(pulse1, DEC);
			//          Serial_sendString("\t");
		}
		else if (TCNT1 > 10000)
		{
			TCCR1B = 0;
			TCNT1 = 0;
			return 112;
		}
	}
	return (.035 * pulse2 / 2);
}



uint16_t sonar3Read(void)
{
	sonar3_ack = 0;
	trig1_PORT &= ~(1 << trig3_PIN);
	trig1_PORT |= (1 << trig3_PIN);
	_delay_us(10);
	trig1_PORT &= ~(1 << trig3_PIN);
	
	while (sonar3_ack == 0)
	{
		TCCR1B |= (1 << CS11);
		if ((PIND & (1 << PIND3)))
		{
			TCNT1 = 0;
			TCCR1B |= (1 << CS11);
			while ((PIND & (1 << PIND3)))
			{
				if (TCNT1 > 5000)
				{
					TCCR1B = 0;
					TCNT1 = 0;
					return 111;
				}
			}
			TCCR1B = 0;
			pulse3 = TCNT1 / 2;
			TCNT1 = 0;
			sonar3_ack = 1;
			//          Serial_sendInt(pulse1, DEC);
			//          Serial_sendString("\t");
		}
		else if (TCNT1 > 10000)
		{
			TCCR1B = 0;
			TCNT1 = 0;
			return 112;
		}
	}
	return (.035 * pulse3 / 2);
}




#endif