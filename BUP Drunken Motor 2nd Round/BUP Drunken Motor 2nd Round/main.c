/*
 * BUP 2019.c
 *

 * Author : Rusab
 */ 

#define F_CPU 16000000UL
#define sensorNum 8
#define BAUD 38400

#define maxSpeed 255
#define sSpeed 200

#define revSpeed 150

#define LF PIND7
#define LB PIND5
#define RF PINB3
#define RB PIND4



#include <avr/io.h>
#include <util/delay.h>
#include "LFR.h"

void sensorMapping(void);

int blackLimit[sensorNum];
int digitalReading[sensorNum];
int mappedValue;
int targetValue = 7;
int left, right;
float error, prevError = 0;
float kp = 47;
float kd = 57;
int motorResponse;
float correction;

int rotationSpeed = 255;
int leftSpeed, rightSpeed;
int extremeCounter = 0;
int extremeTrigger = 15;

int linegapcount = 0;


int stopTrigger = 0;
int stopThreshold = 20;
float safety = 0.35;

int near = 10;
int far = 16;

uint16_t rDistance = 111;
uint16_t lDistance = 111;
uint16_t fDistance = 111;

int obsTrigger = 20;

const int rotateDelay = 350;
const int blindDelay = 350;
const int curvature = 50;
const int revDelay = 200;

int allBlack = 0;

int main(void)
{
	initADC();
	initPWM();
	initUSART();
	
	//sonar pins
	DDRC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2); //trig
	DDRD &= ~ ((1 << PIND2) | (1 << PIND3)); //echo
	DDRB &= ~ (1 << PINB2); //echo
	//motor pins
	DDRD = (1 << PIND4 | 1 << PIND5 | 1 << PIND7);
	DDRB = (1 << PINB3);
	serialPrintStr("Calibration Start");
	serialPrintStr("\n");
	calibration(2);
	serialPrintStr("Calibration Done");
	serialPrintStr("\n");
    while (1)	
    {
	//	motor(-200, -200);
		
		sensorMapping();
		serialPrintStr("ERROR: ");
		serialPrintInt(error);
		serialPrintStr("\n");
		
		if(stopTrigger > stopThreshold)
		{
			sensorMapping();
			while(allBlack == 1)
			{
				brake();
				sensorMapping();
			}
			
		}
		
	
		if(mappedValue != 100)
		{
			pid();
			motor(leftSpeed, rightSpeed);
			serialPrintStr("Left: ");
			serialPrintInt(leftSpeed);
			serialPrintStr("\n");
			serialPrintStr("Right: ");
			serialPrintInt(rightSpeed);
			serialPrintStr("\n");
		}
		
		else
		{
			if(left == 1 && right == 0)
			{
				serialPrintStr("Left Acute Turn\n");
				while(digitalReading[3] != 1)
				{
					serialPrintStr("Left Acute Turning");
					plannedACRotate(150);
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
			}
			
			else if(left == 0 && right == 1)
			{
				serialPrintStr("Right Acute Turn\n");
				while(digitalReading[3] != 1)
				{
					serialPrintStr("Right Acute Turning");
					plannedCRotate(150);
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
			}
			
			else if(left == 0 && right == 0)
			{	
				triggerSonars();
				if(rDistance < far || lDistance < far)
				{
					motor(150,150);
					_delay_ms(100);
					while(mappedValue == 100)
					{
						//wallRun();
						sensorMapping();
					}	
				}
				
				else if(fDistance < obsTrigger)
				{
					//curveRun();
					sensorMapping();
				}
				else
				{
					serialPrintStr("Line gap run");
					if(linegapcount  == 0)
					{
						while(mappedValue == 100)
						{
							motor(sSpeed, sSpeed+10);
							sensorMapping();
							triggerSonars();
							if(fDistance < obsTrigger)
							{
							//	curveRun();
								sensorMapping();
							}
						}
						pid();
						motor(leftSpeed, rightSpeed);
						linegapcount++;
					}
					else
					{
						while(mappedValue == 100)
						{
							motor(sSpeed, sSpeed);
							sensorMapping();
							triggerSonars();
							if(fDistance < obsTrigger)
							{
								//curveRun();
								sensorMapping();
							}
						}
						pid();
						motor(leftSpeed, rightSpeed);
					
					}

				}
			}	
			
			else
			{
				serialPrintStr("Right default turn\n");
				while(digitalReading[3] != 1)
				{
					serialPrintStr("Right default Turning");
					plannedCRotate(150);
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
				
			}	
		}
		
	/*
		//plannedCRotate(100);
		//analogWrite(RF, 200);
		for(int i = 0; i < 8; i++)
		{
			serialPrintInt(readADC(i) < blackLimit(i));
			serialPrintStr(" ");
		}
		serialPrintStr("\n");
		_delay_ms(500);
	*/
	}
	
}


void sensorMapping(void)
{
	int sum = 0;
	int count = 0;
	
	for(int i = 0; i < 8; i++)
	{
		if(readADC(i) < blackLimit[i])
		{
			sum += i*2;
			count++;
			digitalReading[i] = 1;
		}	
		
		else
		{	
			digitalReading[i] = 0;
		}
		
	}
	
	if(count != 0)
	{
		mappedValue = sum / count;
	}
	else
	{
		mappedValue = 100; //All white
	}
	
	if(digitalReading[0]||digitalReading[sensorNum-1])  //if left or right gives black saves their value
	{
		left = digitalReading[0];
		right = digitalReading[sensorNum-1];
		serialPrintStr("Acute detected\n");
	}
	else
	{
		if(mappedValue != 100 && (left == 1 || right == 1))
			extremeCounter++;
	}
	
	if(extremeCounter > extremeTrigger)
	{
		left = 0;
		right = 0;
		extremeCounter = 0;
		serialPrintStr("Acute Reset\n");
	}
	
	if(count > 5)
	{
		stopTrigger++;
		allBlack = 1;
	}
	else
	{
		stopTrigger = 0;
		allBlack = 0;
	}
	
}

void triggerSonars(void)
{
	fDistance = sonar3Read();
	rDistance = sonar1Read();
	lDistance = sonar2Read();
	serialPrintStr("FDistance = ");
	serialPrintInt(fDistance);
	serialPrintStr("\n");
}

void wallRun()
{
	lDistance = sonar2Read();
	serialPrintStr("\nLeftSonar : ");
	serialPrintInt(lDistance);
	serialPrintStr(" \n");
	
	rDistance = sonar1Read();
	serialPrintStr("\nRightSonar: ");
	serialPrintInt(rDistance);
	serialPrintStr(" \n");
	
	fDistance = sonar2Read();
	serialPrintStr("\nFrontSonar: ");
	serialPrintInt(fDistance);
	serialPrintStr(" \n");
	
	
	
	
	
	//wall run
	if(rDistance <= near)
	{
		//plannedACRotate(150);
		serialPrintStr("Wall is near\n");
		motor(100, 170);
		//serialPrintStr("fRONT turn \n");
		//_delay_ms(350);
		//fDistance = sonar3Read();
		//rDistance = sonar1Read();
	}
	

	else if(lDistance != 0 && lDistance != 112)
	{
		if(lDistance == 111 && rDistance == 111)
		{
			motor(100,150);
		}
		else if(lDistance <= near)
		{
			serialPrintStr("Wall is near\n");
			motor(150, 100);
		}
		
		else if(fDistance <= near)
		{
			//serialPrintStr("Wall is near\n");
			motor(100, 150);
		}
		else if(lDistance > near && lDistance < far)
		{
			serialPrintStr("Wall Running\n");
			motor(100, 100);
		}
		else if(lDistance >= far)
		{
			serialPrintStr("Wall is far\n");
			motor(150, 100);
		}
		
		
		else
		{
			/*
			brake();
			motor(100, 100);
			_delay_ms(800);
			plannedACRotate();
			_delay_ms(350);
			motor(100, 100);
			_delay_ms(300);
			//fDistance = sonar3Read();
			lDistance = sonar2Read();
			*/
		}
	}
	
	else if(lDistance == 0 || lDistance == 112)
	{
		lDistance = sonar2Read();
		rDistance = sonar1Read();
		fDistance = sonar3Read();
	}
	
}

void curveRun(void)
{
	motor(-150, -150);
	serialPrintStr("Reverse\n");
	_delay_ms(revDelay);
	
	plannedCRotate(150);
	serialPrintStr("Rotate\n");
	_delay_ms(rotateDelay);
	serialPrintStr("blind run\n");
	motor(140, 180+curvature);
	_delay_ms(blindDelay);
	while(mappedValue == 100)
	{
		serialPrintStr("scanning run\n");
		motor(140, 180+curvature);
		sensorMapping();
	}
}

void wallRun2(void)
{
	triggerSonars();
	if(rDistance > far)
	{
		motor(150, 100);
	}
	else if(lDistance > far)
	{
		motor(100, 165);
	}
	else
	{
		motor(100, 100);
	}
}

void pid(void)
{
	error = targetValue - mappedValue;
	correction = (kp*error)+(kd*(error-prevError));
	prevError = error;
	motorResponse = (int)correction;
	
	if(motorResponse > maxSpeed)
		motorResponse = maxSpeed;
	if(motorResponse < -maxSpeed)
		motorResponse = -maxSpeed;
	
	if(motorResponse > 0)
	{
		rightSpeed = maxSpeed;
		leftSpeed = maxSpeed - motorResponse;
	}
	
	else if(motorResponse < 0)
	{
		rightSpeed = maxSpeed + motorResponse;
		leftSpeed = maxSpeed;	
	}
	
	else
	{
		leftSpeed = sSpeed;
		rightSpeed = sSpeed;
	}
}

void motor(int leftMotor, int rightMotor)
{
	if(leftMotor > 0)
	{
		leftMotor = motorBalance(leftMotor);
	}
	else
	{
		leftMotor = -motorBalance(-leftMotor);
	}
	if(rightMotor > 0)
	{	
		analogWrite(RF, rightMotor);
		PORTD &= ~(1 << PIND4);
		
		//analogWrite(RB, 0);
	}
	else
	{	
		analogWrite(RF, 255 + rightMotor);
		//analogWrite(RB, -rightMotor);
		PORTD |= ( 1 << PIND4);
		
	}
	
	if(leftMotor > 0)
	{
		analogWrite(LF, leftMotor);
		//analogWrite(LB, 0);
		PORTD &= ~(1 << PIND5);
	}
	else
	{
		
		//analogWrite(LB, -leftMotor);
		PORTD |= (1 << PIND5);
		analogWrite(LF, 255 + leftMotor);
	}	
}


void plannedCRotate(int rotationSpeed)
{
	leftSpeed = rotationSpeed;
	rightSpeed = -rotationSpeed;
	motor(leftSpeed, rightSpeed);
}

void plannedACRotate(int rotationSpeed)
{
	leftSpeed = -rotationSpeed;
	rightSpeed = rotationSpeed;	
	motor(leftSpeed, rightSpeed);
}

void brake(void)
{
	motor(0,0);
}

void reverseGear(void)
{
	motor(-revSpeed, -revSpeed);
}

int motorBalance(int speed)
{
	float y = 0.7534*speed + 28.9;
	int newSpeed = (int) y;
	return newSpeed;
	
}

void calibration(int time)
{	
	serialPrintStr("Calibrating");
	plannedCRotate(100);
	float upSum = 0,lowSum = 0;
	int sensorArray[sensorNum][2];

	for(int i = 0; i < sensorNum; i++)
	{
		sensorArray[i][0] = readADC(i);
		sensorArray[i][1] = readADC(i);
	}
	

	int loopCounter = (int)(time * 1000 / 2.5);
	while(loopCounter)
	{
		serialPrintStr("Calibrating");
		serialPrintStr("\n");
		for(int i = 0; i < sensorNum; i++)
		{
			if(readADC(i)<sensorArray[i][0]) sensorArray[i][0]=readADC(i);
			if(readADC(i)>sensorArray[i][1]) sensorArray[i][1]=readADC(i);
		}
		loopCounter--;

	}

	for(int i=0; i < sensorNum; i++)
	blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

	brake();
	_delay_ms(1000);
} 