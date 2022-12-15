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

#define LF PIND5
#define LB PIND7
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

int stopTrigger = 0;
int stopThreshold = 20;
float safety = 0.35;

int main(void)
{
	initADC();
	initPWM();
	initUSART();
	
	DDRD = (1 << PIND4 | 1 << PIND5 | 1 << PIND7);
	DDRB = (1 << PINB3);
	serialPrintStr("Calibration Start");
	serialPrintStr("\n");
	calibration(4);
	serialPrintStr("Calibration Done");
	serialPrintStr("\n");
    while (1)	
    {
		sensorMapping();
		serialPrintStr("ERROR: ");
		serialPrintInt(error);
		serialPrintStr("\n");
		if(digitalReading[7] == 1)
		{	//brake();
			//_delay_ms(150);
			motor(150,150);
			_delay_ms(100);
			sensorMapping();
			if(mappedValue != 100)
			{
				while(digitalReading[3] != 0 || digitalReading[4] != 0)
				{
					serialPrintStr("Right Acute Turning");
					plannedCRotate();
					sensorMapping();
				}
				while(digitalReading[3] != 1)
				{
					serialPrintStr("Right Acute Turning");
					plannedCRotate();
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
				
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
					plannedACRotate();
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
					plannedCRotate();
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
			}
			
			else if(left == 0 && right == 0)
			{	
				serialPrintStr("Line gap run");
				while(mappedValue == 100)
				{
					motor(sSpeed, sSpeed);
					sensorMapping();
				}
				pid();
				motor(leftSpeed, rightSpeed);
				
			}	
			
			else
			{
				serialPrintStr("Right default turn\n");
				while(digitalReading[3] != 1)
				{
					serialPrintStr("Right default Turning");
					plannedCRotate();
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
	
	if(count == sensorNum)
	{
		stopTrigger++;
	}
	else
	{
		stopTrigger = 0;
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
		analogWrite(RB, 0);
	}
	else
	{	
		analogWrite(RF, 0);
		analogWrite(RB, -rightMotor);
	}
	
	if(leftMotor > 0)
	{
		analogWrite(LF, leftMotor);
		analogWrite(LB, 0);
	}
	else
	{
		analogWrite(LF, 0);
		analogWrite(LB, -leftMotor);
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