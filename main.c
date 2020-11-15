/*
 * Main module for testing the PWM Code for the K64F
 * 
 * Author:  
 * Created:  
 * Modified: Carson Clarke-Magrab <ctc7359@rit.edu> 
 */ 

#include "MK64F12.h"
#include "uart.h"
#include "pwm.h"
#include "camera_FTM.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h> 

#define A (1<<0)
#define B (1<<1)
#define C (1<<2)
#define D (1<<3)
#define desiredMidpoint 63
#define M_PI 3.14159265358979323846
#define MAX_THETA 90
#define servoMiddle 5
#define leftServoMax 3.6
#define rightServoMax 6.4
#define midpointMin 45
#define midpointMax 80

void delay(int del);

//camera IRQHandlers
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
float MapDifference(float, float, float, float, float);

bool InRange(int,int,int);
double Clamp(double, double, double);

//car mechanics
void MotorSpeed(int);
bool CarpetDetection(int,int);
void SharpRight(int);
void SharpLeft(int);
void ServoDirection(float);

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];
uint16_t newData[128];

// These variables are for streaming the camera
//	 data over UART
char str[200];

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

pid_t PID = {.kp = 5.0, .ki = 0.0, .kd = 100.0};
uint16_t freq0 = 10000; // Frequency = 10 kHz
uint16_t freq3 = 50; // Frequency = 50 Hz 		
uint16_t dir = 0;

float turnCycle;

int main(void) {
	
	// Initialize UART and PWM
	uart_init();
	PWM_init();
	camera_init();
	
	// Print welcome over serial
	// uart0_put("Running... \n\r");
	for(;;)  //loop forever
	{
		// uint16_t dc = 0;
//		char c = 48;
		int firstOne = 0;
		int risingEdge = 0, fallingEdge = 0;
		int actualMidpoint = 0;
		float difference;
		int oneCount;
		double i = 0;
		int j;
				
			// send the array over uart
			//sprintf(str,"%i\n\r",-1); // start value
			//uart0_put(str);
				
			GPIOE_PCOR |= GREEN_LED;
//			motorSpeed(30);			
		
			//smooth trace
			for (j = 0; j < 127; j++) {					
				if(j < 123)
				{
					line[j] = (line[j] + line[j+1] + line[j+2] + line[j+3] + line[j+4])/5;
				}
				else
				{
					line[j] = 127;
				}
			}	
			
			//smoothtrace with simple filter: will change for future
			for (j = 0; j < 127; j++)
			{
				if(line[j] <= 30000)
				{
					newData[j] = 0;
					if(firstOne == 1)
					{
						firstOne = 0;
						fallingEdge = j-1;
					}
				}
				else
				{
					newData[j] = 1;
					oneCount +=1;
					if(firstOne == 0)
					{
						risingEdge = j;
						firstOne = 1;	
					}					
				}
			}
					
						//sprintf(str,"%i\n\r",risingEdge);
						//uart0_put(str);
						//sprintf(str,"%i\n\r",fallingEdge);
						//uart0_put(str);
			
			
			actualMidpoint = (fallingEdge + risingEdge)/2;
			sprintf(str,"%i\n\r",actualMidpoint);
			//uart0_put(str);
			difference = calculatePID(desiredMidpoint, actualMidpoint, &PID);
			//sprintf(str,"%f\n\r",difference);
			//uart0_put(str);
			turnCycle = MapDifference(difference, -100.0, 100.0, leftServoMax, rightServoMax);
			turnCycle = Clamp(turnCycle, leftServoMax, rightServoMax);
			//sprintf(str,"%f\n\r",turnCycle);
			//uart0_put(str);
			//difference = abs(actualMidpoint - desiredMidpoint)/10;
			ServoDirection(turnCycle);
		  sprintf(str,"Rising Edge = %i,  Falling Edge = %i, actualMidpoint = %i oneCount = %i\n\r pidOut = %f\n\r turnCycle = %f\n\r", risingEdge, fallingEdge, actualMidpoint, oneCount, difference, turnCycle);			
			uart0_put(str);			
			//MotorSpeed(30);
			if( CarpetDetection(risingEdge, fallingEdge) ) 	// carpet detection
			{			
				break;
			}
			oneCount = 0;
	}
	return 0;
}	

void SharpRight(int duty_cycle){
	FTM0_set_duty_cycleA(0,freq0,dir);
	FTM0_set_duty_cycleB(duty_cycle,freq0,!dir);
}

void SharpLeft(int duty_cycle){
	FTM0_set_duty_cycleA(duty_cycle,freq0,dir);
	FTM0_set_duty_cycleB(0,freq0,!dir);
}

void MotorSpeed(int duty_cycle){	
	FTM0_set_duty_cycleA(duty_cycle,freq0,dir);
	FTM0_set_duty_cycleB(duty_cycle,freq0,!dir);
}

void ServoDirection(float duty_cycle){
	FTM3_set_duty_cycle(duty_cycle, freq3);
}

bool CarpetDetection(int risingEdge, int fallingEdge){
	if( risingEdge == 0 && fallingEdge == 0 )
	{
		GPIOE_PSOR |= GREEN_LED;
		GPIOB_PCOR |= RED_LED;
		MotorSpeed(0);
		return true;
	}
	else
	{
		return false;
	}
}

bool InRange(int low, int high, int x) 
{ 
    return ((x-high)*(x-low) <= 0); 
}

double Clamp( double value, double min, double max )
{
    if( value >= max ){ return max; }
		if( value <= min) { return min; }
		else{ return value; }
}

float MapDifference(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * ((out_max - out_min) / (in_max - in_min)) + out_min;
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void Delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
		// uart0_put("hello");
	}
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
		// Reading ADC0_RA clears the conversion complete flag
		// Read the result (upper 12-bits). This also clears the Conversion complete flag.
		ADC0VAL = ADC0_RA;
}


/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC &= ~FTM_SC_TOF_MASK;
	
	// Toggle clk
	GPIOB_PTOR |= CAMERA_CLK;
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~FTM_SC_TOIE_MASK;
		while(PIT_TFLG0 != 1)
		{
			FTM2_SC |= FTM_SC_TOIE_MASK;
		}
	
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
		// Clear interrupt
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
		// Setting mod resets the FTM counter
		//FTM2_MOD = 0;
		FTM2_MOD |= ((DEFAULT_SYSTEM_CLOCK/100000)); // unsure
	
		//Enable FTM2 interrupts (camera)
		FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}
