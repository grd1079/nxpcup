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
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define A (1<<0)
#define B (1<<1)
#define C (1<<2)
#define D (1<<3)
#define midpoint 63
#define M_PI 3.14159265358979323846
#define MAX_THETA 90
#define servoMiddle 5
//#define servoMiddle 3.5
void delay(int del);
int motorTest(int testNum);

//camera IRQHandlers
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void motorSpeed(int);

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
int debugcamdata = 1;
int capcnt = 0;
char str[100];

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int main(void) {
	
	// Initialize UART and PWM
	uart_init();
	PWM_init();
	camera_init();
	
	// Print welcome over serial
	uart0_put("Running... \n\r");
	
	for(;;)  //loop forever
	{
		// uint16_t dc = 0;
		uint16_t freq0 = 10000; // Frequency = 10 kHz
		uint16_t freq3 = 50; // Frequency = 50 Hz 		
		uint16_t dir = 0;
//		char c = 48;
		int firstOne = 0;
		int risingEdge = 0, fallingEdge = 0;
		int mPoint = 0;
		int difference;
		double i = 0;
		int j;
		
		// camera debugging and operation 	
			// send the array over uart
			sprintf(str,"%i\n\r",-1); // start value
			uart0_put(str);
				
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
				if(line[j] <= 7000)
				{
					newData[j] = 0;						
				}
				else
				{
					newData[j] = 1;
				}
			}
			
			for(j = 0; j < 127; j++)
			{
				if((newData[j] == 1) && (firstOne == 0))
				{
					firstOne = 1;
					risingEdge = j;
				}
				else if((newData[j] == 0) && (firstOne == 1))
				{
					firstOne = 0;
					fallingEdge = j-1;
				}
//					sprintf(str,"%i\n\r",newData[j]);
//					sprintf(str,"%i\n\r",line[j]);
//					uart0_put(str);
			}
			
			mPoint = (fallingEdge + risingEdge)/2;
			difference = abs(mPoint - midpoint)/10;
			
			if( mPoint > midpoint)
			{
				//turn right
//				motorSpeed(10);
				FTM3_set_duty_cycle(5 - difference*.75, freq3);
//				sprintf(str,"duty_cycle: %i\n\r", servoMiddle - difference*.5);
//				uart0_put(str);
			}
			else if( mPoint < midpoint)
			{
				//turn left
//				motorSpeed(10);				
				FTM3_set_duty_cycle(5 + difference*.75, freq3);
//				sprintf(str,"duty_cycle: %i\n\r", servoMiddle + difference*.5);
//				uart0_put(str);
			}
			sprintf(str,"Rising Edge = %i,  Falling Edge = %i, Midpoint = %i\n\r",risingEdge,fallingEdge,mPoint);
			//sprintf(str,"%i\n\r",-2); // end value
			uart0_put(str);
//			if( risingEdge == 0 && fallingEdge == 0 ) 	// carpet detection
//			{
//				//brake
//				GPIOB_PCOR |= (1 << 22);
////				motorSpeed(0);
//				break;
//			}				
	}
	return 0;
}	

void motorSpeed(int duty_cycle){	
	FTM0_set_duty_cycleA(duty_cycle,10000,0);
	FTM0_set_duty_cycleB(duty_cycle,10000,!0);
}

float calculateAngle(int left, int right, int center) {
    // See if we're the left side or the right side.
    // Sin function will only report a positive angle.
    float angle = 0;
    float diff = 0;
    // Leaning to the right
    if (center < midpoint) {
        diff = ((float)(center - midpoint))/((float)midpoint);
        angle = acosf(diff);
        // Shifts it to the angle we want
        angle = MAX_THETA - angle;
    }
    // Leaning to the left
    else if (center > midpoint) {
        diff = ((float)(midpoint - center))/((float)midpoint);
        angle  = acosf(diff);
        // Shifts it to the angle we want
        angle = angle - MAX_THETA;
    }
    // else: Dead center :)
    angle = ((angle * 180.0f) / M_PI);
    return angle;
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
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
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
		// Clear interrupt
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
		// Setting mod resets the FTM counter
		//FTM2_MOD = 0;
		FTM2_MOD |= ((DEFAULT_SYSTEM_CLOCK/100000)); // unsure
	
		//Enable FTM2 interrupts (camera)
		FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}
