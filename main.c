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
#include "stdio.h"

#define A (1<<0)
#define B (1<<1)
#define C (1<<2)
#define D (1<<3)
void delay(int del);
int motorTest(int testNum);

//camera IRQHandlers
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

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
	
	// Part 1 - UNCOMMENT THIS
	// Generate 20% duty cycle at 10kHz
	// FTM0_set_duty_cycle(75, 10000, 0);
	// FTM2_set_duty_cycle(75, 50, 0);
	// for(;;) ;  //then loop forever
	
	
	// Part 2 - UNCOMMENT THIS
	//  Enable  clocks  on Port D
	// SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	// Configure the Signal Multiplexer for GPIO
	// PORTD_PCR0 = PORT_PCR_MUX(1);	//PTD0
	// PORTD_PCR1 = PORT_PCR_MUX(1);	//PTD1
	// PORTD_PCR2 = PORT_PCR_MUX(1);	//PTD2
	// PORTD_PCR3 = PORT_PCR_MUX(1);	//PTD3

	// Switch the GPIO pins to output mode
	// GPIOD_PDDR |= 0x0F;
	
    // int  forward = 1;
    // int  phase = 0;
	// int  testNum = 1;
	
	for(;;)  //loop forever
	{
		// uint16_t dc = 0;
		uint16_t freq0 = 10000; // Frequency = 10 kHz
		uint16_t freq3 = 50; // Frequency = 50 Hz 		
		uint16_t dir = 1;
//		char c = 48;
		double i=0;
		
		// camera debugging and operation 
		if (debugcamdata) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (500)) {
				GPIOB_PCOR |= (1 << 22);
				// send the array over uart
				sprintf(str,"%i\n\r",-1); // start value
				uart0_put(str);
				for (int i = 0; i < 127; i++) {
					sprintf(str,"%i\n", line[i]);
					uart0_put(str);
				}
				sprintf(str,"%i\n\r",-2); // end value
				uart0_put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}
		
		FTM0_set_duty_cycleA(30,freq0,dir);
		FTM0_set_duty_cycleB(30,freq0,dir);

		// 0 to 100% duty cycle in forward direction
//		for (i=0; i<100; i++) {
//			FTM0_set_duty_cycleA(30, freq0, !dir);
//			FTM0_set_duty_cycleB(30, freq0, !dir);
//			delay(10);
//		}

//			delay(20);
//        
//		for (i=0; i<100; i++) {
//				FTM0_set_duty_cycle(30, freq0, dir);
//				delay(10);
//				
//		}
			
//			delay(20);
//        
//		// 100% down to 0% duty cycle in the forward direction
//		for (i=100; i>=0; i--) {
//			FTM0_set_duty_cycle(i, freq0, dir);
//			delay(10);
//		}

		// 0 to 100% duty cycle in reverse direction
//		for (i=0; i<100; i++) {
//			FTM0_set_duty_cycle(i, freq0, !dir);	
//			delay(10);
//			
//		}

//		// 100% down to 0% duty cycle in the reverse direction
//		for (i=100; i>=0; i--) {
//			FTM0_set_duty_cycle(i, freq0, !dir);
//			delay(10);
//		}		
		// 100% down to 0% duty cycle in the reverse direction
		// 5.43 = neutral
//		for (i=1;i<=100;i++){			
//			FTM3_set_duty_cycle(i, freq3);
//		
		//delay(20000);
		for (i=4;i<=8;i+=.1){			
			FTM3_set_duty_cycle(i, freq3);
			delay(10);
		}
//		delay(100);
	}
	return 0;
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

int motorTest(int testNum){
	//int i;
	int dir = 1; // forward
	if (testNum > 3) {
		 dir = 0; // reverse
	
	}
	switch (testNum) {
		//full speed forward
		case 1:
			delay(1);
			break;
		//half speed forward
		case 2:
			delay(5);
			break;
		//quarter speed forward
		case 3:
			delay(10);
			break;

		//quarter speed reverse
		case 4:
			delay(10);
			break;

		//half speed reverse
		case 5:
			delay(5);
			break;
		//full speed reverse
		case 6:
			delay(1);
			break;					
	}
	return dir;
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
