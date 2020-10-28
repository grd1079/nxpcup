/*
 * Freescale Cup linescan camera code
 *
 *	This method of capturing data from the line
 *	scan cameras uses a flex timer module, periodic
 *	interrupt timer, an ADC, and some GPIOs.
 *	CLK and SI are driven with GPIO because the FTM2
 *	module used doesn't have any output pins on the
 * 	development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 * 	it enables interrupts from the FTM2 module and then
 *	the FTM2 and ADC are active for 128 clock cycles to
 *	generate the camera signals and read the camera 
 *  output.
 *
 *	PTB8			- camera CLK
 *	PTB23 		- camera SI
 *  ADC0_DP1 	- camera AOut
 *
 * Author:  Alex Avery
 * Created:  11/20/15
 * Modified:  11/23/15
 */

#include "camera_FTM.h"


/* Initialize all camera components */
void camera_init(void){
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	//FTM2_MODE |= FTM_MODE_INIT_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = FTM_CNT_COUNT(0);
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = FTM_CNTIN_INIT(0);
	
	// Set the period (~10us)
	//(Sysclock/128)- clock after prescaler
	//(Sysclock/128)/1000- slow down by a factor of 1000 to go from
	//Mhz to Khz, then 1/KHz = msec
	//Every 1msec, the FTM counter will set the overflow flag (TOF) and
	//FTM2_SC |= FTM_SC_PS(7);			
	//FTM2->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/100000;		// maybe 10us???
	FTM2_MOD |= (DEFAULT_SYSTEM_CLOCK/100000);
	
	// 50% duty
	//FTM2_C0V |= FTM_CnV_VAL((DEFAULT_SYSTEM_CLOCK/(1<<7))/200000);	// maybee??
	FTM2_C0V |= (DEFAULT_SYSTEM_CLOCK/200000);
	
	// Set edge-aligned mode
	FTM2_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
	FTM2_COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;
	FTM2_COMBINE &= ~FTM_COMBINE_COMBINE0_MASK;
	FTM2_SC &= ~FTM_SC_CPWMS_MASK;
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;	
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	
	// Enable hardware trigger from FTM2
	//FTM2_SYNC |= FTM_SYNC_TRIG0_MASK;
	FTM2_EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC &= ~(FTM_SC_PS(0));
	FTM2_SC |= FTM_SC_CLKS(1);	// 1 = 01 (system clock)
	
	// Set up interrupt
	//FTM2_SC |= FTM_SC_TOIE_MASK;	// unsure
	NVIC_EnableIRQ(FTM2_IRQn);	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
		// Setup periodic interrupt timer (PIT)
		
		// Enable clock for timers
		SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
		// Enable timers to continue in debug mode
		// default // In case you need to debug
		PIT_MCR &= ~(PIT_MCR_MDIS_MASK);
		PIT_MCR &= ~(PIT_MCR_FRZ_MASK);
	
		// PIT clock frequency is the system clock
		// Load the value that the timer will count down from
		PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME;
	
		// Enable timer interrupts
		PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; 
	
		// Enable the timer
		PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

		// Clear interrupt flag
		PIT_TFLG0 &= PIT_TFLG_TIF_MASK;

		// Enable PIT interrupt in the interrupt controller
		NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
		//initialize push buttons and LEDs
	LED_Init();

	//set for mux alt1
	PORTB_PCR9 = PORT_PCR_MUX(1);
	PORTB_PCR23 = PORT_PCR_MUX(1);

	//set for output
	GPIOB_PDDR |= CAMERA_CLK | CAMERA_SI;
	//Enable the pins
	GPIOB_PDOR |= CAMERA_CLK | CAMERA_SI | RED_LED;
	return;
}

/* Set up ADC for capturing camera data */  //400mA
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
		SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
		
		// Single ended 16 bit conversion, no clock divider		
    ADC0_CFG1 |= ADC_CFG1_MODE(3);
	
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK; 
    // Set to single ended mode	
		//default
		ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
		ADC0_SC1A &= ~ADC_SC1_ADCH(0x1F);

		// Set up FTM2 trigger on ADC0
		// FTM2 select... binary lsb1010		
		//SIM_SOPT7 = 0;
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10);		
		
		// Alternative trigger en
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;
		
		// Pretrigger A
		SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK;
		
		// Enable interrupt
		ADC0_SC1A |= ADC_SC1_AIEN_MASK;
    NVIC_EnableIRQ(ADC0_IRQn);
}
