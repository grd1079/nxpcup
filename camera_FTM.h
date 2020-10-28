#ifndef CAMERA_FTM_H_
#define CAMERA_FTM_H_
#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void camera_init(void);
void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
#endif /* ifndef CAMERA_FTM_H_ */
