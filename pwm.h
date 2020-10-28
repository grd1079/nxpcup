#ifndef PWM_H_
#define PWM_H_
#include "MK64F12.h"
void PWM_init(void);
void FTM_init(void);
void FTM0_set_duty_cycleA(unsigned int duty_cycle, unsigned int frequency, int dir);
void FTM0_set_duty_cycleB(unsigned int duty_cycle, unsigned int frequency, int dir);
void FTM3_set_duty_cycle(double duty_cycle, unsigned int frequency);
void EN_Init(void);
void EN_A_Toggle(void);
void EN_B_Toggle(void);

#endif /* PWM_H_ */
