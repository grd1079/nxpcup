#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "uart.h"

#include <stdbool.h> 
/** Internal functions **/
float calculatePorportional(pid_t* control, float error);
float calculateIntegration(pid_t* control, float error);
float calculateDerivative(pid_t* control, float error);
char str2[200];

float calculatePID(const float desired, const float actual, pid_t* control) {
    // Calculate error
    float error = desired - actual;
		sprintf(str2,"%f\n\r", error);
		//uart0_put(str2);
    // Calculate PID components
    control->pid_p = calculatePorportional(control, error);
    control->pid_i = calculateIntegration(control, error);
    control->pid_d = calculateDerivative(control, error);
    // Store diffs
    control->diff = error;
    return (control->pid_p + control->pid_i + control->pid_d/100.0);
}

float calculatePorportional(pid_t* control, float error) {
    return control->kp * error;
}

float calculateIntegration(pid_t* control, float error) {
    control->sigma = control->sigma + error;
    return (control->sigma * control->ki);
}

float calculateDerivative(pid_t* control, float error) {
    return (((error - control->diff) * control->kd));
}
