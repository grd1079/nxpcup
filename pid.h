#ifndef PID_H
#define PID_H

typedef struct PID {
    /* Configuration parameters */
    float kp;
    float ki;
    float kd;
    /* Output */
    float pid_p;
    float pid_i;
    float pid_d;
    /* Private */
    float sigma; // sum all integration terms
    float diff; // previous differential term
} pid_t;

float calculatePID(const float desired, const float actual, pid_t* control);

#endif
