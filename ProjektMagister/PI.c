#include "PI.h"
struct PI PI_i_DAB;
struct PI PI_u_DAB;
struct PI PI_i_PLL;
struct PI PI_u_PLL;
struct PI PI_PLL;

void PI_Init(struct PI *pid, float kp, float ki, float min, float max, float Ts) {
    pid->kp = kp;
    pid->ki = ki;
    pid->min = min;
    pid->max = max;
    pid->Ts = Ts;
    pid->a = 0;
    pid->b = 0;
    pid->b1 = 0;
    pid->x = 0;
    pid->x1 = 0;
    pid->y = 0;
    pid->y1 = 0;
    pid->flag = 0;
}

void PI_Calc(struct PI *pid, float x) {
    pid->x = x;

    pid->a = pid->x * pid->kp;

    //antywindup
    if (pid->flag == 1) {
        pid->b = ((0 * pid->ki * pid->Ts) / 2) + ((0 * pid->ki * pid->Ts) / 2) + pid->b1;
    }
    else {
        pid->b = ((pid->x * pid->ki * pid->Ts) / 2) + ((pid->x1 * pid->ki * pid->Ts) / 2) + pid->b1;
    }

    //pierwsze ograniczenie
    if (pid->b > pid->max) {
        pid->b = pid->max;
    }
    if (pid->b < pid->min) {
        pid->b = pid->min;
    }

    pid->e = pid->a + pid->b;
    pid->y = pid->e;


    //drugie ograniczenie
    if (pid->y > pid->max) {
        pid->y = pid->max;
    }
    if (pid->y < pid->min) {
        pid->y = pid->min;
    }

    //antywindupcheck
    if ((pid->y - pid->e) != 0) {
        pid->flag = 1;
    }
    else {
        pid->flag = 0;
    }


    pid->b1 = pid->b;
    pid->x1 = pid->x;
    pid->y1 = pid->y;
}
