/**
 * main.c
 */
#include "DSP28x_Project.h"

void setupGPIO(){
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0x01;
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1;
    GpioCtrlRegs.GPADIR.bit.GPIO0=0;
    EDIS;
}

void setupPWM(){
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x09;  // 10
    EPwm1Regs.TBCTL.bit.CLKDIV=0x09;     // 32
    EPwm1Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm1Regs.AQCTLA.bit.CBU=0x02;       // disable output A when compare A and counting UP
    EPwm1Regs.AQCTLA.bit.CBD=0x01;       // enable output A when compare A and counting DOWN
    EPwm1Regs.TBPRD=0xB71B;            // 46875 , PWM f = 150mhz/32/10 = 468750 / 2*46875 = 5,
    EPwm1Regs.CMPB=0x249F;         // 0.2 * 46875= 9375
    EDIS;
}



int main(void)
{
    InitSysCtrl();
    setupPWM();
    setupGPIO();
    while(1){
        __asm(" NOP");
    }
}

