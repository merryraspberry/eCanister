
/**
 * main.c
 */
#include "DSP28x_Project.h"

void setupGPIO(){
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0x01; //epwm1A
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1;
    GpioCtrlRegs.GPADIR.bit.GPIO0=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO1=0x01; //epwm1B
    GpioCtrlRegs.GPAPUD.bit.GPIO1=1;
    GpioCtrlRegs.GPADIR.bit.GPIO1=0;
    EDIS;
}

void setupPWM(){
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE=2;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x05; //10 f wyjsciowe jak najwieksza dzielnik jka najmniejszy
    EPwm1Regs.TBCTL.bit.CLKDIV=0x00;    //1
    EPwm1Regs.TBCTL.bit.FREE_SOFT=0x02;
    EPwm1Regs.AQCTLA.bit.CAU=0x02;
    EPwm1Regs.AQCTLA.bit.CAD=0x01;
    EPwm1Regs.TBPRD=0x2EE;   //750
    EPwm1Regs.CMPA.half.CMPA=0x20D; //0.7*7500 = 525

    //DEAD - BAND setup
    EPwm1Regs.DBCTL.bit.IN_MODE=0x00; //EPWMA source for both rising and falling
    EPwm1Regs.DBCTL.bit.POLSEL=0x02;  //AHC mode, epwmb is inverted
    EPwm1Regs.DBCTL.bit.OUT_MODE=0x03; //deadband fully on

    EPwm1Regs.DBFED=0x4B; //75 cycles TBCLK 5us
    EPwm1Regs.DBRED=0x4B; //75 cycles TBCLK 5us

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

