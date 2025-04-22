#include "DSP28x_Project.h"
/**
* diodes.c
*/

int pb1 = 0;
int pb2 = 0;

unsigned int vol0 = 0;
unsigned int vol1 = 0;
unsigned int vol2 = 0;
unsigned int vol3 = 0;

void Setup(){
    EALLOW;
    //LED1
    GpioCtrlRegs.GPAMUX1.bit.GPIO9=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO9=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO9=0;

    //LED2
    GpioCtrlRegs.GPAMUX1.bit.GPIO11=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO11=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11=0;

    //LED3
    GpioCtrlRegs.GPBMUX1.bit.GPIO34=0x00;
    GpioCtrlRegs.GPBDIR.bit.GPIO34=1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34=0;

    //LED4
    GpioCtrlRegs.GPBMUX2.bit.GPIO49=0x00;
    GpioCtrlRegs.GPBDIR.bit.GPIO49=1;
    GpioCtrlRegs.GPBPUD.bit.GPIO49=0;

    //PB1
    GpioCtrlRegs.GPAMUX2.bit.GPIO17=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO17=0; //input
    GpioCtrlRegs.GPAPUD.bit.GPIO17=0;

    //PB2
    GpioCtrlRegs.GPBMUX2.bit.GPIO48=0x00;
    GpioCtrlRegs.GPBDIR.bit.GPIO48=0; //input
    GpioCtrlRegs.GPBPUD.bit.GPIO48=0;

    //ENCODER
    GpioCtrlRegs.GPAMUX1.bit.GPIO12=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO12=0;
    GpioCtrlRegs.GPAPUD.bit.GPIO12=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO13=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO13=0;
    GpioCtrlRegs.GPAPUD.bit.GPIO13=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO14=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO14=0;
    GpioCtrlRegs.GPAPUD.bit.GPIO14=0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO15=0x00;
    GpioCtrlRegs.GPADIR.bit.GPIO15=0;
    GpioCtrlRegs.GPAPUD.bit.GPIO15=0;

    EDIS;
}

int huj(void)
{
    InitSysCtrl();
    Setup();
    DINT;
    GpioDataRegs.GPADAT.bit.GPIO9=0;
    GpioDataRegs.GPADAT.bit.GPIO11=0;
    GpioDataRegs.GPBDAT.bit.GPIO34=0;
    GpioDataRegs.GPBDAT.bit.GPIO49=0;



    while(1)
    {
        DELAY_US(100000);
        pb1 = GpioDataRegs.GPADAT.bit.GPIO17;
        pb2 = GpioDataRegs.GPBDAT.bit.GPIO48;

        vol0 = GpioDataRegs.GPADAT.bit.GPIO12;
        vol1 = GpioDataRegs.GPADAT.bit.GPIO13;
        vol2 = GpioDataRegs.GPADAT.bit.GPIO14;
        vol3 = GpioDataRegs.GPADAT.bit.GPIO15;


        //GpioDataRegs.GPADAT.bit.GPIO9=vol3;
        GpioDataRegs.GPADAT.bit.GPIO11=vol2;
        GpioDataRegs.GPBDAT.bit.GPIO34=vol1;
        GpioDataRegs.GPBDAT.bit.GPIO49=vol0;
        GpioDataRegs.GPADAT.bit.GPIO9=vol3;


        asm(" NOP");
    }


}
