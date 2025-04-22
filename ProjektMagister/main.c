/**
* main.c
*/
#include "DSP28x_Project.h"
__interrupt void button1ISR();
__interrupt void button2ISR();

unsigned int counter=0;
unsigned int counter1=0;
unsigned int vol0 = 0;
unsigned int vol1 = 0;
unsigned int vol2 = 0;
unsigned int vol3 = 0;
unsigned int value_enc = 0;

void setupGPIO(){
EALLOW;

GpioCtrlRegs.GPACTRL.bit.QUALPRD1=0xF0;

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
GpioCtrlRegs.GPAPUD.bit.GPIO17=0x01;
GpioCtrlRegs.GPAMUX2.bit.GPIO17=0;
GpioCtrlRegs.GPADIR.bit.GPIO17=0;

//PB2
GpioCtrlRegs.GPBMUX2.bit.GPIO48=0x01;
GpioCtrlRegs.GPBDIR.bit.GPIO48=0;
GpioCtrlRegs.GPBPUD.bit.GPIO48=0;
GpioCtrlRegs.GPBQSEL2.bit.GPIO48=2;


//PB2 INTERR
GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL=48; // wybiera GPIO17 na zrodlo XINT1
XIntruptRegs.XINT3CR.bit.ENABLE=1;
XIntruptRegs.XINT3CR.bit.POLARITY=0x01;

//ENCODER
GpioCtrlRegs.GPAMUX1.bit.GPIO12=0x00;
GpioCtrlRegs.GPADIR.bit.GPIO12=0;
GpioCtrlRegs.GPAPUD.bit.GPIO12=0;
GpioCtrlRegs.GPAQSEL1.bit.GPIO12=2;

GpioCtrlRegs.GPAMUX1.bit.GPIO13=0x00;
GpioCtrlRegs.GPADIR.bit.GPIO13=0;
GpioCtrlRegs.GPAPUD.bit.GPIO13=0;
GpioCtrlRegs.GPAQSEL1.bit.GPIO13=2;

GpioCtrlRegs.GPAMUX1.bit.GPIO14=0x00;
GpioCtrlRegs.GPADIR.bit.GPIO14=0;
GpioCtrlRegs.GPAPUD.bit.GPIO14=0;
GpioCtrlRegs.GPAQSEL1.bit.GPIO14=2;

GpioCtrlRegs.GPAMUX1.bit.GPIO15=0x00;
GpioCtrlRegs.GPADIR.bit.GPIO15=0;
GpioCtrlRegs.GPAPUD.bit.GPIO15=0;
GpioCtrlRegs.GPAQSEL1.bit.GPIO15=2;

//PB1 INTERR
GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=12; // wybiera GPIO17 na zrodlo XINT1
XIntruptRegs.XINT1CR.bit.ENABLE=1;
XIntruptRegs.XINT1CR.bit.POLARITY=0x03;


EDIS;
}
void setupInterrupts(){
DINT;
InitPieCtrl();
IER = 0x0000;
IFR = 0x0000;
InitPieVectTable();
EALLOW;
PieVectTable.XINT1 = &button1ISR;
PieVectTable.XINT3 = &button2ISR;
EDIS;

PieCtrlRegs.PIECTRL.bit.ENPIE=1; //wlacza modul przerwan

PieCtrlRegs.PIEIER1.bit.INTx4=1; ///Enable interrupt 4 in group1 of interrupts
PieCtrlRegs.PIEIER12.bit.INTx1=1; ///Enable interrupt 1 in group1 of interrupts

IER|=M_INT1; // GROUP 1INTERRUPT ENABLE
IER|=M_INT12;

EINT; // Enable Global interrupt INTM
ERTM; // Enable Global realtime interrupt DBGM
}
int main(void)
{
InitSysCtrl();
setupInterrupts();
setupGPIO();

while(1){
__asm(" NOP");
GpioDataRegs.GPBDAT.bit.GPIO34=0;
GpioDataRegs.GPBDAT.bit.GPIO49=0;

}
}

__interrupt void button1ISR(){
__asm(" NOP"); //wstawka assemblerowska dodaj¹ca jeden cykl procesora robienia nic 1/150mhz
vol3 = GpioDataRegs.GPADAT.bit.GPIO15;
vol0 = GpioDataRegs.GPADAT.bit.GPIO12;
vol1 = GpioDataRegs.GPADAT.bit.GPIO13;
vol2 = GpioDataRegs.GPADAT.bit.GPIO14;

value_enc = vol3*8 + vol2*4 +vol1*2 + vol0;

counter++;
PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // zbicie flagi grupa 1
GpioDataRegs.GPATOGGLE.bit.GPIO9=1;
}

__interrupt void button2ISR(){
__asm(" NOP");
PieCtrlRegs.PIEACK.all = PIEACK_GROUP12; // zbicie flagi grupa 12
counter1++;
GpioDataRegs.GPATOGGLE.bit.GPIO11=1;
}
