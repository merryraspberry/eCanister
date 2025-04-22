
/**
 * main.c
 */

#include "DSP28x_Project.h"


__interrupt void Timer0_ISR();
__interrupt void Timer2_ISR();
__interrupt void Timer1_ISR();

__interrupt void button1ISR();
__interrupt void button2ISR();

unsigned int Timer0_counter=0;
unsigned int Timer2_counter=0;
unsigned int Timer1_counter=0;

unsigned int button1_counter=0;
unsigned int button2_counter=0;
unsigned int h = 0x01;
unsigned int l = 0x2C;
unsigned int h_led2 = 0x00;
unsigned int l_led2 = 0x96;
unsigned int h_led3 = 0x01;
unsigned int l_led3 = 0x2C;
unsigned int freq = 5;
unsigned int freq_calc = 300;

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

    //PB1
    GpioCtrlRegs.GPAPUD.bit.GPIO17=0x01;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17=0;
    GpioCtrlRegs.GPADIR.bit.GPIO17=0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17=2;

    //PB2
    GpioCtrlRegs.GPBMUX2.bit.GPIO48=0x01;
    GpioCtrlRegs.GPBDIR.bit.GPIO48=0;
    GpioCtrlRegs.GPBPUD.bit.GPIO48=0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48=2;

    EDIS;
}

void setupInterrupts(){
    DINT;
    InitPieCtrl();
    InitPieVectTable();

    EALLOW;
    IER=0;
    IFR=0;
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;


    //PB1 INTERR
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=17;
    XIntruptRegs.XINT1CR.bit.ENABLE=1;
    XIntruptRegs.XINT1CR.bit.POLARITY=0x03;

    //PB2 INTERR
    GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL=48;
    XIntruptRegs.XINT3CR.bit.ENABLE=1;
    XIntruptRegs.XINT3CR.bit.POLARITY=0x01;

    PieCtrlRegs.PIEIER1.bit.INTx4=1; ///Enable interrupt 4 in group1 of interrupts
    PieCtrlRegs.PIEIER1.bit.INTx7=1; //Enable interrupt 1 in group1 of interrupts
    PieCtrlRegs.PIEIER12.bit.INTx1=1; //Enable interrupt 1 in group1 of interrupts
    XIntruptRegs.XNMICR.bit.SELECT = 0;

    PieVectTable.TINT0 = &Timer0_ISR;
    PieVectTable.XINT1 = &button1ISR;
    PieVectTable.XINT3 = &button2ISR;
    PieVectTable.TINT2 = &Timer2_ISR;
    PieVectTable.XINT13 = &Timer1_ISR;

    IER|=M_INT1;
    IER|=M_INT12;
    IER|=M_INT14;//timer2
    IER|=M_INT13;


    EINT;
    ERTM;

    EDIS;
}

void setupTimer0(int hi, int lo){
    CpuTimer0Regs.TCR.bit.TSS=0x01;
    CpuTimer0Regs.TPRH.bit.TDDRH=hi;
    CpuTimer0Regs.TPR.bit.TDDR=lo;
    //CpuTimer0Regs.PRD.all=0x00;
    CpuTimer0Regs.TCR.bit.TIE=0x01;
    CpuTimer0Regs.TCR.bit.TRB=0x01;
    CpuTimer0Regs.TCR.bit.TSS=0x0;
}
void setupTimer2(){
    CpuTimer2Regs.TCR.bit.TSS=0x01;
    CpuTimer2Regs.TPRH.bit.TDDRH=h_led2;
    CpuTimer2Regs.TPR.bit.TDDR=l_led2;
    //CpuTimer0Regs.PRD.all=0x00;
    CpuTimer2Regs.TCR.bit.TIE=0x01;
    CpuTimer2Regs.TCR.bit.TRB=0x01;
    CpuTimer2Regs.TCR.bit.TSS=0x0;
}
void setupTimer1(){
    CpuTimer1Regs.TCR.bit.TSS=0x01;
    CpuTimer1Regs.TPRH.bit.TDDRH=h_led3;
    CpuTimer1Regs.TPR.bit.TDDR=l_led3;
    //CpuTimer0Regs.PRD.all=0x00;
    CpuTimer1Regs.TCR.bit.TIE=0x01;
    CpuTimer1Regs.TCR.bit.TRB=0x01;
    CpuTimer1Regs.TCR.bit.TSS=0x0;
}

int main(void)
{
    InitSysCtrl();
    setupGPIO();
    setupTimer0(h,l);
    setupTimer2();
    setupTimer1();
    setupInterrupts();

    while(1){
        __asm(" NOP");
    }
}
__interrupt void Timer0_ISR(){
    Timer0_counter++;
    GpioDataRegs.GPATOGGLE.bit.GPIO9=1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void Timer2_ISR(){
    Timer2_counter++;
    GpioDataRegs.GPATOGGLE.bit.GPIO11=1;
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void Timer1_ISR(){
    Timer2_counter++;
    GpioDataRegs.GPBTOGGLE.bit.GPIO34=1;
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void button1ISR(){
    __asm(" NOP");
    if(freq>=1 && freq<10){
        freq++;
    }

    freq_calc = 1500 / freq;
    h = freq_calc >> 8;
    l = freq_calc & 0x00FF;
    button1_counter++;

    setupTimer0(h,l);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // zbicie flagi grupa 12
}

__interrupt void button2ISR(){
    __asm(" NOP");
    if(freq>1 && freq<=10){
        freq--;
    }

    freq_calc = 1500 / freq;
    h = freq_calc >> 8;
    l = freq_calc & 0x00FF;
    button2_counter++;

    setupTimer0(h,l);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12; // zbicie flagi grupa 12
}
