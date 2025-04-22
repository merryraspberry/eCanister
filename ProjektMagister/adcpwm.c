/*
 * adcpwm.c
 *
 *  Created on: 29 cze 2024
 *      Author: nexus
 */


#include "DSP28x_Project.h"
__interrupt void adc_isr(void);

unsigned int ADCresult=0;
unsigned int ADCcounter=0;

void setupGPIO(){
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0x01; // PWM1A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1; //pull up reseistor off
    GpioCtrlRegs.GPADIR.bit.GPIO0=1; // pin output direction

    GpioCtrlRegs.GPBMUX2.bit.GPIO61=0x00; // general GPIO SET
    GpioCtrlRegs.GPBPUD.bit.GPIO61=1; //pull up reseistor off
    GpioCtrlRegs.GPBDIR.bit.GPIO61=1; // pin output direction


    EDIS;
}

void configureADC(){
    InitAdc();
    EALLOW;
    SysCtrlRegs.HISPCP.bit.HSPCLK = 0x06; // 12 divide -> 150Mhz/12 = 12,5Mhz
    AdcRegs.ADCTRL3.bit.ADCCLKPS=0x0; // not used when cps=0
    AdcRegs.ADCTRL1.bit.CPS=0; //f/2, not used now
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; // cascaded
    AdcRegs.ADCTRL1.bit.CONT_RUN= 0; //start-stop
    AdcRegs.ADCTRL1.bit.SEQ_OVRD=0; // continous run mode
    AdcRegs.ADCMAXCONV.all = 0x02; // one adc result needed n+1 = 2
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; //interrupt sequencer1 enable

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x01; //adc input channel ADCINA1 -> VR2 potentiometer
    AdcRegs.ADCTRL1.bit.ACQ_PS=0xE; // 15 adc cycles

    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=0x01; // adc start from pwm

    EDIS;
}

void setupPWM(){
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x05;  // 10
    EPwm1Regs.TBCTL.bit.CLKDIV=0x00;     // 32
    EPwm1Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm1Regs.AQCTLA.bit.CAU=0x02;       // disable output A when compare A and counting UP
    EPwm1Regs.AQCTLA.bit.CAD=0x01;       // enable output A when compare A and counting DOWN
    EPwm1Regs.TBPRD=0x2EE;            // 750
    EPwm1Regs.CMPA.half.CMPA=0x20D;         // 0.2*750 = 525

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 0x02; //adc start when TBPRD
    EPwm1Regs.ETPS.bit.SOCACNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.SOCAPRD = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTCNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTPRD = 0x01; // 1 event has occured
    EDIS;
}

void setupInterrupts(){
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;
    PieVectTable.SEQ1INT = &adc_isr;
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;       //wlacza modul przerwan
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // SEQ1 INT interrupt
    EDIS;
    IER|=M_INT1;
    EINT;
    ERTM;
}



int main(void)
{
    InitSysCtrl();
    setupGPIO();
    setupPWM();
    configureADC();
    setupInterrupts();

    //while(1){
      //  DELAY_US(10000);
     //   WhileCounter++;
        //AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;      // uruchamia konwersje ADC z SEQ1

    //}
}

__interrupt void adc_isr(void)
{
    ADCcounter++;
    ADCresult=AdcMirror.ADCRESULT0;
    GpioDataRegs.GPBDAT.bit.GPIO61=1; //high state for adc synchro
    DELAY_US(1);
    GpioDataRegs.GPBDAT.bit.GPIO61=0; //low state for adc synchro
    //AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // reset sequencer 1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // Clear INT SEQ1 bit ADCCST FLAG
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;            // Acknowledge interrupt to PIE
}


