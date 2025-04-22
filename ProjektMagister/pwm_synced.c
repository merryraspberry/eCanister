#include "DSP28x_Project.h"


__interrupt void button2ISR();
__interrupt void button1ISR();
__interrupt void adc_isr(void);
__interrupt void tz5_isr();

unsigned int counter_button1=0;
unsigned int counter_button2=0;
unsigned int counter_tripzone1=0;
unsigned int ADCresult=0;
unsigned int ADCcounter=0;
unsigned int syncinfo=0;

void setupGPIO(){
    EALLOW;

    //EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0x01; // PWM1A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1; //pull up reseistor off
    GpioCtrlRegs.GPADIR.bit.GPIO0=1; // pin output direction

    //EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO1=0x01; //epwm1B
    GpioCtrlRegs.GPAPUD.bit.GPIO1=1;
    GpioCtrlRegs.GPADIR.bit.GPIO1=0;

    //EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2=0x01; // PWM1A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO2=1; //pull up reseistor off
    GpioCtrlRegs.GPADIR.bit.GPIO2=1; // pin output direction

    //EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO3=0x01; //epwm1B
    GpioCtrlRegs.GPAPUD.bit.GPIO3=1;
    GpioCtrlRegs.GPADIR.bit.GPIO3=0;

    //SYNC PIN PWM1
    GpioCtrlRegs.GPBMUX2.bit.GPIO61=0x00; // general GPIO SET
    GpioCtrlRegs.GPBPUD.bit.GPIO61=1; //pull up reseistor off
    GpioCtrlRegs.GPBDIR.bit.GPIO61=1; // pin output direction

    //SYNC PIN PWM2
    GpioCtrlRegs.GPBMUX2.bit.GPIO60=0x00; // general GPIO SET
    GpioCtrlRegs.GPBPUD.bit.GPIO60=1; //pull up reseistor off
    GpioCtrlRegs.GPBDIR.bit.GPIO60=1; // pin output direction

    //TRIP ZONE 1
   /*GpioDataRegs.GPADAT.bit.GPIO12=1; //TZ5 REVERSE LOGIC
    GpioCtrlRegs.GPAMUX1.bit.GPIO12=0x01; // TZ5 SET PIN 16
    GpioCtrlRegs.GPAPUD.bit.GPIO12=0; //pull up reseistor on
    GpioCtrlRegs.GPADIR.bit.GPIO12=0;*/ // pin INput direction

    //PB1
    GpioCtrlRegs.GPAMUX2.bit.GPIO17=0; //GPIO SELECTED
    GpioCtrlRegs.GPADIR.bit.GPIO17=0; //INPUT
    GpioCtrlRegs.GPAPUD.bit.GPIO17=0; //PULLUP ON

    //PB2
    GpioCtrlRegs.GPBMUX2.bit.GPIO48=0x01;
    GpioCtrlRegs.GPBDIR.bit.GPIO48=0; //input
    GpioCtrlRegs.GPBPUD.bit.GPIO48=0; //pullup on

    //PB2 INTERR
    GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL=48; // wybiera GPIO48 na zrodlo XINT3
    XIntruptRegs.XINT3CR.bit.ENABLE=1;
    XIntruptRegs.XINT3CR.bit.POLARITY=0x01;

    //PB1 INTERR
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=17; // wybiera GPIO17 na zrodlo XINT1
    XIntruptRegs.XINT1CR.bit.ENABLE=1;
    XIntruptRegs.XINT1CR.bit.POLARITY=0x03;

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

void setupPWM1(){
    EALLOW;
    //PWM CONFIG
    EPwm1Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x00;  // 1
    EPwm1Regs.TBCTL.bit.CLKDIV=0x00;     // 1
    EPwm1Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm1Regs.AQCTLA.bit.CAU=0x02;       // disable output A when compare A and counting UP
    EPwm1Regs.AQCTLA.bit.CAD=0x01;       // enable output A when compare A and counting DOWN
    EPwm1Regs.TBPRD=0x1D4C;            // 7500
    EPwm1Regs.CMPA.half.CMPA=0x5DC;         // 1500

    //DEAD - BAND setup
    EPwm1Regs.DBCTL.bit.IN_MODE = 0x00; //EPWMA source for both rising and falling
    EPwm1Regs.DBCTL.bit.POLSEL = 0x02;  //AHC mode, epwmb is inverted
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x03; //deadband fully on
    EPwm1Regs.DBFED = 750; //75 cycles TBCLK 5us
    EPwm1Regs.DBRED = 750; //75 cycles TBCLK 5us

    //ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; // soca enable
    EPwm1Regs.ETSEL.bit.SOCASEL = 0x02; //adc start when TBPRD
    EPwm1Regs.ETPS.bit.SOCACNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.SOCAPRD = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTCNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTPRD = 0x01; // 1 event has occured

    //TRIPZONE
    EPwm1Regs.TZSEL.bit.OSHT1 = 1; // tripzone1 ONE SHOT enable
    EPwm1Regs.TZCTL.bit.TZA = 0x02; //pwmA tripzone to low state
    EPwm1Regs.TZCTL.bit.TZB = 0x02; // pwmB tripzone to low state
    EPwm1Regs.TZEINT.bit.OST = 1; //enable cyclebycycle interrupts
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;

    //SYNC PWM1(MASTER) WITH PWM2(SLAVE)
    EPwm1Regs.TBCTL.bit.SYNCOSEL=1;



    EDIS;
}

void setupPWM2(){
    EALLOW;
    //PWM CONFIG
    EPwm2Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm2Regs.TBCTL.bit.HSPCLKDIV=0x00;  // 10
    EPwm2Regs.TBCTL.bit.CLKDIV=0x00;     // 32
    EPwm2Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm2Regs.AQCTLA.bit.CAU=0x02;       // disable output A when compare A and counting UP
    EPwm2Regs.AQCTLA.bit.CAD=0x01;       // enable output A when compare A and counting DOWN
    EPwm2Regs.TBPRD=0x1D4C;            // 7500
    EPwm2Regs.CMPA.half.CMPA=0x5DC;         // 1500

    //DEAD - BAND setup
    EPwm2Regs.DBCTL.bit.IN_MODE=0x00; //EPWMA source for both rising and falling
    EPwm2Regs.DBCTL.bit.POLSEL=0x02;  //AHC mode, epwmb is inverted
    EPwm2Regs.DBCTL.bit.OUT_MODE=0x03; //deadband fully on
    EPwm2Regs.DBFED=750; //750 cycles TBCLK 5us
    EPwm2Regs.DBRED=750; //750 cycles TBCLK 5us

    //ADC
    EPwm2Regs.ETSEL.bit.SOCAEN = 1; // soca enable
    EPwm2Regs.ETSEL.bit.SOCASEL = 0x02; //adc start when TBPRD
    EPwm2Regs.ETPS.bit.SOCACNT = 0x01; // 1 event has occured
    EPwm2Regs.ETPS.bit.SOCAPRD = 0x01; // 1 event has occured
    EPwm2Regs.ETPS.bit.INTCNT = 0x01; // 1 event has occured
    EPwm2Regs.ETPS.bit.INTPRD = 0x01; // 1 event has occured

    //TRIPZONE
    EPwm2Regs.TZSEL.bit.OSHT1 = 1; // tripzone1 ONE SHOT enable
    EPwm2Regs.TZCTL.bit.TZA = 0x02; //pwmA tripzone to low state
    EPwm2Regs.TZCTL.bit.TZB = 0x02; // pwmB tripzone to low state
    EPwm2Regs.TZEINT.bit.OST = 1; //enable cyclebycycle interrupts

    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;

    //SYNC PWM1(MASTER) WITH PWM2(SLAVE)
    EPwm2Regs.TBCTL.bit.PHSEN = 1; //turn on sync
    EPwm2Regs.TBPHS.half.TBPHS =  1000; // 7500/2 +2 = 3752
    EPwm2Regs.TBCTL.bit.PHSDIR = 0; //1 for pwm2 before pwm1, 0 for pwm2 after pwm1

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
    PieVectTable.SEQ1INT = &adc_isr;
    PieVectTable.EPWM1_TZINT = &tz5_isr;

    PieCtrlRegs.PIECTRL.bit.ENPIE=1;       //wlacza modul przerwan
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // SEQ1 INT interrupt
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1; // PWM1 TZ INT
    PieCtrlRegs.PIEIER1.bit.INTx4=1; ///Enable interrupt 4 in group1 of interrupts PB1
    PieCtrlRegs.PIEIER12.bit.INTx1=1; ///Enable interrupt 1 in group1 of interrupts PB2




    IER|=M_INT1; //FOR SEQ1INT
    IER|=M_INT2; //FOR PWM1TZINT
    IER|=M_INT12; //FOR PB2 XINT3
    EDIS;
    EINT;
    ERTM;
}



int main(void)
{
    InitSysCtrl();
    setupGPIO();
    setupPWM1();
    setupPWM2();
    configureADC();
    setupInterrupts();

    while(1){
        __asm(" NOP");
        syncinfo = EPwm2Regs.TBSTS.bit.SYNCI;
    }
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

__interrupt void button1ISR(){
    __asm(" NOP");
    //GpioDataRegs.GPADAT.bit.GPIO12=1; //TRIPZONE1 OFF
    counter_button1++;
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST= 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // zbicie flagi grupa 1
}

__interrupt void button2ISR(){
    __asm(" NOP");
    EALLOW;
    //GpioDataRegs.GPADAT.bit.GPIO12=0; //TRIPZONE1 ON
    EPwm1Regs.TZFRC.bit.OST = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12; // zbicie flagi grupa 12

    EDIS;

    counter_button2++;

}
__interrupt void tz5_isr(){
    __asm(" NOP");

    counter_tripzone1++;

/*    EALLOW;
    EPwm1Regs.TZCLR.bit.OST= 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;*/

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // zbicie flagi grupa 12

}

