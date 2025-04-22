#include "DSP28x_Project.h"
#include "math.h"
#include "float.h"
#include "PI.h"
#define pi 3.141516

__interrupt void button2ISR();
__interrupt void button1ISR();
__interrupt void adc_isr(void);
__interrupt void tz5_isr();

unsigned int deadband = 125; //125 = 0.83 us
float freq = 20000; //20kHz

//CURRENT CONTROLLER VALUES
float kp_I = 5*1.843545e-3;
float ki_I = 7*1.1173e-1;
//VOLTAGE CONTROLLER VALUES
float kp_U = 1200*1.843545/1000;
float ki_U = 20000*1.1173/10;
//one step time
float Ts= 0.00005;
//VOLTAGE CONTROLLER LIMITS
float sat=32; //50 A max current
//CURRENT CONTROLLER LIMITRS
float Imax = 0.25; //90 deg max phase shift
float Imin = -0.25; //90 deg max phase shift
//Voltage
float Vout = 350;

//ADC Measurements
float ADCresultVout = 0;
float ADCresultVout1 = 0;
float ADCresultVout2 = 0;
float ADCresultVout3 = 0;

float ADCresultIout = 0;
float ADCresultIout1 = 0;
float ADCresultIout2 = 0;
float ADCresultIout3 = 0;

float phase_pu = 0;

float Iscale = 0.075;
float Ioffset = 0.75;
float Uscale = 0.0075;
float Uoffset = 0;

float Vrefplus = 3;
float maxADCval = 4095;

unsigned int counter_button1=0;
unsigned int counter_button2=0;
unsigned int counter_tripzone1=0;
float ADCresult=0;

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
    GpioCtrlRegs.GPADIR.bit.GPIO1=1;

    //EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2=0x01; // PWM3A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO2=1; //pull up reseistor off
    GpioCtrlRegs.GPADIR.bit.GPIO2=1; // pin output direction

    //EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO3=0x01; //epwm3B
    GpioCtrlRegs.GPAPUD.bit.GPIO3=1;
    GpioCtrlRegs.GPADIR.bit.GPIO3=1;

    //EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO4=0x01; // PWM2A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO4=1; //pull up resistor off
    GpioCtrlRegs.GPADIR.bit.GPIO4=1; // pin output direction

    //EPWM3B
    GpioCtrlRegs.GPAMUX1.bit.GPIO5=0x01; //epwm3B
    GpioCtrlRegs.GPAPUD.bit.GPIO5=1;
    GpioCtrlRegs.GPADIR.bit.GPIO5=1;


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
    AdcRegs.ADCMAXCONV.all = 0x04; // 3 adc result needed n+1 = 4
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; //interrupt sequencer1 enable
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x01; //adc input channel ADCINA1 -> VR2 potentiometer
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x02; //adc input channel ADCINA2
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x03; //adc input channel ADCINA3
    AdcRegs.ADCTRL1.bit.ACQ_PS=0xF; // 16 adc cycles
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
    EPwm1Regs.AQCTLA.bit.CAU=0x01;       // disable output A when compare A and counting UP
    EPwm1Regs.AQCTLA.bit.CAD=0x02;       // enable output A when compare A and counting DOWN
    EPwm1Regs.TBPRD= 3750;            // 7500
    EPwm1Regs.CMPA.half.CMPA=1875;         // 1500

    //DEAD - BAND setup
    EPwm1Regs.DBCTL.bit.IN_MODE = 0x00; //EPWMA source for both rising and falling
    EPwm1Regs.DBCTL.bit.POLSEL = 0x02;  //AHC mode, epwmb is inverted
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x03; //deadband fully on
    EPwm1Regs.DBFED = deadband; //375 TBCLK 2.5us
    EPwm1Regs.DBRED = deadband; //375 TBCLK 2.5us

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
    EPwm2Regs.AQCTLA.bit.CAU=0x01;       // disable output A when compare A and counting UP
    EPwm2Regs.AQCTLA.bit.CAD=0x02;       // enable output A when compare A and counting DOWN
    EPwm2Regs.TBPRD=3750;            // 7500
    EPwm2Regs.CMPA.half.CMPA=1875;         // 1500

    //DEAD - BAND setup
    EPwm2Regs.DBCTL.bit.IN_MODE=0x00; //EPWMA source for both rising and falling
    EPwm2Regs.DBCTL.bit.POLSEL=0x02;  //AHC mode, epwmb is inverted
    EPwm2Regs.DBCTL.bit.OUT_MODE=0x03; //deadband fully on
    EPwm2Regs.DBFED=deadband; //375 cycles TBCLK 2.5us
    EPwm2Regs.DBRED=deadband; //375 cycles TBCLK 2.5us

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

    //SYNC PWM2 (MASTER) WITH PWM3 (SLAVE)
    EPwm2Regs.TBCTL.bit.SYNCOSEL=0;

    //SYNC PWM1(MASTER) WITH PWM2(SLAVE)
    EPwm2Regs.TBCTL.bit.PHSEN = 1; //turn on sync
    EPwm2Regs.TBPHS.half.TBPHS = 2502; // 7500/3 +2 = 2502
    EPwm2Regs.TBCTL.bit.PHSDIR = 0; //1 for pwm2 before pwm1, 0 for pwm2 after pwm1

    EDIS;
}

void setupPWM3(){
    EALLOW;
    //PWM CONFIG
    EPwm3Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=0x00;  // 10
    EPwm3Regs.TBCTL.bit.CLKDIV=0x00;     // 32
    EPwm3Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm3Regs.AQCTLA.bit.CAU=0x01;       // disable output A when compare A and counting UP
    EPwm3Regs.AQCTLA.bit.CAD=0x02;       // enable output A when compare A and counting DOWN
    EPwm3Regs.TBPRD=3750;            // 7500
    EPwm3Regs.CMPA.half.CMPA=1875;         // 1500

    //DEAD - BAND setup
    EPwm3Regs.DBCTL.bit.IN_MODE=0x00; //EPWMA source for both rising and falling
    EPwm3Regs.DBCTL.bit.POLSEL=0x02;  //AHC mode, epwmb is inverted
    EPwm3Regs.DBCTL.bit.OUT_MODE=0x03; //deadband fully on
    EPwm3Regs.DBFED=deadband; //375 cycles TBCLK 2.5us
    EPwm3Regs.DBRED=deadband; //375 cycles TBCLK 2.5us

    //ADC
    EPwm3Regs.ETSEL.bit.SOCAEN = 1; // soca enable
    EPwm3Regs.ETSEL.bit.SOCASEL = 0x02; //adc start when TBPRD
    EPwm3Regs.ETPS.bit.SOCACNT = 0x01; // 1 event has occured
    EPwm3Regs.ETPS.bit.SOCAPRD = 0x01; // 1 event has occured
    EPwm3Regs.ETPS.bit.INTCNT = 0x01; // 1 event has occured
    EPwm3Regs.ETPS.bit.INTPRD = 0x01; // 1 event has occured

    //TRIPZONE
    EPwm3Regs.TZSEL.bit.OSHT1 = 1; // tripzone1 ONE SHOT enable
    EPwm3Regs.TZCTL.bit.TZA = 0x02; //pwmA tripzone to low state
    EPwm3Regs.TZCTL.bit.TZB = 0x02; // pwmB tripzone to low state
    EPwm3Regs.TZEINT.bit.OST = 1; //enable cyclebycycle interrupts

    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.INT = 1;

    //SYNC PWM1(MASTER) WITH PWM2(SLAVE)
    EPwm3Regs.TBCTL.bit.PHSEN = 1; //turn on sync
    EPwm3Regs.TBPHS.half.TBPHS =2 ; // 7500/3 +2 = 2502 2 samples for ynchro
    EPwm3Regs.TBCTL.bit.PHSDIR = 1; //1 for pwm2 before pwm1, 0 for pwm2 after pwm1; 2 samples before to synchronize

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
    setupPWM3();
    configureADC();
    setupInterrupts();
    PI_Init(&PI_i_DAB, kp_I, ki_I, Imin, Imax, Ts);
    PI_Init(&PI_u_DAB, kp_U, ki_U, -sat, sat, Ts);

    while(1){
        __asm(" NOP");
    }
      //  DELAY_US(10000);
     //   WhileCounter++;
        //AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;      // uruchamia konwersje ADC z SEQ1

    //}
}

__interrupt void adc_isr(void)
{
    //I SCALE 0.06 OFFSET 1.2
    //U SCALE 0.006 OFFSET 0
    ADCcounter++;
    ADCresult=AdcMirror.ADCRESULT0;
    ADCresultIout=AdcMirror.ADCRESULT1;
    ADCresultVout=AdcMirror.ADCRESULT2;

    ADCresultIout1 = ADCresultIout * Vrefplus/maxADCval;
    ADCresultIout2 = ADCresultIout1 - Ioffset;
    ADCresultIout3 = ADCresultIout2 / Iscale;

    ADCresultVout1 = ADCresultVout * Vrefplus/maxADCval;
    ADCresultVout2 = ADCresultVout1 - Uoffset;
    ADCresultVout3 = ADCresultVout2 / Uscale;


    PI_Calc(&PI_u_DAB, Vout - ADCresultVout3);
    PI_Calc(&PI_i_DAB, PI_u_DAB.y - ADCresultIout3);

    phase_pu = PI_i_DAB.y*4*1875; // PI output in p.u., max phase shift 90deg = 0.25, 1875 = 90deg for 3750 PRD
    EALLOW;
    EPwm2Regs.TBPHS.half.TBPHS =  phase_pu-2; // 2*tbclk delay substracted, pwm2 is after pwm1 so substraction is needed
    EDIS;

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // reset sequencer 1
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

