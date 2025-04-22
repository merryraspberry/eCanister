#include "DSP28x_Project.h"
#include "math.h"
#include "float.h"
#include "PI.h"
#include "lowpass.h"
#include "res.h"
#include "bandstop.h"
#define pi 3.141516

__interrupt void button2ISR();
__interrupt void button1ISR();
__interrupt void adc_isr(void);
__interrupt void tz5_isr();

unsigned int deadband = 125; //125 = 0.83 us
float freq = 20000; //20kHz

//CURRENT CONTROLLER VALUES
float kp_I = 0.8*2;
float ki_I = 20*20;

//VOLTAGE CONTROLLER VALUES
float kp_U = 0.25*0.02*30;
float ki_U = 0.02*200;

// PLL PHASE CONTROLLER
float kp_PLL = 0.01*5;
float ki_PLL = 0.1/2;

//one step time
float Ts= 0.00005;
//VOLTAGE CONTROLLER LIMITS
float sat=1000;
//CURRENT CONTROLLER LIMITRS
float Imax = 100;
float Imin = -100;
//Voltage
float Vout = 400;
//PLL PI CONTROLLER LIMITS
float pllmax = 5000;
float pllmin = -5000;
//RESETABLE INTEGRATOR LIMITS

float integrator_max = 2*pi;
float integrator_min = -2*pi;

//LOWPASS FILTER
float lowpass_gain = 1;
float lowpass_damping = 0.707;
float lowpass_omega = 15*2*pi;
///BANDSTOP FILTER
float bandstop_band = 60*2*pi;
float bandstop_omega = 100*2*pi;
//PLL VARIABLES
float pll_product_in = 0;
float pll_sum = 0;
float pll_sin = 0;
float pll_cos = 0;
float pll_product_out = 0;
float pll_sum_out = 0;
float pll_sum_out_inverted = 0;
float pll_normalized = 0;
float pll_normalized_inverted = 0;
float grid_omega = 2*50*pi;

//ADC Measurements
float ADCresultVout = 0;
float ADCresultVout1 = 0;
float ADCresultVout2 = 0;
float ADCresultVout3 = 0;

float ADCresultVin = 0;
float ADCresultVin1 = 0;
float ADCresultVin2 = 0;
float ADCresultVin3 = 0;

float ADCresultIout = 0;
float ADCresultIout1 = 0;
float ADCresultIout2 = 0;
float ADCresultIout3 = 0;

float  pulse_width= 0;
float  pulse_width_inverted= 0;

//-120;120
float Iscale = 0.0125;
float Ioffset = 1.5;
// 0-500V
float Uscale = 0.006;
float Uoffset = 0;
//-400-400V
float Uinscale = 0.00375;
float Uinoffset = 1.5;

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
    AdcRegs.ADCMAXCONV.all = 0x05; // 4 adc result needed n+1 = 5
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; //interrupt sequencer1 enable
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x01; //adc input channel ADCINA1 -> VR2 potentiometer
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x02; //adc input channel ADCINA2
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x03; //adc input channel ADCINA3
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x04; //adc input channel ADCINA4
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
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x00; //deadband fully OFF
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
    EPwm2Regs.DBCTL.bit.OUT_MODE=0x00; //deadband fully OFF
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

    IER|=M_INT1; //FOR SEQ1INT
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
    PI_Init(&PI_i_PLL, kp_I, ki_I, Imin, Imax, Ts);
    PI_Init(&PI_u_PLL, kp_U, ki_U, -sat, sat, Ts);
    PI_Init(&PI_PLL, kp_PLL, ki_PLL, pllmin, pllmax, Ts);
    lowpass_Init(&LOW_1, lowpass_gain, lowpass_damping, lowpass_omega, Ts);
    band_Init(&BAND_1, bandstop_band, bandstop_omega, Ts);
    R_Init(&R_1, integrator_min, integrator_max, Ts);

    while(1){
        __asm(" NOP");
    }
}

__interrupt void adc_isr(void)
{
    //I SCALE 0.06 OFFSET 1.2
    //U SCALE 0.006 OFFSET 0
    ADCcounter++;
    ADCresult=AdcMirror.ADCRESULT0;
    ADCresultIout=AdcMirror.ADCRESULT1;
    ADCresultVout=AdcMirror.ADCRESULT2;
    ADCresultVin=AdcMirror.ADCRESULT3;

    ADCresultIout1 = ADCresultIout * Vrefplus/maxADCval;
    ADCresultIout2 = ADCresultIout1 - Ioffset;
    ADCresultIout3 = ADCresultIout2 / Iscale;

    ADCresultVout1 = ADCresultVout * Vrefplus/maxADCval;
    ADCresultVout2 = ADCresultVout1 - Uoffset;
    ADCresultVout3 = ADCresultVout2 / Uscale;

    ADCresultVin1 = ADCresultVin * Vrefplus/maxADCval;
    ADCresultVin2 = ADCresultVin1 - Uinoffset;
    ADCresultVin3 = ADCresultVin2 / Uinscale;



    pll_sum = grid_omega + PI_PLL.y; //insertion of data for calculations
    R_Calc(&R_1, pll_sum);
    pll_cos = cos(R_1.y);
    pll_sin = sin(R_1.y);
    pll_product_in = ADCresultVin3*pll_cos;
    lowpass_Calc(&LOW_1, pll_product_in);
    PI_Calc(&PI_PLL, LOW_1.y);


    //band_Calc(&BAND_1, ADCresultVout3);
    //PI_Calc(&PI_u_PLL, Vout - BAND_1.y);

    PI_Calc(&PI_u_PLL, Vout - ADCresultVout3);
    pll_product_out = pll_sin * -1 * PI_u_PLL.y;
    PI_Calc(&PI_i_PLL, pll_product_out - ADCresultIout3);
    pll_sum_out = PI_i_PLL.y + ADCresultVin3;

    //bipolar modulator used
    pll_normalized = ((pll_sum_out/Vout)+1)/2; //normalized to signal in range 0 to 1
    pulse_width = pll_normalized*3750; //according to max pwm counter 3750 set now
    pulse_width_inverted = (1-pll_normalized)*3750; //according to max pwm counter 3750 set now
    EALLOW;
    EPwm1Regs.CMPA.half.CMPA=pulse_width;
    EPwm2Regs.CMPA.half.CMPA=pulse_width_inverted;
    EDIS;

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // reset sequencer 1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // Clear INT SEQ1 bit ADCCST FLAG
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;            // Acknowledge interrupt to PIE
}



