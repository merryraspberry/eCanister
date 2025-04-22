
/**
 * main.c
 */
#include "DSP28x_Project.h"

unsigned long int RX_counter=0;
unsigned char RX_char;
unsigned int ADCcounter=0;
double ADCresult=0;
double ADCresult1=0;
double ADCresult2=0;


__interrupt void SCI_ISR();
__interrupt void adc_isr(void);

void setupInterrupts(){
    DINT;
    InitPieCtrl();
    InitPieVectTable();
    EALLOW;
    IER=0;
    IFR=0;
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // SEQ1 INT interrupt
    PieCtrlRegs.PIEIER9.bit.INTx1=1;
    PieVectTable.SCIRXINTA=&SCI_ISR; // sciA Recived Interrupt
    PieVectTable.SEQ1INT = &adc_isr; //adc

    IER|=M_INT1;
    IER|=M_INT9;

    EINT;
    ERTM;
    EDIS;
}

void setupGpioUart(){
    EALLOW;

    //EPWM5B
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0x01; // PWM1A SET
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1; //pull up reseistor off
    GpioCtrlRegs.GPADIR.bit.GPIO0=1; // pin output direction

    GpioCtrlRegs.GPAMUX2.bit.GPIO28=1; //SCIA RX
    GpioCtrlRegs.GPAPUD.bit.GPIO28=0; // pullup resistor on
    GpioCtrlRegs.GPADIR.bit.GPIO28=0; // pin input

    GpioCtrlRegs.GPAMUX2.bit.GPIO29=1; //SCIA TX
    GpioCtrlRegs.GPAPUD.bit.GPIO29=0; //Pullup resistor on
    GpioCtrlRegs.GPADIR.bit.GPIO29=1; //Input
    EDIS;
}

void setupUART(){
    EALLOW;

    SciaRegs.SCICTL1.bit.SWRESET=0;     // enable SCI reset condition
    SciaRegs.SCIFFTX.bit.SCIRST=0;      // enable tx reset condition
    SciaRegs.SCIFFRX.bit.RXFIFORESET=0; // enable rx reset condition

    // data frame config
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
    SysCtrlRegs.LOSPCP.bit.LSPCLK=0;     // LSPCLK divider = 1
    SciaRegs.SCIHBAUD=0x3D;             //975,5625 for 19200, set 976=0x3D0
    SciaRegs.SCILBAUD=0x00;
    SciaRegs.SCICCR.bit.STOPBITS=0;      //one stop bit
    SciaRegs.SCICCR.bit.PARITY=0;        //odd parity
    SciaRegs.SCICCR.bit.PARITYENA=1;     //parity enable
    SciaRegs.SCICCR.bit.LOOPBKENA=1;    //loopback test mode enable
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE=0; //idle line
    SciaRegs.SCICCR.bit.SCICHAR=7;      // 1 1 1 bits en, 8bit character
    // end of data frame config


    SciaRegs.SCICTL1.bit.TXENA=1;       //enable transmiter
    SciaRegs.SCICTL1.bit.RXENA=1;       //enable receiver


    SciaRegs.SCICTL2.bit.RXBKINTENA=1;  //enable RX interrupt (optional)

    SciaRegs.SCICTL1.bit.SWRESET=1;     //disable SCI reset condition 1 for
    SciaRegs.SCIFFTX.bit.SCIRST=1;      //disable tx reset condition
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1; //disable rx reset condition


    EDIS;
}
void setupPWM1(){
    EALLOW;
    //PWM CONFIG
    EPwm1Regs.TBCTL.bit.CTRMODE=2;    // set up-down mode
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x00;  // 1
    EPwm1Regs.TBCTL.bit.CLKDIV=0x00;     // 1 75mhz
    EPwm1Regs.TBCTL.bit.FREE_SOFT=0x02;  // do not stop when cpu is halted
    EPwm1Regs.AQCTLA.bit.CAU=0x02;       // disable output A when compare A and counting UP
    EPwm1Regs.AQCTLA.bit.CAD=0x01;       // enable output A when compare A and counting DOWN
    EPwm1Regs.TBPRD = 20;            // 7500
    EPwm1Regs.CMPA.half.CMPA=0;         // 0% DUTY for start

    //ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; // soca enable
    EPwm1Regs.ETSEL.bit.SOCASEL = 0x02; //adc start when TBPRD
    EPwm1Regs.ETPS.bit.SOCACNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.SOCAPRD = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTCNT = 0x01; // 1 event has occured
    EPwm1Regs.ETPS.bit.INTPRD = 0x01; // 1 event has occured

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

int main(void)
{
    InitSysCtrl();
    setupGpioUart();
    setupPWM1();
    configureADC();
    setupUART();
    setupInterrupts();
    while(1){
        __asm(" NOP");
    }
}

__interrupt void adc_isr(void)
{
    ADCcounter++;
    ADCresult=AdcMirror.ADCRESULT0;
    ADCresult1 = ADCresult*10;
    ADCresult2 = ADCresult1/4095;

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // Clear INT SEQ1 bit ADCCST FLAG
    SciaRegs.SCITXBUF= ADCresult2;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;            // Acknowledge interrupt to PIE
}

__interrupt void SCI_ISR(){
    RX_counter++;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP9;;
    RX_char=SciaRegs.SCIRXBUF.bit.RXDT; //read received character
    EALLOW;
    //EPwm1Regs.CMPA.half.CMPA=RX_char;
    EPwm1Regs.CMPA.half.CMPA=10;
    EDIS;
}
