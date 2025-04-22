/**
 * main.c
 */
#include "DSP28x_Project.h"

__interrupt void adc_isr(void);

unsigned int ADCresult=0;
unsigned int ADCresult1=0;
unsigned int ADCcounter=0;
unsigned int rev_bits = 0;


unsigned int WhileCounter=0;

void configureADC(){
    InitAdc();
    EALLOW;
    SysCtrlRegs.HISPCP.bit.HSPCLK = 0xF;
    AdcRegs.ADCTRL3.bit.ADCCLKPS=0;
    AdcRegs.ADCTRL1.bit.CPS=1;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; // cascaded
    AdcRegs.ADCTRL1.bit.CONT_RUN= 0; //start-stop
    AdcRegs.ADCTRL1.bit.SEQ_OVRD=0; // continous run mode
    AdcRegs.ADCMAXCONV.all = 0x02;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; //interrupt sequencer1 enable

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x00;
    AdcRegs.ADCTRL1.bit.ACQ_PS=0xF;
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
    EDIS;
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;       //wlacza modul przerwan
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // SEQ1 INT interrupt
    IER|=M_INT1;
    EINT;
    ERTM;
}
int main(void)
{
    InitSysCtrl();

    setupInterrupts();
    configureADC();
    while(1){
        DELAY_US(10000);
        WhileCounter++;
        AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;      // uruchamia konwersje ADC z SEQ1

    }
}

__interrupt void adc_isr(void)
{
    ADCcounter++;
    ADCresult=AdcMirror.ADCRESULT0;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;    // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;            // Acknowledge interrupt to PIE
}



