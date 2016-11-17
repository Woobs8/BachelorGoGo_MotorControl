#include "adc.h"


typedef struct
{
	uint8_t intSample;
}

ADC_OBJECT;

static ADC_OBJECT adc1_obj;

void ADC_Initialize( void ){
    // ============= ADC - Measure Current & Pot ======================
    // ADC setup for simultanous sampling on 
    //      CH0=AN4, CH1=AN3, CH2=AN0, CH3=AN6. 
    // Sampling triggered by Timer3 and stored in signed fractional form.
    // fractional (DOUT = dddd dddd dd00 0000) ~ is same as Signed fractional (DOUT = sddd dddd dd00 0000) becuase AVDD/2 is offset by Gate driver
    AD1CON1bits.FORM = 3;    
	// Timer3 overflow ends sampling and starts conversion
	AD1CON1bits.SSRC = ADC1_SAMPLING_SOURCE_PWM;
	AD1CON1bits.SSRCG = 0;
    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
    AD1CON1bits.SIMSAM = 1;  
    // Sampling begins immediately after last conversion completes. 
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1; 
    
    AD1CON1bits.ADDMABM = 0;

    AD1CON2 = 0;
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    AD1CON2bits.CHPS = 3;  
    AD1CON2bits.SMPI = 1;


    AD1CON3 = 0;
    // A/D Conversion Clock Select bits = 6 * Tcy
    AD1CON3bits.ADCS = 3;  
    
    
    AD1CON4 = 0;
    AD1CON4bits.ADDMAEN = 1;
    AD1CON4bits.DMABL = 0;


     /* ADCHS: ADC Input Channel Select Register */
    AD1CHS0 = 0;
    // CH0 is AN4
    AD1CHS0bits.CH0SA = 4;

   // CH1 positive input is AN3, CH2 positive input is AN0, CH3 positive input is AN6
    AD1CHS123bits.CH123SA = 1;
  
 /* ADCSSL: ADC Input Scan Select Register */
    AD1CSSL = 0;

    // Turn on A/D module
    AD1CON1bits.ADON = 1;
    
    adc1_obj.intSample = AD1CON2bits.SMPI;
   __delay32(1000);

}
