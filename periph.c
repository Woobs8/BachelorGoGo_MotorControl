#include "periph.h"

void DDelay(void);
/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/
/*
_FPOR(ALTI2C1_OFF & ALTI2C2_OFF);
_FWDT(PLLKEN_ON & FWDTEN_OFF);
_FOSCSEL(FNOSC_FRC & IESO_OFF & PWMLOCK_OFF);
_FGS(GWRP_OFF & GCP_OFF);

_FICD(ICS_PGD2 & JTAGEN_OFF);	// PGD3 for 28pin 	PGD2 for 44pin
//_FOSC(FCKSM_CSECMD & POSCMD_XT);		//XT W/PLL
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE); // FRC W/PLL
 * */

// FICD
#pragma config ICS = PGD2    // ICD Communication Channel Select bits->Communicate on PGEC2 and PGED2
#pragma config JTAGEN = OFF    // JTAG Enable bit->JTAG is disabled

// FPOR
#pragma config ALTI2C1 = OFF    // Alternate I2C1 pins->I2C1 mapped to SDA1/SCL1 pins
#pragma config ALTI2C2 = OFF    // Alternate I2C2 pins->I2C2 mapped to SDA2/SCL2 pins
#pragma config WDTWIN = WIN25    // Watchdog Window Select bits->WDT Window is 25% of WDT period

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config WDTPRE = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config PLLKEN = ON    // PLL Lock Enable bit->Clock switch to PLL source will wait until the PLL lock signal is valid.
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config FWDTEN = OFF    // Watchdog Timer Enable bit->Watchdog timer enabled/disabled by user software

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFNC = OFF    // OSC2 Pin Function bit->OSC2 is clock output
#pragma config IOL1WAY = ON    // Peripheral pin select configuration->Allow only one reconfiguration
#pragma config FCKSM = CSECMD    // Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are disabled

// FOSCSEL
#pragma config FNOSC = FRCDIVN    // Oscillator Source Selection->Internal Fast RC (FRC) Oscillator with postscaler
#pragma config PWMLOCK = ON    // PWM Lock Enable bit->Certain PWM registers may only be written after key sequence
#pragma config IESO = OFF    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source
// FGS
#pragma config GWRP = OFF    // General Segment Write-Protect bit->General Segment may be written
#pragma config GCP = OFF    // General Segment Code-Protect bit->General Segment Code protect is Disabled


/********************Setting Configuration Bits End********************************/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: InitPeriph                                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Init the peripherals: PLL, ADC, PWM & GPIO                    */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void InitPeriph(void)
{
  //The settings below set up the oscillator and PLL for x MIPS as
    //follows:
    //                  Fin  * M
    // Fosc       =     ---------
    //                   N2 * N1   
    //
    // Fin   	  = 7.37 MHz
    // Fosc       = x MHz
    // Fcy        = Fosc/2
    // 70 MIPS                   (7.37 * M)/(N2*N1)
    // 70 MIPS (70.015 Mhz) (i.e (7.37 * 76 )/(2*2))

	/****************** Clock definitions *********************************/

	PLLFBD = 74;                // M    =   76 
	CLKDIVbits.PLLPOST = 0;		// N1   =   2
	CLKDIVbits.PLLPRE = 0;		// N2   =   2
    // Initiate Clock Switch to FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    // Wait for Clock to lock at 70MPIS
    while(OSCCONbits.COSC != 0b001);
    while(OSCCONbits.LOCK != 1);

    // Turn saturation on to insure that overflows will be handled smoothly.
    CORCONbits.SATA  = 0;

	// ============= Port A ==============
	LATA  = 0x0000;
	TRISA = 0x079F;             // Setup Port C input/outputs reset=079F
    TRISAbits.TRISA10 = 1;      // RA10 Input Unconnected
    TRISAbits.TRISA9  = 1;      // RA9 Input SPI SDI
    TRISAbits.TRISA8  = 0;      // RA8 Output Red Led
    TRISAbits.TRISA7  = 1;      // RA7 Unconnected
                                // 
    TRISAbits.TRISA4  = 0;      // RA4 Output SPI SDO
    TRISAbits.TRISA3  = 1;      // RA3 Input Ex OSC
    TRISAbits.TRISA2  = 1;      // RA2 Input Ex OSC
    TRISAbits.TRISA1  = 1;      // RA1 Input AN1
    TRISAbits.TRISA0  = 1;      // RA0 Input AN0
    
    RPINR7bits.IC1R = 0x0027;   //RB7->IC1:IC1;
    
    ANSELA = 0;
    ANSELAbits.ANSA4  = 1;      // RA4 Spi - Analog not specified can be set! 
    ANSELAbits.ANSA1  = 1;      // RA1 (AN1) Temperature
	ANSELAbits.ANSA0  = 1;      // RA0 (AN0) Bus Voltage
     
	
	// ============= Port B ==============
    LATB  = 0x0000;
    TRISB = 0xFFFF;             // Setup Port B input/outputs reset=FFFF
    TRISBbits.TRISB15 = 0;      // RB10~15 outputs for PWM, rest are inputs
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB9  = 0;      // RB9 Output Blue Diode
    TRISBbits.TRISB8  = 0;      // RB8 Output Green Diode
    TRISBbits.TRISB7  = 1;      // RB7 Input SpeedRef Pwm 
    TRISBbits.TRISB6  = 1;      // RB5~6 Inputs Programming
    TRISBbits.TRISB5  = 1;
    TRISBbits.TRISB4  = 1;      // RB4 Fault Input
    TRISBbits.TRISB3  = 1;      // RB1~3 Inputs is AN3~5
    TRISBbits.TRISB2  = 1;
    TRISBbits.TRISB1  = 1;
    TRISBbits.TRISB0  = 0;      // RB0 Output Spi Slave Select
    ANSELB = 0;
    ANSELBbits.ANSB1  = 1;      // RB1 (AN3) Phase 0 Current
    ANSELBbits.ANSB2  = 1;      // RB2 (AN4) Phase 1 Current
    ANSELBbits.ANSB3  = 1;      // RB3 (AN5) Phase 2 Current
    
	// ============= Port C ==============
	/*LATC  = 0x0000;
	TRISC = 0x03FF;             // Setup Port C input/outputs Reset = 03FF
    TRISCbits.TRISC9  = 0;      // RC9 Output Enable Gates
    TRISCbits.TRISC8  = 0;      // RC8 Output Wake Gate Driver
    TRISCbits.TRISC7  = 1;      // RC7 Input Power good
    TRISCbits.TRISC6  = 0;      // RC6 Output Yellow Led
    TRISCbits.TRISC5  = 0;      // RC5 Output I2C SCL
    TRISCbits.TRISC4  = 0;      // RC4 Output I2C SDA
    TRISCbits.TRISC3  = 0;      // RC3 Output SPI SCK
    TRISCbits.TRISC2  = 1;      // RC2~0 Inputs AN8~6
    TRISCbits.TRISC1  = 1;
    TRISCbits.TRISC0  = 1;
    
    ANSELC = 0;
    ANSELCbits.ANSC2 = 1;	// RC2 (AN8) Phase Volt 2
    ANSELCbits.ANSC1 = 1;	// RC1 (AN7) Phase Volt 1
    ANSELCbits.ANSC0 = 1;	// RC0 (AN6) Phase Volt 0
     */
    
		
	/******************* Spi 1 Initialize ***************************/
	/* SPI 1 is used to setup the Gate driver DRV8305*/
    SPI1_Initialize();
    
    /****************************************************************/
    /*
    // Initialise OPAMP/Comparator 
    // OpAmp 1,2,3 for Signal Conditioning Currents & Comparator 4 for Fault Generation

                                // C1IN1+ :IBUS+ ; C1IN1- : IBUS- 
    CM1CON = 0x8C00;			// OpAmp1- OA1OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C1IN1- pin
                                // C2IN1+ :IA+ ; C2IN1- : IBUS+ 
    CM2CON = 0x8C00;			// OpAmp2- OA2OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C2IN1- pin
                                // C3IN1+ :IB+ ; C3IN1- : IBUS+ 
    CM3CON = 0x8C00;			// OpAmp3- OA3OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C3IN1- pin
    
    CM4CON = 0x8011;            // cmp output non inverted
                                // VIN+ input connects to internal CVREFIN voltage
                                // VIN- input connects to CMP1 (source Ibus) 
 
    CVRCON = 0x008F;            // CVREF = (0.1031)*CVR + 0.825, CVR = 15
    CM4FLTR = 0x000F;           // max filtering
    */
   // ============= Motor PWM ======================

    // Center aligned PWM. 20kHz
    //PWM_Initialize(20000);

    // ============= Motor Speed Reference Measurement ======================
    TMR2_Initialize(); //  Timer will be a clock to IC1
    IC1_Initialize();  //  IC1 Will measure an input PWM where
                       //  20% pwm is 0% torque -> 80% pwm is 100% torque
    
    // ============= Global Interrupts ======================
    INTERRUPT_GlobalEnable();
}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DDelay                                                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Delay Routine											      */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//void DDelay(void)
//{
	//long i;
	//for (i = 0;i < 100000;i++);
	//return;
//}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: ResetPeriph                                                 */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Reset the peripherals during runtime/reset configurations     */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*
 * void ResetPeriph(void)
{

    // clear and Enable ADC interrupt 
    IFS0bits.AD1IF = 0; 
    IEC0bits.AD1IE = 1;

    // Initialise duty regs // 
    //PDC1 = MIN_DUTY;
    //PDC2 = MIN_DUTY;
    //PDC3 = MIN_DUTY;
    return;

}
*/
