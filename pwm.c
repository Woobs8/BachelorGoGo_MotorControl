#include "pwm.h"
PWM_PARAMS mPWM_MARAMS;
/*
 * Params:
 *      PWMFreq = PWM frequency in Hz
 *      PWM1DutyCycle = Duty Cycle in % for PWM 1
 *      PWM2DutyCycle = Duty Cycle in % for PWM 2
 *      PWM3DutyCycle = Duty Cycle in % for PWM 3
 * 
 * Initializes the High-Speed PWM Module to output three complimentary, 
 * center-aligned PWM signals.
 * 
 * The PWM signals are configured with independent duty cycles and periods. 
 * The PWM signals have the same period, but different duty cycles, as 
 * specified by the params.
 */
void PWM_Initialize(uint16_t PWMFreq)
{
    /* Calcuate register value for PWM time period*/
    uint16_t period_register = mPWM_MARAMS.PWM_FREQUENCY = PWM_CalcPeriodRegisterValue(PWMFreq);

    /* Set PWM Periods on PHASEx Registers */
    PHASE1 = PHASE2 = PHASE3 = mPWM_MARAMS.PWM_FREQUENCY_PHASE_VAL = period_register;

    /* Set duty cycle register values*/
    PDC1 = mPWM_MARAMS.PWM1_DUTY = 0x0000;
    PDC2 = mPWM_MARAMS.PWM2_DUTY = 0x0000;
    PDC3 = mPWM_MARAMS.PWM3_DUTY = 0x0000;
    
    /* Set Dead Time Values */
    /* DTRx Registers are ignored in center-aligned mode */
    DTR1 = DTR2 = DTR3 = 0;                             /* PWMH dead time */ 
    ALTDTR1 = ALTDTR2 = ALTDTR3 = DEAD_TIME_REGISTER;   /* PWML dead time */
    mPWM_MARAMS.PWM1_DT = ALTDTR1;
    mPWM_MARAMS.PWM2_DT = ALTDTR2;
    mPWM_MARAMS.PWM3_DT = ALTDTR3;

    /* Set PWM Mode to Complementary */
    IOCON1 = IOCON2 = IOCON3 = 0xC000;

    /* Set Independent Time Bases, Center-Aligned mode and
    Independent Duty Cycles */
    PWMCON1 = PWMCON2 = PWMCON3 = 0x0204;

    /* Configure Faults */
    FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
    
    /* PWM Clock Prescaler */
    PTCON2 = 0x0000;
}

/* Enables the output of all configured PWM signals on the High-Speed PWM Module */
void PWM_Start(void) 
{
    /* Enable PWM Module */
    PTCON = 0x8000;
}

/* Disables the output of all configured PWM signals on the High-Speed PWM Module */
void PWM_Stop(void) 
{
    /* Disable PWM Module */
    PTCON = 0x0000; 
}

/*
 * Params:
 *      PWM1DutyCycle = Duty Cycle in % for PWM 1
 *      PWM2DutyCycle = Duty Cycle in % for PWM 2
 *      PWM3DutyCycle = Duty Cycle in % for PWM 3
 * 
 * Configures the duty cycles for the PWM signals.
 */
void PWM_SetDutyCycles(uint16_t PWM1DutyCycle, uint16_t PWM2DutyCycle, uint16_t PWM3DutyCycle) 
{
    dutyCyclePDC1(mPWM_MARAMS.PWM_FREQUENCY_PHASE_VAL,PWM1DutyCycle);
    dutyCyclePDC2(mPWM_MARAMS.PWM_FREQUENCY_PHASE_VAL,PWM2DutyCycle);
    dutyCyclePDC3(mPWM_MARAMS.PWM_FREQUENCY_PHASE_VAL,PWM3DutyCycle);
}

/* Calcuate register value for PWM time period*/
uint16_t PWM_CalcPeriodRegisterValue(uint16_t PWMFreq)
{
    /*
     * High-Speed PWM Module Datasheet p. 72:
     *                       F_osc
     * Phase =  --------------------------------
     *           F_pwm * PWM_clock_prescaler * 2
     */
    return FOSC/(PWMFreq*PWM_PRESCALER*2);
}
