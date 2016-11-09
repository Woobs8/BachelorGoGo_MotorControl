/* 
 * File:   pwm.h
 * Author: THP
 *
 * Created on 4. november 2016, 16:03
 */

#ifndef PWM_H
#define	PWM_H

#include <xc.h>
#include "general.h"
#include <stdint.h>
#include "periph.h"

#define PWM_PRESCALER 1             //Valid prescalers: 1, 2, 4, 8, 16, 32, 64
#define DEAD_TIME_SEC 0.0000004     // 400 ns
#define DEAD_TIME_REGISTER (unsigned int)(DEAD_TIME_SEC*FOSC)
#define PWM_CalcDutyCycleRegisterValue(PWMDutyCycle) ((phase*PWMDutyCycle)>>15)


#ifdef	__cplusplus
extern "C" {
#endif

typedef struct{
    unsigned int PWM1_DUTY;
    unsigned int PWM1_DT;        
    unsigned int PWM2_DUTY;
    unsigned int PWM2_DT;
    unsigned int PWM3_DUTY;
    unsigned int PWM3_DT;
    unsigned int PWM_FREQUENCY;
    unsigned int PWM_FREQUENCY_PHASE_VAL;       
} PWM_PARAMS;

extern PWM_PARAMS mPWM_MARAMS;
    
extern uint32_t phase;
void PWM_Initialize(uint16_t PWMFreq );
void PWM_Start(void);
void PWM_Stop(void);
void PWM_SetDutyCycles(uint16_t PWM1DutyCycle, uint16_t PWM2DutyCycle, uint16_t PWM3DutyCycle);
uint16_t PWM_CalcPeriodRegisterValue(uint16_t PWMFreq);
uint16_t dutyCyclePDC1(uint16_t, uint16_t);
uint16_t dutyCyclePDC2(uint16_t, uint16_t);
uint16_t dutyCyclePDC3(uint16_t, uint16_t);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

