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
#define DEAD_TIME_SEC 0.00000004     // 400 ns
#define DEAD_TIME_REGISTER (unsigned int)(DEAD_TIME_SEC*FOSC)
#define PWM_CalcDutyCycleRegisterValue(PWMDutyCycle) ((phase*PWMDutyCycle)>>15)

#define PWM_ON(void)  (PTCON |= 0x8000)
#define PWM_OFF(void) (PTCON &= ~(0x8000))
#define PWM_FLT32_FAULT_SRC         (0b1111)
#define PWM_FAULT_ACTIVE_LOW        (0b1)
#define PWM_FAULT_LATCHED_OUTPUT_TO_FLTDAT (0b00)
#define PWM_FAULT_INPUT_DISABLED (0b11)
#define PWMxH_AND_PWMxL_INACTIVER (0b0) 
#define PWM_FAULT_INTERRUPT_ENABLE  1
#define PWM_FAULT_INTERRUPT_DISABLE  0
#define PWM_EDGE_ALIGNED_MODE  0
#define PWM_CENTER_ALIGNED_MODE  1
#define PWM_INDEPENDANT_TIME_BASE_MODE_ENABLE (1)
#define PWM_CONTROLS_PWMxH_PINS 1
#define PWM_CONTROLS_PWMxL_PINS 1
#define PWM_DTRx_IGNORED 0

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
void PWM_Start_Fault();
void PWM_Stop_Fault();
uint16_t PWM_CalcPeriodRegisterValue(uint16_t PWMFreq);
uint16_t dutyCyclePDC1(uint16_t, uint16_t);
uint16_t dutyCyclePDC2(uint16_t, uint16_t);
uint16_t dutyCyclePDC3(uint16_t, uint16_t);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

