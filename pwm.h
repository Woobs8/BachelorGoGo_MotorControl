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
extern uint32_t phase;
void PWM_Initialize(uint16_t PWMFreq, uint16_t PWM1DutyCycle, uint16_t PWM2DutyCycle, uint16_t PWM3DutyCycle);
void PWM_Start(void);
void PWM_Stop(void);
void PWM_SetDutyCycles(uint16_t PWM1DutyCycle, uint16_t PWM2DutyCycle, uint16_t PWM3DutyCycle);
uint16_t PWM_CalcPeriodRegisterValue(uint16_t PWMFreq);
uint16_t dutyCycle(uint16_t, uint16_t);

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

