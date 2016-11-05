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

typedef enum {
    DIVIDER_1  = 0b000,
    DIVIDER_2 = 0b001,
    DIVIDER_4 = 0b010,
    DIVIDER_8 = 0b011,
    DIVIDER_16 = 0b100,
    DIVIDER_32 = 0b101,
    DIVIDER_64 = 0b110
}PWM_CLOCK_PRESCALER;

#ifdef	__cplusplus
extern "C" {
#endif

void PWM_Initialize(uint16_t PWMFreq, uint8_t PWM1DutyCycle, uint8_t PWM2DutyCycle, uint8_t PWM3DutyCycle);
void PWM_Start(void);
void PWM_Stop(void);
void PWM_SetDutyCycles(uint16_t PWMFreq, uint8_t PWM1DutyCycle, uint8_t PWM2DutyCycle, uint8_t PWM3DutyCycle);

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

