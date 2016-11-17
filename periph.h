/**********************************************************************
* © 2012 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
******************************************************************************/
#ifndef PERIPH_H
#define PERIPH_H

#include <stdbool.h>
#include "general.h"   /* includes types definitions used */
#include "userparams.h"

#define FCY_HZ                      35000000            // Instruction cycle frequency (Hz)
#define FCY                         35000000UL
#define FOSC                        2*FCY
#define TCY_SEC                     (1.0/FCY_HZ)          // Instruction cycle period (sec)

#define LOOPTIME_TCY                (unsigned int)(LOOPTIME_SEC/TCY_SEC)   // Basic loop period in units of Tcy

#define RED_LED_ON()              (LATAbits.LATA8 = 1)
#define RED_LED_TOGGLE()          (LATAbits.LATA8 ^= 1)
#define RED_LED_OFF()             (LATAbits.LATA8 = 0)

#define YELLOW_LED_TOGGLE()         (LATBbits.LATB9 ^= 1)
#define YELLOW_LED_OFF()            (LATBbits.LATB9 = 0)
#define YELLOW_LED_ON()             (LATBbits.LATB9 = 1)

#define GREEN_LED_TOGGLE()           (LATCbits.LATC6 ^= 1)
#define GREEN_LED_OFF()              (LATCbits.LATC6 = 0)
#define GREEN_LED_ON()               (LATCbits.LATC6 = 1)

#define BLUE_LED_TOGGLE()            (LATBbits.LATB8 ^= 1)
#define BLUE_LED_OFF()               (LATBbits.LATB8 = 0)
#define BLUE_LED_ON()                (LATBbits.LATB8 = 1)

#define ENABLE_GATES()              (LATCbits.LATC9 = 1)
#define DISABLE_GATES()             (LATCbits.LATC9 = 0)

#include "spi.h"   /* includes types definitions used */
#include "adc.h"
#include "interrupt_manager.h"
#include "ic1.h"
#include "tmr2.h"
#include "pwm.h"
#include "dma.h"
#include "tmr2.h"
#include "i2c.h"
#include "drv8305.h"

typedef struct{
    bool GREEN_ON;
    bool YELLOW_ON;
    bool BLUE_ON;
    bool RED_ON;
}LEDs;

extern volatile LEDs  LEDbits;

void InitPeriph(void);
void ResetPeriph(void);

#endif // PERIPH_H
