
/**
  IC1 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    ic1.c

  @Summary
    This is the generated source file for the IC1 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for driver for IC1.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 3.16
        Device            :  dsPIC33EP128MC504
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.26
        MPLAB 	          :  MPLAB X 3.20
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
/**
  Section: Included Files
*/
#include <xc.h>
#include <p33EP128MC504.h>
#include "ic1.h"
#include "general.h"

/**
  IC Mode.

  @Summary
    Defines the IC Mode.

  @Description
    This data type defines the IC Mode of operation.

*/

//Macros
#define IC1_ManualTriggerSet( void ) (IC1CON2bits.TRIGSTAT = 1)
#define IC1_TriggerStatusClear( void ) (IC1CON2bits.TRIGSTAT = 0)
#define IC1_Calc_Duty( void ) ((unsigned int)( falling_edge_timestamp / rising_edge_timestamp * 100))
#define IC_MEAS_BUF_LEN     2
#define INITIAL_DUTY         0.1

static uint16_t         gIC1Mode;
float mIC_DUTY_CYCKLE = INITIAL_DUTY;
short mFOC_REF = Q15(INITIAL_DUTY);
unsigned int mIC_OFFSET = 0;
uint16_t mIC_MEAS_BUF[IC_MEAS_BUF_LEN];
uint16_t mIC_BUF_COUNTER = 0;
uint16_t temp;
/**
  Section: Driver Interface
*/

void IC1_Initialize (void)
{
    // ICSIDL enabled; ICM Edge Detect Capture; ICTSEL TMR3; ICI Every; 
    IC1CON1 = 0x2001;
    IC1CON1bits.ICTSEL = 0b001;
    // SYNCSEL None; TRIGSTAT disabled; IC32 disabled; ICTRIG Trigger; 
    IC1CON2 = 0x80;
    
    gIC1Mode = IC1CON1bits.ICM;
    
    IFS0bits.IC1IF = false;
    IEC0bits.IC1IE = true;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _IC1Interrupt( void )
{
    if(IFS0bits.IC1IF)
    {
        IFS0bits.IC1IF = 0;
    }
    
    // CHECK IF PWM IS HIGH
    if(PORTBbits.RB7) {
    // GET IC BUFFER VALUES
    while(IC1CON1bits.ICBNE){
        if(mIC_BUF_COUNTER < IC_MEAS_BUF_LEN){
            mIC_MEAS_BUF[mIC_BUF_COUNTER] = (uint16_t)IC1BUF;
        }
        else{
             temp = IC1BUF; // Make sure we read the buffer untill empty
        }
            
        mIC_BUF_COUNTER++;
    }
        
    if(mIC_BUF_COUNTER==IC_MEAS_BUF_LEN && mIC_MEAS_BUF[0] < mIC_MEAS_BUF[1]){
        mIC_DUTY_CYCKLE = ((mIC_MEAS_BUF[0] + mIC_OFFSET ) / (float)(mIC_MEAS_BUF[1]+ mIC_OFFSET));
        mIC_OFFSET = IC1TMR - mIC_MEAS_BUF[1];
        if(mIC_DUTY_CYCKLE>0.02 && mIC_DUTY_CYCKLE<0.98)
        mFOC_REF = Q15(mIC_DUTY_CYCKLE);
    }
    else{
        mIC_OFFSET = 0;
    }

    IC1_TriggerStatusClear();
    __delay32(100);
    IC1_ManualTriggerSet();
    }
    else {
    }
    mIC_BUF_COUNTER = 0;
    
}
void IC1_Start( void )
{
    IC1CON1bits.ICM = gIC1Mode;
    // Trigger capture timer
    IC1_ManualTriggerSet();
}

void IC1_Stop( void )
{
    IC1CON1bits.ICM = 0;
}

uint16_t IC1_CaptureDataRead( void )
{
    return(IC1BUF);
}



bool IC1_TriggerStatusGet( void )
{
    return( IC1CON2bits.TRIGSTAT );
}


bool IC1_HasCaptureBufferOverflowed( void )
{
    return( IC1CON1bits.ICOV );
}


bool IC1_IsCaptureBufferEmpty( void )
{
    return( ! IC1CON1bits.ICBNE );
}

/**
 End of File
*/
