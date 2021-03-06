#ifndef GENERAL_H
#define GENERAL_H

/**********************************************************************
* � 2012 Microchip Technology Inc.
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


//--------------------- Common stuff for C routines ---------------------


typedef unsigned short WORD;
typedef signed int SFRAC16;
typedef unsigned char  BYTE;
//typedef unsigned char  bool;
#define False  0
#define True   1

#ifdef INITIALIZE
    // allocate storage
    #define EXTERN
#else
    #define EXTERN extern
#endif

#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))

typedef union   {
        struct
            {
            unsigned RunMotor:1;  /* run motor indication */
            unsigned OpenLoop:1;  /* open loop/clsd loop indication */
            unsigned ChangeMode:1; /* mode changed indication - from open to clsd loop */
            unsigned ChangeSpeed:1; /* speed doubled indication */
            unsigned StopFlag:1;
            unsigned ForceOpenLoop:1;
            unsigned    :10;
            }Bit;
        WORD Word;
        } GENERAL_FLAGS;        // general flags
        
extern GENERAL_FLAGS uGF;
        
#endif      // end of general_H
