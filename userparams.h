#ifndef USERPARMS_H
#define USERPARMS_H

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


/* Bidirectional functioning */
/* For DEMO purpose, a special definition enables bidirectional functioning */
/* Activating the macro below, a speed reverse will be possible */
/* turning the potentiometer across the median point */
/* In this mode the speed doubling is no longer possible */
#undef BIDIRECTIONAL_SPEED

/* definition for tuning - if active the speed reference is a ramp with a 
 constant slope. The slope is determined by TUNING_DELAY_RAMPUP constant.
 */ 
/* the software ramp implementing the speed increase has a constant slope, */
/* adjusted by the delay TUNING_DELAY_RAMPUP when the speed is incremented.*/
/* The potentiometer speed reference is overwritten. The speed is          */
/* increased from 0 up to the END_SPEED_RPM in open loop – with the speed  */
/* increase typical to open loop, the transition to closed loop is done    */
/* and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
#undef TUNING

/* if TUNING was define, then the ramp speed is needed: */
#ifdef TUNING
    #define TUNING_DELAY_RAMPUP   0xF      /* the smaller the value, the quicker the ramp */
#endif


/* open loop continuous functioning */
/* closed loop transition disabled  */
#undef OPEN_LOOP_FUNCTIONING


/* definition for torque mode - for a separate tuning of the current PI
 controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE



/* static snapshots can be taken activating the SNAPSHOT definition */
/* this feature is available only in DEBUG mode of the project */
/* load DMCI GUI from MPLAB - Tools tab 
Please consult DMCI help by pressing Help button on DMCI window for a
complete description of DMCI tool.
Using dedicated open button in DMCI window load dmci_snapshot.dmci 
setup file from the AN1292 project’s folder.
It contains in the Dynamic data view tab the proper variables setup */
/* static snapshots are captured running the code, and, when halt button
from the Debug toolbar is pressed, the DMCI graph windows are updated with
the captured data */
/* consult debug.c API for calling the specific SNAPSHOT debug functions:
   DBG_Init(void)	- debug initialization
   DBG_SnapStart(void) - start of SNAPSHOT trigger
   DBG_SnapUpdate(void) - SNAPSHOT variables update 
   These functions are called in pmsm.c */
/* 
For captured data setup please check: 
"DEFINITIONS FOR SNAP and RTDM captured variables"
section in this file 
*/
/* to enable static snapshot, define the folowing line */  
#undef SNAPSHOT

/* undefine the line below for RTDM control demo */
//************** Real Time Data Monitor, RTDM *******************

#undef RTDM		// This definition enabled Real Time Data Monitor, UART interrupts
					// to handle RTDM protocol, and array declarations for buffering
					// information in real time

#undef DMCI_DEMO	// Define this if a demo with DMCI is done. Start/stop of motor
					// and speed variation is done with DMCI instead of push button
					// and POT. Undefine "DMCI_DEMO" is user requires operating
					// this application with no DMCI
/* Real Time Debug Monitor enable - if activated, RTDM will allow real time
 control and monitoring of the embedded side from host DMCI interface */
/*
IMPORTANT:
___________________--__________--_____________________________________
Communication between the host computer and the target device (dsPIC33EP)
may fail due to noisy environment, real time specificity of the system 
or other unpredictable causes. 
Since the host computer is used to completely control the target device,
in such failure cases, use the RESET button on MCHV module to stop the 
execution and stop the motor.
___________________--__________--_____________________________________

Short usage guide 
(please consult the http://ww1.microchip.com/downloads/en/DeviceDoc/70567A.pdf
for detailed information on RTDM and its usage):
Open DMCI- Data Monitor and Control Interface window from the 
MPLAB’s Tools tab. Using afferent open pictogram, load dmci_rtdm.dmci
setup file from the AN1292 project’s folder. Within the DMCI window’s
frame, the following controls are available:
1/Boolean DMCIFlags.StartStop – used for start/stop  the motor
2/ Slider SpeedReference – used for reference speed modification
3/ Boolean DMCIFlags.Recorder – used for triggering a new sample of 
measured variables
4/ Slider SnapShotDelay – used for sample rate modification of 
the measured variables
5/ Graph 1-4 – Graphs of the measured variables: A phase current, 
q-axis current, estimated speed, estimated angle 
Running the application note software will connect the target
MCHV board to the host DMCI application as specified in above indicated 
User’s Guide.  
Suggested usage guide: 
1/ Run the program using Run button in Dynamic Data Input DMCI child window
– allow few seconds for communication protocol to be established without
issuing any other command
2/ Run DMCI tab – Remote communication menu, which should indicate 
connection status as DETECTED (if it doesn’t, please refer to the 
above indicated User guide for troubleshooting). Close the DMCI Remote 
Communication Properties window by pressing OK acknowledging the 
detected communication.
3/ Press Run Time Update button in Dynamic Data Input DMCI child window – 
this will update the host DMCI GUI with the initial setup already existing 
on target device
4/ Adjust the speed reference using the afferent Slider SpeedReference 
to a value of roughly 5000 (please note that both positive and negative 
speed references can be set, so bidirectional functioning is selected 
by default with DMCI_DEMO)
5/ Press Start/Stop button, switching it from OFF state to ON state 
using afferent Boolean DMCIFlags.StartStop button in Dynamic Data Input 
DMCI child window –  immediately the motor will start running
6/ Press Boolean DMCIFlags.Recorder in in Dynamic Data Input DMCI child 
window in order to trigger a sample of the measured variables(the button 
should be in ON state)
7/ Press Run Time Update button in Dynamic Data Input DMCI child window 
– this will update the host DMCI GUI with the snapped data in target device.
8/ Use Automated Event Control button to have steps 6/, 7/ executed 
automatically at 1 second period 
9/ Vary the reference speed, vary the snap shot delay, etc.
10/ Press Start/Stop button, switching it from ON state to OFF state 
using afferent Boolean DMCIFlags.StartStop button in Dynamic Data Input 
DMCI child window –  immediately the motor will stop running
*/
/* consult debug.c API for calling the specific RTDM functions:
   DBG_Init(void)	- RTDM initialization
   DBG_SyncComm(void) - RTDM syncrho target and host in pooling mode
   DBG_SnapUpdate(void) - RTDM variables update 
   These functions are called in pmsm.c */
/* 
For captured data setup please check: 
"DEFINITIONS FOR SNAP and RTDM captured variables"
section in this file 
*/

/****************************************************/
/* DEFINITIONS FOR SNAP and RTDM captured variables */
/****************************************************/

    //#include "park.h"
    //#include "estim.h"
    extern tParkParm   ParkParm;       /* park transform params */
    extern tEstimParm 	EstimParm;     /* estim params */
    
    #define DATA_BUFFER_SIZE 50  //Size in 16-bit Words of the snap */
                                  // the value depends on the dsPIC mem
    #define SNAPDELAY	5 // In number of PWM Interrupts
    #define	SNAP1		ParkParm.qIa     // snap captures
    #define	SNAP2		ParkParm.qIq
    #define SNAP3		EstimParm.qVelEstim
    #define SNAP4		EstimParm.qRho

/****************************************************/


#define LOOPTIME_SEC  0.00005           // PWM Period - 50 uSec, 20Khz PWM
#define	DISPLOOPTIME_SEC	0.100		// button polling loop period in sec
#define DEADTIME_SEC  0.000002         // Dead time in seconds - 2us

#define PWM_DT_ERRATA

#define DDEADTIME      (unsigned int)(DEADTIME_SEC*FCY_HZ)	// Dead time in dTcys

#ifdef PWM_DT_ERRATA
    #define MIN_DUTY  (unsigned int)(DDEADTIME/2 + 1)        // Should be >= DDEADTIME/2 for PWM Errata workaround
#else
    #define MIN_DUTY  0x00
#endif


//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
/* the scalling factor for currents are negative because the acquisition for shunts
 is getting reverse sense from LEMs (initialy designed for). */
/* the value of Q15(0.5) represents a 1 multiplication. */

#define KPOT   Q15(0.5)  /* scaling factor for pot */
#define KCURRA Q15(-0.5) /* scaling factor for current phase A */
#define KCURRB Q15(-0.5) /* scaling factor for current phase B */



//**************  support xls file definitions begin **************
/* the following values are given in the xls attached file */

//**************  Motor Parameters **************
/* motor's number of pole pairs */
#define NOPOLESPAIRS 5
/* Nominal speed of the motor in RPM */
#define NOMINAL_SPEED_RPM    2800 // Value in RPM
/* Maximum speed of the motor in RPM - given by the motor's manufacturer */
#define MAXIMUM_SPEED_RPM    5300 // Value in RPM  



#define NORM_CURRENT_CONST     0.000121
/* normalized ls/dt value */
#define NORM_LSDTBASE 1460
/* normalized rs value */
#define NORM_RS  12990
/* the calculation of Rs gives a value exceeding the Q15 range so,
the normalized value is further divided by 2 to fit the 32768 limit */
/* this is taken care in the estim.c where the value is implied */
/* normalized inv kfi at base speed */
#define NORM_INVKFIBASE  7956
/* the calculation of InvKfi gives a value which not exceed the Q15 limit */
/* to assure that an increase of the term with 5 is possible in the lookup table */
/* for high flux weakening the normalized is initially divided by 2 */
/* this is taken care in the estim.c where the value is implied */
/* normalized dt value */
#define NORM_DELTAT  1790

// Limitation constants 
/* di = i(t1)-i(t2) limitation */ 
/* high speed limitation, for dt 50us */
/* the value can be taken from attached xls file */
#define D_ILIMIT_HS 1502
/* low speed limitation, for dt 8*50us */
#define D_ILIMIT_LS 6117

//**************  support xls file definitions end **************


// Filters constants definitions  
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 1200
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 164
/* estimated speed filter constatn */
#define KFILTER_VELESTIM 2*374


/* initial offset added to estimated value, */
/* when transitioning from open loop to closed loop */
/* the value represents 45deg and should satisfy both */
/* open loop and closed loop functioning */
/* normally this value should not be modified, but in */
/* case of fine tuning of the transition, depending on */
/* the load or the rotor moment of inertia */
#define INITOFFSET_TRANS_OPEN_CLSD 0x2000

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* open loop startup constants */
/* the following values depends on the PWM frequency, */
/* lock time is the time needed for motor's poles alignment 
 before the open loop speed ramp up */
#define LOCK_TIME 4000 // This number is: 20,000 is 1 second.
/* open loop speed ramp up end value */
#define END_SPEED_RPM 1000 // Value in RPM
/* open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 10
/* open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.41)

/* in case of the potentimeter speed reference, a reference ramp
 is needed for assuring the motor can follow the reference imposed */
#define    SPEEDREFRAMP   Q15(0.00003)  /*minimum value accepted */

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM           Q15(0.02)
#define     D_CURRCNTR_ITERM           Q15(0.002)
#define     D_CURRCNTR_CTERM           Q15(0.999)
#define     D_CURRCNTR_OUTMAX          0x7FFF

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM           Q15(0.02)
#define     Q_CURRCNTR_ITERM           Q15(0.002)
#define     Q_CURRCNTR_CTERM           Q15(0.999)
#define     Q_CURRCNTR_OUTMAX          0x7FFF

//*** Velocity Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM        Q15(0.5)
#define     SPEEDCNTR_ITERM        Q15(0.005)
#define     SPEEDCNTR_CTERM        Q15(0.999)
#define     SPEEDCNTR_OUTMAX       0x5000


//************** Field Weakening **************

/// Field Weakening constant for constant torque range
#define     IDREF_BASESPEED            NORM_CURRENT(0.0)       // Flux reference value

/*-------------------------------------------------------------*/
/* IMPORTANT:--------------------------------------------------*/
/*-------------------------------------------------------------*/ 
/* In flux weakening of the surface mounted permanent magnets  */
/* PMSMs the mechanical damage of the rotor and the            */
/* demagnetization of the permanent magnets is possible if     */
/* cautions measures are not taken or the motor’s producer     */
/* specifications are not respected.                           */
/*-------------------------------------------------------------*/
/* IMPORTANT:--------------------------------------------------*/
/*-------------------------------------------------------------*/ 
/* In flux weakening regime implementation, if the FOC is lost */
/* at high speed above the nominal value, the possibility of   */
/* damaging the inverter is eminent. The reason is that the    */
/* BEMF will have a greater value than the one that would be   */
/* obtained for the nominal speed exceeding the DC bus voltage */
/* value and though the inverter’s power semiconductors and DC */
/* link capacitors would have to support it. Since the tuning  */
/* proposed implies iterative coefficient corrections until    */
/* the optimum functioning is achieved, the protection of the  */
/* inverter with corresponding circuitry should be assured in  */
/* case of stalling at high speeds.                            */
/*-------------------------------------------------------------*/ 
/*-------------------------------------------------------------*/ 

#define SPEED_INDEX_CONST 10 // speed index is increase

/* the following values indicate the d-current variation with speed */
/* please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)       // up to 2800 RPM
#define	IDREF_SPEED1	NORM_CURRENT(-0.7)   // ~2950 RPM
#define	IDREF_SPEED2	NORM_CURRENT(-0.9)   // ~3110 RPM
#define	IDREF_SPEED3	NORM_CURRENT(-1.0)  // ~3270 RPM
#define	IDREF_SPEED4	NORM_CURRENT(-1.4)  // ~3430 RPM
#define	IDREF_SPEED5	NORM_CURRENT(-1.7)  // ~3600 RPM
#define	IDREF_SPEED6	NORM_CURRENT(-2.0)    // ~3750 RPM
#define	IDREF_SPEED7	NORM_CURRENT(-2.1)    // ~3910 RPM
#define	IDREF_SPEED8	NORM_CURRENT(-2.2)  // ~4070 RPM
#define	IDREF_SPEED9	NORM_CURRENT(-2.25)  // ~4230 RPM
#define	IDREF_SPEED10	NORM_CURRENT(-2.3)   // ~4380 RPM
#define	IDREF_SPEED11	NORM_CURRENT(-2.35)    // ~4550 RPM
#define	IDREF_SPEED12	NORM_CURRENT(-2.4)   // ~4700 RPM
#define	IDREF_SPEED13	NORM_CURRENT(-2.45)    // ~4860 RPM
#define	IDREF_SPEED14	NORM_CURRENT(-2.5)   // ~5020 RPM
#define	IDREF_SPEED15	NORM_CURRENT(-2.5)    // ~5180 RPM
#define	IDREF_SPEED16	NORM_CURRENT(-2.5)  // ~5340 RPM
#define	IDREF_SPEED17	NORM_CURRENT(-2.5)  // ~5500 RPM


/* the following values indicate the invKfi variation with speed */
/* please consult app note for details on tuning */
#define	INVKFI_SPEED0	NORM_INVKFIBASE     // up to 2800 RPM
#define	INVKFI_SPEED1	8674     // ~2950 RPM
#define	INVKFI_SPEED2	9156    // ~3110 RPM
#define	INVKFI_SPEED3	9638    // ~3270 RPM
#define	INVKFI_SPEED4	10120    // ~3430 RPM
#define	INVKFI_SPEED5	10602    // ~3600 RPM
#define	INVKFI_SPEED6	11084    // ~3750 RPM
#define	INVKFI_SPEED7	11566    // ~3910 RPM
#define	INVKFI_SPEED8	12048    // ~4070 RPM
#define	INVKFI_SPEED9	12530    // ~4230 RPM 
#define	INVKFI_SPEED10	13012    // ~4380 RPM
#define	INVKFI_SPEED11	13494    // ~4550 RPM
#define	INVKFI_SPEED12	13976    // ~4700 RPM
#define	INVKFI_SPEED13	14458    // ~4860 RPM
#define	INVKFI_SPEED14	14940    // ~5020 RPM
#define	INVKFI_SPEED15	15422    // ~5180 RPM
#define	INVKFI_SPEED16	15904    // ~5340 RPM
#define	INVKFI_SPEED17	16387    // ~5500 RPM



/* the following values indicate the Ls variation with speed */
/* please consult app note for details on tuning */
#define     LS_OVER2LS0_SPEED0            Q15(0.5)   // up to 2800 RPM
#define     LS_OVER2LS0_SPEED1            Q15(0.45)  // ~2950 RPM
#define     LS_OVER2LS0_SPEED2            Q15(0.4)   // ~3110 RPM
#define     LS_OVER2LS0_SPEED3            Q15(0.4)  // ~3270 RPM
#define     LS_OVER2LS0_SPEED4            Q15(0.35)   // ~3430 RPM
#define     LS_OVER2LS0_SPEED5            Q15(0.35)  // ~3600 RPM
#define     LS_OVER2LS0_SPEED6            Q15(0.34)  // ~3750 RPM
#define     LS_OVER2LS0_SPEED7            Q15(0.34)  // ~3910 RPM
#define     LS_OVER2LS0_SPEED8            Q15(0.33)  // ~4070 RPM
#define     LS_OVER2LS0_SPEED9            Q15(0.33)  // ~4230 RPM
#define     LS_OVER2LS0_SPEED10           Q15(0.32)  // ~4380 RPM
#define     LS_OVER2LS0_SPEED11           Q15(0.32)  // ~4550 RPM
#define     LS_OVER2LS0_SPEED12           Q15(0.31)  // ~4700 RPM
#define     LS_OVER2LS0_SPEED13           Q15(0.30)  // ~4860 RPM
#define     LS_OVER2LS0_SPEED14           Q15(0.29)  // ~5020 RPM
#define     LS_OVER2LS0_SPEED15           Q15(0.28)  // ~5180 RPM
#define     LS_OVER2LS0_SPEED16           Q15(0.27)  // ~5340 RPM
#define     LS_OVER2LS0_SPEED17           Q15(0.26)  // ~5500 RPM

#endif
