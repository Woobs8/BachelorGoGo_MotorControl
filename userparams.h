#ifndef USERPARMS_H
#define USERPARMS_H
#include <math.h>
/* Bidirectional functioning */
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
/* increased from 0 up to the END_SPEED_RPM in open loop ? with the speed  */
/* increase typical to open loop, the transition to closed loop is done    */
/* and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
#undef TUNING

/* if TUNING was define, then the ramp speed is needed: */
#ifdef TUNING
    #define TUNING_DELAY_RAMPUP   0xFF      /* the smaller the value, the quicker the ramp */
#endif


/* open loop continuous functioning */
/* closed loop transition disabled  */
//#define OPEN_LOOP_FUNCTIONING


/* definition for torque mode - for a separate tuning of the current PI
 controllers, tuning mode will disable the speed PI controller */
//#define TORQUE_MODE 1




#define LOOPTIME_SEC  0.00005           // PWM Period - 50 uSec, 20Khz PWM
#define	SPEED_POLL_LOOPTIME_SEC	10   // button polling loop period in sec
#define DEADTIME_SEC  0.0000004          // Dead time in seconds - 2us

#undef PWM_DT_ERRATA

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

#define K_BUSVOLTAGE   Q15(0.5)  /* scaling factor for pot */
#define K_CURRA Q15(-0.5) /* scaling factor for current phase A */
#define K_CURRB Q15(-0.5) /* scaling factor for current phase B */
#define K_CURRC Q15(-0.5) /* scaling factor for current phase B */


//**************  Motor Parameters **************
/* motor's number of pole pairs */
#define NOPOLESPAIRS 3
/* Nominal speed of the motor in RPM */
#define RPM_SCALE           1
#define NOMINAL_SPEED_RPM_NON_SCALED 11000
#define NOMINAL_SPEED_RPM    NOMINAL_SPEED_RPM_NON_SCALED/(powf(2,RPM_SCALE)) 
/* Maximum speed of the motor in RPM - given by the motor's manufacturer */
#define MAXIMUM_SPEED_RPM_NON_SCALED 21000
#define MAXIMUM_SPEED_RPM   MAXIMUM_SPEED_RPM_NON_SCALED/(powf(2,RPM_SCALE))



#define NORM_CURRENT_CONST     0.002442
/* normalized ls/dt value */
#define LS_SCALE      3
#define NORM_LSDTBASE_NON_SCALED        24411
#define NORM_LSDTBASE                   (NORM_LSDTBASE_NON_SCALED/(powf(2,LS_SCALE)))
/* normalized rs value */
#define RS_SCALE 0
#define NORM_RS_NON_SCALED 10680
#define NORM_RS  NORM_RS_NON_SCALED/(powf(2,RS_SCALE))
/* the calculation of Rs gives a value exceeding the Q15 range so,
the normalized value is further divided by 2 to fit the 32768 limit */
/* this is taken care in the estim.c where the value is implied */  

/* normalized inv kfi at base speed */
#define NORM_INVKFIBASE_SCALE 3
#define NORM_INVKFIBASE_NON_SCALED 81164
#define NORM_INVKFIBASE  (NORM_INVKFIBASE_NON_SCALED/(powf(2,NORM_INVKFIBASE_SCALE)))
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
#define D_ILIMIT_HS 3277
/* low speed limitation, for dt 8*50us */
#define D_ILIMIT_LS 13107

// Filters constants definitions  
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 1200
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 164

/* estimated speed filter constant */
#define KFILTER_VELESTIM 2*374


/* initial offset added to estimated value, */
/* when transitioning from open loop to closed loop */
/* the value represents 45deg and should satisfy both */
/* open loop and closed loop functioning */
/* normally this value should not be modified, but in */
/* case of fine tuning of the transition, depending on */
/* the load or the rotor moment of inertia */
#define INITOFFSET_TRANS_OPEN_CLSD 0x2000 //10000

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* open loop startup constants */
/* the following values depends on the PWM frequency, */
/* lock time is the time needed for motor's poles alignment 
 before the open loop speed ramp up */

#define LOCK_TIME 5000 // This number is: 20,000 is 1 second.
/* open loop speed ramp up end value */
#define END_SPEED_RPM 1500 // Value in RPM
/* open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 2
/* open loop q current setup - */
#define START_UP_WAIT_SHIFT 8
#define Q_CURRENT_REF_LOCK NORM_CURRENT(8)
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(6)

/* in case of the potentimeter speed reference, a reference ramp
 is needed for assuring the motor can follow the reference imposed */
#define    SPEEDREFRAMP   Q15(0.00003)  /*minimum value accepted */

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM           Q15(0.01)
#define     D_CURRCNTR_ITERM           Q15(0.005)
#define     D_CURRCNTR_CTERM           Q15(0.999)
#define     D_CURRCNTR_OUTMAX          0x7FFF

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM           Q15(0.01)
#define     Q_CURRCNTR_ITERM           Q15(0.005)
#define     Q_CURRCNTR_CTERM           Q15(0.999)
#define     Q_CURRCNTR_OUTMAX          0x7FFF

//*** Velocity Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM        Q15(0.7)
#define     SPEEDCNTR_ITERM        Q15(0.03)
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
/* cautions measures are not taken or the motor?s producer     */
/* specifications are not respected.                           */
/*-------------------------------------------------------------*/
/* IMPORTANT:--------------------------------------------------*/
/*-------------------------------------------------------------*/ 
/* In flux weakening regime implementation, if the FOC is lost */
/* at high speed above the nominal value, the possibility of   */
/* damaging the inverter is eminent. The reason is that the    */
/* BEMF will have a greater value than the one that would be   */
/* obtained for the nominal speed exceeding the DC bus voltage */
/* value and though the inverter?s power semiconductors and DC */
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
#define	IDREF_SPEED1	IDREF_SPEED0//NORM_CURRENT(-0.7)   // ~2950 RPM
#define	IDREF_SPEED2	IDREF_SPEED0//NORM_CURRENT(-0.9)   // ~3110 RPM
#define	IDREF_SPEED3	IDREF_SPEED0//NORM_CURRENT(-1.0)  // ~3270 RPM
#define	IDREF_SPEED4	IDREF_SPEED0//NORM_CURRENT(-1.4)  // ~3430 RPM
#define	IDREF_SPEED5	IDREF_SPEED0//NORM_CURRENT(-1.7)  // ~3600 RPM
#define	IDREF_SPEED6	IDREF_SPEED0//NORM_CURRENT(-2.0)    // ~3750 RPM
#define	IDREF_SPEED7	IDREF_SPEED0//NORM_CURRENT(-2.1)    // ~3910 RPM
#define	IDREF_SPEED8	IDREF_SPEED0//NORM_CURRENT(-2.2)  // ~4070 RPM
#define	IDREF_SPEED9	IDREF_SPEED0//NORM_CURRENT(-2.25)  // ~4230 RPM
#define	IDREF_SPEED10	IDREF_SPEED0//NORM_CURRENT(-2.3)   // ~4380 RPM
#define	IDREF_SPEED11	IDREF_SPEED0//NORM_CURRENT(-2.35)    // ~4550 RPM
#define	IDREF_SPEED12	IDREF_SPEED0//NORM_CURRENT(-2.4)   // ~4700 RPM
#define	IDREF_SPEED13	IDREF_SPEED0//NORM_CURRENT(-2.45)    // ~4860 RPM
#define	IDREF_SPEED14	IDREF_SPEED0//NORM_CURRENT(-2.5)   // ~5020 RPM
#define	IDREF_SPEED15	IDREF_SPEED0//NORM_CURRENT(-2.5)    // ~5180 RPM
#define	IDREF_SPEED16	IDREF_SPEED0//NORM_CURRENT(-2.5)  // ~5340 RPM
#define	IDREF_SPEED17	IDREF_SPEED0//NORM_CURRENT(-2.5)  // ~5500 RPM


/* the following values indicate the invKfi variation with speed */
/* please consult app note for details on tuning */
    #define	INVKFI_SPEED0	NORM_INVKFIBASE     // up to 2800 RPM
#define	INVKFI_SPEED1	NORM_INVKFIBASE//8674     // ~2950 RPM
#define	INVKFI_SPEED2	NORM_INVKFIBASE    // ~3110 RPM
#define	INVKFI_SPEED3	NORM_INVKFIBASE    // ~3270 RPM
#define	INVKFI_SPEED4	NORM_INVKFIBASE    // ~3430 RPM
#define	INVKFI_SPEED5	NORM_INVKFIBASE    // ~3600 RPM
#define	INVKFI_SPEED6	NORM_INVKFIBASE    // ~3750 RPM
#define	INVKFI_SPEED7	NORM_INVKFIBASE    // ~3910 RPM
#define	INVKFI_SPEED8	NORM_INVKFIBASE    // ~4070 RPM
#define	INVKFI_SPEED9	NORM_INVKFIBASE    // ~4230 RPM 
#define	INVKFI_SPEED10	NORM_INVKFIBASE    // ~4380 RPM
#define	INVKFI_SPEED11	NORM_INVKFIBASE    // ~4550 RPM
#define	INVKFI_SPEED12	NORM_INVKFIBASE    // ~4700 RPM
#define	INVKFI_SPEED13	NORM_INVKFIBASE    // ~4860 RPM
#define	INVKFI_SPEED14	NORM_INVKFIBASE    // ~5020 RPM
#define	INVKFI_SPEED15	NORM_INVKFIBASE    // ~5180 RPM
#define	INVKFI_SPEED16	NORM_INVKFIBASE    // ~5340 RPM
#define	INVKFI_SPEED17	NORM_INVKFIBASE    // ~5500 RPM



/* the following values indicate the Ls variation with speed */
/* please consult app note for details on tuning */
#define     LS_OVER2LS0_SPEED0            Q15(0.5)   // up to 2800 RPM
#define     LS_OVER2LS0_SPEED1            LS_OVER2LS0_SPEED0//Q15(0.45)  // ~2950 RPM
#define     LS_OVER2LS0_SPEED2            LS_OVER2LS0_SPEED0//Q15(0.4)   // ~3110 RPM
#define     LS_OVER2LS0_SPEED3            LS_OVER2LS0_SPEED0//Q15(0.4)  // ~3270 RPM
#define     LS_OVER2LS0_SPEED4            LS_OVER2LS0_SPEED0//Q15(0.35)   // ~3430 RPM
#define     LS_OVER2LS0_SPEED5            LS_OVER2LS0_SPEED0//Q15(0.35)  // ~3600 RPM
#define     LS_OVER2LS0_SPEED6            LS_OVER2LS0_SPEED0//Q15(0.34)  // ~3750 RPM
#define     LS_OVER2LS0_SPEED7            LS_OVER2LS0_SPEED0//Q15(0.34)  // ~3910 RPM
#define     LS_OVER2LS0_SPEED8            LS_OVER2LS0_SPEED0//Q15(0.33)  // ~4070 RPM
#define     LS_OVER2LS0_SPEED9            LS_OVER2LS0_SPEED0//Q15(0.33)  // ~4230 RPM
#define     LS_OVER2LS0_SPEED10           LS_OVER2LS0_SPEED0//Q15(0.32)  // ~4380 RPM
#define     LS_OVER2LS0_SPEED11           LS_OVER2LS0_SPEED0//Q15(0.32)  // ~4550 RPM
#define     LS_OVER2LS0_SPEED12           LS_OVER2LS0_SPEED0//Q15(0.31)  // ~4700 RPM
#define     LS_OVER2LS0_SPEED13           LS_OVER2LS0_SPEED0//Q15(0.30)  // ~4860 RPM
#define     LS_OVER2LS0_SPEED14           LS_OVER2LS0_SPEED0//Q15(0.29)  // ~5020 RPM
#define     LS_OVER2LS0_SPEED15           LS_OVER2LS0_SPEED0//Q15(0.28)  // ~5180 RPM
#define     LS_OVER2LS0_SPEED16           LS_OVER2LS0_SPEED0//Q15(0.27)  // ~5340 RPM
#define     LS_OVER2LS0_SPEED17           LS_OVER2LS0_SPEED0//Q15(0.26)  // ~5500 RPM

#endif