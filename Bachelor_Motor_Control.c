/*
 * File:   Bachelor_Motor_Control.c
 * Author: rasmus
 *
 * Created on November 2, 2016, 11:22 AM
 * Rød motor
31mH   @ 120Hz
0,088 ohm
54,4uH @ 1000Hz
0,064 ohm

Calibre
@1000hz
0,9uH
0,017ohm

Ny motor
@ 1000hz
0,071ohm
14,8uH

@ 120Hz
0,062ohm
19mH

 */

#include "xc.h"
#include "userparams.h" /* contains the motor's params & other calibrations */
#include "general.h"   /* includes types definitions used */

#include "periph.h"
#include "drv8305.h"
#include "ic1.h"

#include "readadc.h" /* adc read functions */
#include "meascurr.h" /* current read functions */
#include "svgen.h" /* Space Vector Modulation low level routines */

#include "control.h" /* control structure */

#include "pi.h" /* PI controller definitions */
#include "park.h" /* Park transform definitions */

#include "estim.h" /* estimator definitions */
#include "fdweak.h" /* flux weakening routines */

#include <libq.h> /* q15 sqrt function use */
#include <math.h>




/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    typedef definitions                                     */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
union   {
        struct
            {
            unsigned RunMotor:1;  /* run motor indication */
            unsigned OpenLoop:1;  /* open loop/clsd loop indication */
            unsigned ChangeMode:1; /* mode changed indication - from open to clsd loop */
            unsigned ChangeSpeed:1; /* speed doubled indication */
            unsigned StopFlag:1;
            unsigned    :11;
            }Bit;
        WORD Word;
        } uGF;        // general flags


tPIParm     PIParmQ;        /* parms for PI controlers */
tPIParm     PIParmD;        /* parms for PI controlers */
tPIParm     PIParmQref;     /* parms for PI controlers */

tParkParm   ParkParm;       /* park transform params */
tMeasCurrParm MeasCurrParm; /* current measurement params */

tReadADCParm ReadADCParm;   /* adc values structure */

tSVGenParm SVGenParm;       /* SVM structure */

float testref = 0.7;
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    local variables                                         */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/

unsigned long Startup_Ramp = 0; /* ramp angle variable for initial ramp */
unsigned int Startup_Lock = 0; /* lock variable for initial ramp */       
WORD iDispLoopCnt; /* display loop count */


/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    local definitions                                       */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
/* maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*NOPOLESPAIRS
/* nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*NOPOLESPAIRS



/* end speed conveted to fit the startup ramp */
#define START_UP_WAIT_DEVISION powf(2,START_UP_WAIT_SHIFT)
#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0)*(START_UP_WAIT_DEVISION)
/* end speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM*NOPOLESPAIRS

// Number of control loops that must execute before the button 
// routine is executed.
#define	DISPLOOPTIME	(SPEED_POLL_LOOPTIME_SEC/LOOPTIME_SEC)

/* buttons definitions for the current setup */
#define PINBUTTON1           1//!PORTGbits.RG7
#define PINBUTTON2           1//!PORTGbits.RG6

/* maximum pot value in sfrac mode 0xFFC0 */
/* shifted left with 1 meaning 16368 */ 
#define  MAXPOTVAL_SHL1 16368



/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    local functions prototypes                              */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/

static void InitControlParameters(void);
static void DoControl( void );
static void CalculateParkAngle(void);
int Q15SQRT(int);
#define SNAPSHOT_BUF_SIZE 1700
int32_t pIbBuf[SNAPSHOT_BUF_SIZE] = {0};
int32_t pIaBuf[SNAPSHOT_BUF_SIZE] = {0};

int main(void) {
        /* init the peripherals - contain the definitions for config bits */
    /* configure PLL, GPIO, PWM & ADC */
    
    InitPeriph();
    
    while(1)
        {
        uGF.Word = 0;                   // clear flags
        LEDbits.RED_ON = false;
        /* setup to openloop */
        uGF.Bit.OpenLoop = 1;
        
        /* init PI control parameters */
   	    InitControlParameters();        
        /* init estim parameters */
        InitEstimParm();
   	    /* init flux weakening params */
   	    InitFWParams();
       
        /* Inital offsets for current measurment */
        //InitMeasCompCurr( BufferAdc.Adc1Ch1[0], BufferAdc.Adc1Ch0[0]); 
        
        // zero out i sums 
        PIParmD.qdSum = 0;
        PIParmQ.qdSum = 0;
        PIParmQref.qdSum = 0;

        /* re set peripheral settings if the case */
        /* duty for PWM, ADC interrupt status */
        ResetPeriph();
		
        if(uGF.Bit.RunMotor == 0)
            {	            
            // Initialize current offset compensation
			ResetPeriph();
            
            // Inital offsets
        	//InitMeasCompCurr( BufferAdc.Adc1Ch1[0], BufferAdc.Adc1Ch0[0]);
            InitMeasCompCurr( Q15(0), Q15(0)); 
            uGF.Bit.RunMotor = 1;               //then start motor
            }

        // Run the motor
        uGF.Bit.ChangeMode = 1;
        //DISABLE_GATES();
        ENABLE_GATES();
        PWM_Start();
        
		//Run Motor loop
        while(1)
        {
            readAllStatusRegisters();
            if(mWARNINGS_AND_WATCHDOG_RESET_MAP.FAULT){
                DISABLE_GATES(); // IMMIDIATELY STOP PWM ON MOSFETS!
                uGF.Bit.StopFlag = 1;
                LEDbits.RED_ON = true;

            }
	        /* as long as display loop time is not reached */
            /* skip */
            if(iDispLoopCnt >= DISPLOOPTIME)
            {
				if (uGF.Bit.RunMotor == 0)
                {
				/* exit loop if motor not runned */
					break;
                }

                // PWM servo input starts or stops the motor
                if(uGF.Bit.StopFlag ){
                    /* set the reference speed value to 0 */
                    CtrlParm.qVelRef = 0;
                    /* restart in open loop */
                    uGF.Bit.OpenLoop = 1;
                    // Button just released
                    uGF.Bit.StopFlag  = 0;
                    // Button just released (forced)
                    //uGF.Bit.Btn2Pressed  = 0;
                    // change speed 
                    uGF.Bit.ChangeSpeed =0;
                    // change mode 
                     uGF.Bit.ChangeMode =0;
                    //begin stop sequence
                    uGF.Bit.RunMotor = 0;
                    //reset periph
                    PWM_Stop();
                    ResetPeriph();
                    break;
                }
            }			              
        }   // End of Run Motor loop
        
    } // End of Main loop
        
    // should never get here
    while(1){}
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DoControl                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Executes one PI itteration for each of the three loops Id,Iq,Speed         */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static void DoControl( void )
{
    /* temporary vars for sqrt calculation of q reference */
    int temp_qref_pow_q15;
    
    if( uGF.Bit.OpenLoop )
    {
        // OPENLOOP:  force rotating angle,Vd,Vq
        if( uGF.Bit.ChangeMode )
        {
            // just changed to openloop
            uGF.Bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;

			/* reinit vars for initial speed ramp */
			Startup_Lock = 0;
			Startup_Ramp = 0;
            
        }
        
        /* If Rotor Lock use Q_CURRENT_REF_LOCK */
        if (Startup_Lock < LOCK_TIME){
            CtrlParm.qVelRef = Q_CURRENT_REF_LOCK;
        }
	    /* If Open loop ramp use Q_CURRENT_REF_OPENLOOP */
		else{
            CtrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
        }


        /* q current reference is equal to the vel reference */
        /* while d current reference is equal to 0 */
        /* for maximum startup torque, set the q current to maximum acceptable */
        /* value represents the maximum peak value */
        CtrlParm.qVqRef    = CtrlParm.qVelRef;
       	
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut; 
        //ParkParm.qVq    = CtrlParm.qVqRef; 
        
        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;
        //ParkParm.qVd    = CtrlParm.qVdRef;


    } else
    // Closed Loop Vector Control
	{
        //PIParmD.qKp = PIParmQ.qKp = Q15(0.02);    
        /* if change speed indication, double the speed */
        if(uGF.Bit.ChangeSpeed)
	    {
            ReadADCParm.qAnRef = mFOC_REF;
            if(ReadADCParm.qAnRef / 2 > MAXIMUMSPEED_ELECTR)
                ReadADCParm.qAnRef = MAXIMUMSPEED_ELECTR;	    
	    } 
        else {
		    /* Q15 values are shifted with 2 */
		    ReadADCParm.qAnRef=mFOC_REF;//ReadADCParm.qADValue>>2;		
            // Speed ref max value +-8190
	    }

        // Ramp generator to limit the change of the speed reference
        // the rate of change is defined by CtrlParm.qRefRamp
        
    	CtrlParm.qDiff=CtrlParm.qVelRef - ReadADCParm.qAnRef;		
        // Speed Ref Ramp
		if (CtrlParm.qDiff < 0)
		{
			/* set this cycle reference as the sum of */
			/* previously calculated one plus the reframp value */
			CtrlParm.qVelRef=CtrlParm.qVelRef+CtrlParm.qRefRamp;
		}
		else
		{
			/* same as above for speed decrease */
			CtrlParm.qVelRef=CtrlParm.qVelRef-CtrlParm.qRefRamp;
		}
		/* if difference less than half of ref ramp, set reference */
		/* directly from the pot */
		if (_Q15abs(CtrlParm.qDiff) < (CtrlParm.qRefRamp<<1))
		{
	    	CtrlParm.qVelRef = ReadADCParm.qAnRef;
		}
        
        
        if( uGF.Bit.ChangeMode )
        {
            // just changed from openloop
            uGF.Bit.ChangeMode = 0;
			PIParmQref.qdSum = (long)CtrlParm.qVqRef << 13;
	    }               

/* if TORQUE MODE skip the speed controller */                
#ifndef	TORQUE_MODE
       	// Execute the velocity control loop
		PIParmQref.qInMeas = EstimParm.qVelEstim;
    	PIParmQref.qInRef  = CtrlParm.qVelRef;
    	CalcPI(&PIParmQref);
    	CtrlParm.qVqRef = PIParmQref.qOut;

#else
        CtrlParm.qVqRef = CtrlParm.qVelRef;
#endif
       
        /* Flux weakenign control - the actual speed is replaced */
        /* with the reference speed for stability */
        /* reference for d curent component */
        /* adapt the estimator parameters in concordance with the speed */
        //CtrlParm.qVdRef=FieldWeakening(_Q15abs(CtrlParm.qVelRef));
        
        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;
        
        /* dynamic d-q adjustment */
        /* with d component priority */
        /* vq = sqrt (vs^2 - vd^2) */
        /* limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int)(__builtin_mulss(PIParmD.qOut , PIParmD.qOut)>>15);
        temp_qref_pow_q15 = Q15(0.98) - temp_qref_pow_q15;
        PIParmQ.qOutMax = Q15SQRT(temp_qref_pow_q15);
        
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;
         
    }
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DMA Interrupt                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Does speed calculation and executes the vector update loop    */
/* The ADC sample and conversion is triggered by the PWM period.              */
/* The speed calculation assumes a fixed time interval between calculations.  */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
uint16_t test_var = 0;
void __attribute__ ( ( interrupt, no_auto_psv ) ) _DMA0Interrupt( void )
{
    // Reset interrupt flag
    IFS0bits.DMA0IF = 0;
#undef DEBUG_FOC_ALGORITHM_TIME   
#ifdef DEBUG_FOC_ALGORITHM_TIME
    // pin toggle for debug
    GREEN_LED_ON();
#endif    
            
    
    // Increment count variable that controls execution
    // of display and button functions.
    iDispLoopCnt++;
             
    if( uGF.Bit.RunMotor )
    {
        //  ---------- FOC ALGORITHM -------------
        // The algorithm is implemented with a 
        // Back-Emf observer (Sliding Mode)
        // 1: Firstly Phase currents are measured
        // 2: Secondly the Phase currents are Transfered to Clark And Park
                // Based on latest rotor angular estimate or open loop Angle
        // 3: Observer is calculated estimating the current rotor angle
        // 4: Pi Loops are calculating Inputs to SVPWM
        // 5: If open-loop Override estimated angle with Openloop ramp angle
        // 6: Calculate Inverse Clark and do SVPWM to Motor!
        
	    // Calculate qIa,qIb
        MeasCompCurr();
 
        // Calculate qId,qIq from qSin,qCos,qIa,qIb
        ClarkePark();
		        
        // Speed and field angle estimation
        //****************************            
        Estim();

        // Calculate control values
        DoControl();
        // Calculate qAngle from QEI Module
		CalculateParkAngle();

        /* if open loop */
		if(uGF.Bit.OpenLoop == 1)
		{
            
            pIbBuf[iDispLoopCnt% SNAPSHOT_BUF_SIZE ] = EstimParm.qVelEstim;
            pIaBuf[iDispLoopCnt% SNAPSHOT_BUF_SIZE ] = EstimParm.qRho;
		    /* the angle is given by parkparm */
		    SincosParm.qAngle = ParkParm.qAngle;
		} else  
		{
            test_var++;
            //if(test_var < SNAPSHOT_BUF_SIZE/2){
            pIaBuf[iDispLoopCnt% SNAPSHOT_BUF_SIZE ] = EstimParm.qEsdStateVar;
            pIbBuf[iDispLoopCnt% SNAPSHOT_BUF_SIZE ] = EstimParm.qVelEstimStateVar;
            //}
			/* if closed loop, angle generated by estim */
			SincosParm.qAngle = EstimParm.qRho;
        }

        	
	    // Calculate qSin,qCos from qAngle
        SinCos();
    
        ParkParm.qSin=SincosParm.qSin;
		ParkParm.qCos=SincosParm.qCos;
	
        // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
        InvPark();    

        // Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
        CalcRefVec();

        // Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
        CalcSVGen();

    } 
#ifdef DEBUG_FOC_ALGORITHM_TIME
    // pin toggle for debug
    GREEN_LED_OFF();
#endif
    if(iDispLoopCnt%12 == 0){
        if(LEDbits.GREEN_ON)
            GREEN_LED_ON();
        if(LEDbits.YELLOW_ON)
            YELLOW_LED_ON();
        if(LEDbits.BLUE_ON)
            BLUE_LED_ON();
        if(LEDbits.RED_ON)        
            RED_LED_ON();
    }
    else{
        if(LEDbits.GREEN_ON)
            GREEN_LED_OFF();
            YELLOW_LED_OFF();
            BLUE_LED_OFF();
            RED_LED_OFF();
    }
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: CalculateParkAngle                                          */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Generate the start sinwaves feeding the motor's terminals     */
/* Open loop control, forcing the motor to allign and to start speeding up    */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static void CalculateParkAngle(void)
{
    /* if open loop */
	if(uGF.Bit.OpenLoop)	
	{
		/* begin wiht the lock sequence, for field alligniament */
		if (Startup_Lock < LOCK_TIME){
			Startup_Lock+=1;
        }
	    /* then ramp up till the end speed */
		else if (Startup_Ramp < END_SPEED)
			Startup_Ramp+=OPENLOOP_RAMPSPEED_INCREASERATE;
		else /* switch to closed loop */
		{
#ifndef OPEN_LOOP_FUNCTIONING
            uGF.Bit.ChangeMode = 1;
            uGF.Bit.OpenLoop = 0;
#endif
		}
		/* the angle set depends on startup ramp */
		ParkParm.qAngle += (int)(Startup_Ramp >> START_UP_WAIT_SHIFT);
	}
	else /* switched to closed loop */
	{
   	    /* in closed loop slowly decrease the offset add to */
   	    /* the estimated angle */
   	    if(EstimParm.RhoOffset>0)EstimParm.RhoOffset--; 
	}
	return;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: InitControlParameters                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Init control parameters: PI coefficients, scalling consts etc.             */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static void InitControlParameters(void)
{
    
    // ============= ADC - Measure Current & Pot ======================
    // Scaling constants: Determined by calibration or hardware design.
    //ReadADCParm.qK      = KPOT;    
    MeasCurrParm.qKa    = K_CURRA;    
    MeasCurrParm.qKb    = K_CURRB;   
    
    CtrlParm.qRefRamp = SPEEDREFRAMP;
    
    // ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPTIME_TCY;
    
    // ============= PI D Term ===============      
    PIParmD.qKp = D_CURRCNTR_PTERM;       
    PIParmD.qKi = D_CURRCNTR_ITERM;              
    PIParmD.qKc = D_CURRCNTR_CTERM;       
    PIParmD.qOutMax = D_CURRCNTR_OUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

    // ============= PI Q Term ===============
    PIParmQ.qKp = Q_CURRCNTR_PTERM;    
    PIParmQ.qKi = Q_CURRCNTR_ITERM;
    PIParmQ.qKc = Q_CURRCNTR_CTERM;
    PIParmQ.qOutMax = Q_CURRCNTR_OUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

    // ============= PI Qref Term ===============
    PIParmQref.qKp = SPEEDCNTR_PTERM;       
    PIParmQref.qKi = SPEEDCNTR_ITERM;       
    PIParmQref.qKc = SPEEDCNTR_CTERM;       
    PIParmQref.qOutMax = SPEEDCNTR_OUTMAX;   
    PIParmQref.qOutMin = -PIParmQref.qOutMax;
    
    InitPI(&PIParmQref);
	return;
}


