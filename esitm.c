/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    include files                                           */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
#include "general.h"
#include "estim.h"

#define DECIMATE_NOMINAL_SPEED NOMINAL_SPEED_RPM*NOPOLESPAIRS/10
// ****************************************************************************** //
// ****************************************************************************** /
// *                                                                            * /
// *                    typedef definitions                                     * /
// *                                                                            * /
// ****************************************************************************** /
// ****************************************************************************** /
// *
tSincosParm         SincosParm;
tEstimParm 			EstimParm;
tMotorEstimParm 	MotorEstimParm;

/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: Estim                                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Estimation of the speed of the motor and field angle based on */
/* inverter voltages and motor currents.                                      */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Estim(void)
{
    long temp_int;

    // *******************************
    // dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
    // for lower speed the granularity of differnce is higher - the 
    // difference is made between 2 sampled values @ 8 ADC ISR cycles
    if ((long int)_Q15abs(EstimParm.qVelEstim)<(long int)NOMINAL_SPEED_RPM*NOPOLESPAIRS)
    {
    
    	EstimParm.qDIalpha	=	(ParkParm.qIalpha-EstimParm.qLastIalphaHS[(EstimParm.qDiCounter-7)&0x0007]);
    	// * the current difference can exceed the maximum value per 8 ADC ISR cycle 
    	// * the following limitation assures a limitation per low speed - up to the nominal speed 
    	if (EstimParm.qDIalpha>EstimParm.qDIlimitLS) EstimParm.qDIalpha=EstimParm.qDIlimitLS;
    	if (EstimParm.qDIalpha<-EstimParm.qDIlimitLS) EstimParm.qDIalpha=-EstimParm.qDIlimitLS;
    	EstimParm.qVIndalpha = (int)(__builtin_mulss(MotorEstimParm.qLsDt, EstimParm.qDIalpha)>>10);
     
    	EstimParm.qDIbeta	=	(ParkParm.qIbeta-EstimParm.qLastIbetaHS[(EstimParm.qDiCounter-7)&0x0007]);
    	// * the current difference can exceed the maximum value per 8 ADC ISR cycle 
    	// * the following limitation assures a limitation per low speed - up to the nominal speed 
    	if (EstimParm.qDIbeta>EstimParm.qDIlimitLS) EstimParm.qDIbeta=EstimParm.qDIlimitLS;
    	if (EstimParm.qDIbeta<-EstimParm.qDIlimitLS) EstimParm.qDIbeta=-EstimParm.qDIlimitLS;
    	EstimParm.qVIndbeta = (int)(__builtin_mulss(MotorEstimParm.qLsDt, EstimParm.qDIbeta)>>10);
    
    }
     else
    {
    
    	EstimParm.qDIalpha	=	(ParkParm.qIalpha-EstimParm.qLastIalphaHS[(EstimParm.qDiCounter)]);
    	// * the current difference can exceed the maximum value per 1 ADC ISR cycle 
    	// * the following limitation assures a limitation per high speed - up to the maximum speed 
    	if (EstimParm.qDIalpha>EstimParm.qDIlimitHS) EstimParm.qDIalpha=EstimParm.qDIlimitHS;
    	if (EstimParm.qDIalpha<-EstimParm.qDIlimitHS) EstimParm.qDIalpha=-EstimParm.qDIlimitHS;
    	EstimParm.qVIndalpha = (int)(__builtin_mulss(MotorEstimParm.qLsDt, EstimParm.qDIalpha)>>7);
    
    	EstimParm.qDIbeta	=	(ParkParm.qIbeta-EstimParm.qLastIbetaHS[(EstimParm.qDiCounter)]);
    	// * the current difference can exceed the maximum value per 1 ADC ISR cycle 
    	// * the following limitation assures a limitation per high speed - up to the maximum speed 
    	if (EstimParm.qDIbeta>EstimParm.qDIlimitHS) EstimParm.qDIbeta=EstimParm.qDIlimitHS;
    	if (EstimParm.qDIbeta<-EstimParm.qDIlimitHS) EstimParm.qDIbeta=-EstimParm.qDIlimitHS;
    	EstimParm.qVIndbeta= (int)(__builtin_mulss(MotorEstimParm.qLsDt, EstimParm.qDIbeta)>>7);
    
    }
    
    // *******************************
    // update  LastIalpha and LastIbeta
    EstimParm.qDiCounter=(EstimParm.qDiCounter+1) & 0x0007;
    EstimParm.qLastIalphaHS[EstimParm.qDiCounter]	=	ParkParm.qIalpha;
    EstimParm.qLastIbetaHS[EstimParm.qDiCounter] 	=	ParkParm.qIbeta;
    
    // *******************************
    // Stator voltage eqations
    // Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
    // BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt   
    
	EstimParm.qEsa		= 	EstimParm.qLastValpha -
							(int)(__builtin_mulss( MotorEstimParm.qRs, ParkParm.qIalpha)	>>14)
							- EstimParm.qVIndalpha;
    // * the multiplication between the Rs and Ialpha was shifted by 14 instead of 15 
    // * because the Rs value normalized exceeded Q15 range, so it was divided by 2 
    // * immediatelky after the normalization - in userparms.h 
    
    // Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
    // BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt   
	EstimParm.qEsb		= 	EstimParm.qLastVbeta -
							(int)(__builtin_mulss( MotorEstimParm.qRs, ParkParm.qIbeta )	>>14)
							- EstimParm.qVIndbeta;
							
    // the multiplication between the Rs and Ibeta was shifted by 14 instead of 15 
    // because the Rs value normalized exceeded Q15 range, so it was divided by 2 
    // immediatelky after the normalization - in userparms.h 
    
    // *******************************
    // update  LastValpha and LastVbeta
	EstimParm.qLastValpha = ParkParm.qValpha;
	EstimParm.qLastVbeta = ParkParm.qVbeta;


    // Calculate Sin(Rho) and Cos(Rho)
    SincosParm.qAngle 	=	EstimParm.qRho + EstimParm.RhoOffset; 
	
	SinCos();

    // *******************************
    //    Esd =  Esa*cos(Angle) + Esb*sin(Rho)
	EstimParm.qEsd		=	(int)((__builtin_mulss(EstimParm.qEsa, SincosParm.qCos)>>15)
							+
							(__builtin_mulss(EstimParm.qEsb, SincosParm.qSin)>>15));
    // *******************************
    //   Esq = -Esa*sin(Angle) + Esb*cos(Rho)
	EstimParm.qEsq		=	(int)((__builtin_mulss(EstimParm.qEsb, SincosParm.qCos)>>15)
							-
							(__builtin_mulss(EstimParm.qEsa, SincosParm.qSin)>>15));

    // *******************************
    // *******************************
    // Filter first order for Esd and Esq
    // EsdFilter = 1/TFilterd * Intergal{ (Esd-EsdFilter).dt }
 
	temp_int = (int)(EstimParm.qEsd - EstimParm.qEsdf);
	EstimParm.qEsdStateVar			+= __builtin_mulss(temp_int, EstimParm.qKfilterEsdq);
	EstimParm.qEsdf					= (int)(EstimParm.qEsdStateVar>>15);

	temp_int = (int)(EstimParm.qEsq - EstimParm.qEsqf);
	EstimParm.qEsqStateVar			+= __builtin_mulss(temp_int, EstimParm.qKfilterEsdq);
	EstimParm.qEsqf					= (int)(EstimParm.qEsqStateVar>>15);

    // OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf)
    // For stability the conditio for low speed
    if (_Q15abs(EstimParm.qVelEstim)>DECIMATE_NOMINAL_SPEED)
    {
    	if(EstimParm.qEsqf>0)
    	{
        	temp_int = (int)(EstimParm.qEsqf- EstimParm.qEsdf);
    		EstimParm.qOmegaMr	=	(int)(__builtin_mulss(MotorEstimParm.qInvKFi, temp_int)>>15);
    	} else
    	{
        	temp_int = (int)(EstimParm.qEsqf + EstimParm.qEsdf);
    		EstimParm.qOmegaMr	=   (int)(__builtin_mulss(MotorEstimParm.qInvKFi, temp_int)>>15);
    	}
    } else // if est speed<10% => condition VelRef<>0
    {
    	if(EstimParm.qVelEstim>0)
    	{
        	temp_int = (int)(EstimParm.qEsqf - EstimParm.qEsdf);
    		EstimParm.qOmegaMr	=	(int)(__builtin_mulss(MotorEstimParm.qInvKFi,temp_int)>>15);
    	} else
    	{
        	temp_int = (int)(EstimParm.qEsqf+ EstimParm.qEsdf);
    		EstimParm.qOmegaMr	=	 (int)(__builtin_mulss(MotorEstimParm.qInvKFi,temp_int)>>15);
    	}
    }
    // the result of the calculation above is shifted left by one because initally the value of InvKfi 
    // was shifted by 2 after normalizing - assuring that extended range of the variable is possible in the lookup table 
    // the initial value of InvKfi is defined in userparms.h 
    EstimParm.qOmegaMr=EstimParm.qOmegaMr<<1;
    
    	
    // the integral of the angle is the estimated angle 
	EstimParm.qRhoStateVar	+= __builtin_mulss(EstimParm.qOmegaMr, EstimParm.qDeltaT);
	EstimParm.qRho 		= 	(int) (EstimParm.qRhoStateVar>>15);


    // the estiamted speed is a filter value of the above calculated OmegaMr. The filter implementation 
    // is the same as for BEMF d-q components filtering 
    temp_int = (int)(EstimParm.qOmegaMr-EstimParm.qVelEstim);
	EstimParm.qVelEstimStateVar+=__builtin_mulss(temp_int, EstimParm.qVelEstimFilterK);
	EstimParm.qVelEstim=	(int)(EstimParm.qVelEstimStateVar>>15);

}	// End of Estim()

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: InitEstimParm                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialisation of the parameters of the estimator.            */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void	InitEstimParm(void)  
{
    // Constants are defined in usreparms.h

	MotorEstimParm.qLsDtBase = NORM_LSDTBASE;
	MotorEstimParm.qLsDt = MotorEstimParm.qLsDtBase;
	MotorEstimParm.qRs = NORM_RS;

	MotorEstimParm.qInvKFiBase = NORM_INVKFIBASE;
	MotorEstimParm.qInvKFi = MotorEstimParm.qInvKFiBase;

   	EstimParm.qRhoStateVar=0;
	EstimParm.qOmegaMr=0;
	EstimParm.qDiCounter=0;
	EstimParm.qEsdStateVar=0;
	EstimParm.qEsqStateVar=0;
		
	EstimParm.qDIlimitHS = D_ILIMIT_HS;
    EstimParm.qDIlimitLS = D_ILIMIT_LS;
        
    EstimParm.qKfilterEsdq = KFILTER_ESDQ;
    EstimParm.qVelEstimFilterK = KFILTER_VELESTIM;

    EstimParm.qDeltaT = NORM_DELTAT;
    EstimParm.RhoOffset = INITOFFSET_TRANS_OPEN_CLSD;

}
