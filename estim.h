#ifndef ESTIM_H
#define ESTIM_H


#include "park.h"
#include "fdweak.h"
#include "userparams.h"

#include <libq.h> // q15 abs function use /

typedef struct {
    int   			  qDeltaT;      // Integration constant
    int   			  qRho;    	    // angle of estimation
    long  			  qRhoStateVar; // internal variable for angle
    int   			  qOmegaMr;     // primary speed estimation
    int   			  qLastIalpha;  // last value for Ialpha
    int   			  qLastIbeta;   // last value for Ibeta
    int   			  qDIalpha;     // difference Ialpha
    int   			  qDIbeta;      // difference Ibeta
	int				  qEsa;			// BEMF alpha
	int				  qEsb;			// BEMF beta
	int				  qEsd;			// BEMF d
	int				  qEsq;			// BEMF q
	int				  qDiCounter;	// counter in Last DI tables
	int				  qVIndalpha;   // dI*Ls/dt alpha
	int				  qVIndbeta;    // dI*Ls/dt beta
	int				  qEsdf;        // BEMF d filtered
	long			  qEsdStateVar; // state var for BEMF d Filtered
	int				  qEsqf;        // BEMF q filtered
	long			  qEsqStateVar; // state var for BEMF q Filtered
	int				  qKfilterEsdq; // filter constant for d-q BEMF
	unsigned int   	  qVelEstim; 			// Estimated speed 
	int   			  qVelEstimFilterK; 	// Filter Konstant for Estimated speed 
	long   			  qVelEstimStateVar; 	// State Variable for Estimated speed 
    int   			  qLastValpha;  // Value from last control step Ialpha 
    int   			  qLastVbeta;   // Value from last control step Ibeta
	int				  qDIlimitLS;			// dIalphabeta/dt
	int				  qDIlimitHS;			// dIalphabeta/dt
	int				  qLastIalphaHS[8];		//  last  value for Ialpha
	int				  qLastIbetaHS[8];			// last  value for Ibeta
	int               RhoOffset;            // estima angle init offset

    } tEstimParm;


typedef struct {
	int				   qRs;			// Rs value - stator resistance
	int				   qLsDt;		// Ls/dt value - stator inductand / dt - variable with speed
	int				   qLsDtBase;	// Ls/dt value - stator inductand / dt for base speed (nominal)
	int				   qInvKFi;	    // InvKfi constant value ( InvKfi = Omega/BEMF )
	int				   qInvKFiBase; // InvKfi constant - base speed (nominal) value
    } tMotorEstimParm;

	

extern tEstimParm 	EstimParm;
extern tMotorEstimParm 	MotorEstimParm;
extern int temporary;
//------------------  C API for Control routine ---------------------

void	Estim(void);
void	InitEstimParm(void);

#endif
