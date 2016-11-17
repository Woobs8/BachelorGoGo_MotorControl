#ifndef FDWEAK_H
#define FDWEAK_H

#include    "userparams.h"
#include    "estim.h"

typedef struct {
	int		qIdRef;          // d-current reference
	int		qFwOnSpeed;      // flux weakening on speed -
	int		qIndex;          // lookup tables index
    int		qFwCurve[18];	 // Curve for magnetizing current variation with speed
    int		qInvKFiCurve[18];// Curve for InvKfi constant InvKfi = Omega/BEMF variation with speed
    int     qLsCurve[18];    // Curve for Ls variation with speed
    } tFdWeakParm;

    
extern tFdWeakParm FdWeakParm;

//------------------  C API for FdWeak routine ---------------------
void InitFWParams();
int FieldWeakening( int qMotorSpeed );

#endif

