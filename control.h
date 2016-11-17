#ifndef CONTROL_H
#define CONTROL_H

//------------------  C API for Control routine ---------------------

typedef struct {
    short   qVelRef;    // Reference velocity
    short   qVdRef;     // Vd flux reference value
    short   qVqRef;     // Vq torque reference value
	short	qRefRamp;	// Ramp for speed reference value
	short   qDiff;		// Speed of the ramp
    } tCtrlParm;

tCtrlParm CtrlParm;
#endif



