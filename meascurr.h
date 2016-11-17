#ifndef MEASCURR_H
#define MEASCURR_H

typedef struct {
    short   qKa;        // 1.15 
    short   iOffsetLa;
    short   iOffsetHa;

    short   qKb;        // 1.15 
    short   iOffsetLb;
    short   iOffsetHb;

    } tMeasCurrParm;

extern tMeasCurrParm MeasCurrParm;
    
//------------------  C API for MeasCurr routines ---------------------

void MeasCurr( void );
void MeasCompCurr( void );
void InitMeasCompCurr( short iOffset_a, short iOffset_b );

#endif

