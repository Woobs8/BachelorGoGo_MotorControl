#ifndef READADC_H
#define READADC_H

typedef struct {
    short   qK;         // 1.15 
    short   qADValue;   // 1.15
    short   qAnRef;		// 1.15
    } tReadADCParm;

extern tReadADCParm ReadADCParm;

//------------------  C API for ReadADC routines ---------------------

void ReadADC0( tReadADCParm* pParm );       // Returns unsigned value 0 -> 2*iK 
void ReadSignedADC0( tReadADCParm* pParm ); // Returns signed value -2*iK -> 2*iK

#endif

