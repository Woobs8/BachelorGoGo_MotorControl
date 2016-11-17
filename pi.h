#ifndef PI_H
#define PI_H

//------------------  C API for PI routines ---------------------

typedef struct {
    long  qdSum;          // 1.31 format
    int   qKp;
    int   qKi;
    int   qKc;
    int   qOutMax;
    int   qOutMin;
    int   qInRef; 
    int   qInMeas;
    int   qOut;
    } tPIParm;

void InitPI( tPIParm *pParm);
void CalcPI( tPIParm *pParm);
#endif



