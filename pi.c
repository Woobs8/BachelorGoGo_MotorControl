#include "pi.h"

void InitPI(tPIParm *pParm)
{
    pParm->qdSum = 0;
    pParm->qOut = 0;
}

void CalcPI( tPIParm *pParm)
{
 /*   
;    Err  = Ref - Meas
;    U  = Sum + Kp * Err
;    if( U > Outmax )
;        Out = Outmax
;    else if( U < Outmin )
;        Out = Outmin
;    else        
;        Out = U 
;    Exc = U - Out
;    Sum = Sum + Ki * Err - Kc * Exc

*/
	int currentError;
    long U;
    int outTemp;
    
    // Err = REF - MEAS
    currentError = pParm->qInRef - pParm->qInMeas;
    
    // U = Kp * Err
    U = __builtin_mulss(currentError, pParm->qKp)<<4;
    // U = U + Sum
    U = U + pParm->qdSum;
    
    //
    outTemp = (int)(U>>15);
	if(outTemp >  pParm->qOutMax)
		pParm->qOut=  pParm->qOutMax;
	else if(outTemp < pParm->qOutMin)
		pParm->qOut =  pParm->qOutMin;
	else
		pParm->qOut = outTemp;
	
	// Integrate
	U = __builtin_mulss(currentError, pParm->qKi);
    
    // OutError = CalcOut - SetOut ...... Will be 0 if Output could be set. If MAX or Min Limit we have an error
	currentError = outTemp - pParm->qOut;
    
    // If we excessed the The min or Max Limit Integral Windup
	U -= __builtin_mulss(currentError,  pParm->qKc);

    // save the integral sum
	pParm->qdSum = pParm->qdSum + U;
}
