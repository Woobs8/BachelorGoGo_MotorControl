; MeasCompCurr
;  
;Description:        
;  Read Channels 1 & 2 of ADC, scale them as signed fractional values 
;  using qKa, qKb and put the results qIa and qIb of ParkParm. 
;  Running average value of ADC-Ave is maintained and subtracted from 
;  ADC value before scaling.
;
;  Specifically the offset is accumulated as a 32 bit signed integer       
;         iOffset += (ADC-Offset)
;  and is used to correct the raw ADC by
;         CorrADC = ADCBUFn - iOffset/2^16
;  which gives an offset time constant of ~ MeasurementPeriod*2^16
;
;  Do not call this routine until conversion is completed.
;
;  Scaling constant, qKa and qKb, must be set elsewhere such that
;         qIa = 2 * qKa * CorrADC1
;         qIb = 2 * qKb * CorrADC2
;  The factor of 2 is designed to allow qKa & qKb to be given in 1.15.
;
;
;Functional prototypes:  
;         void MeasCompCurr( void );
;         void InitMeasCompCurr( short iOffset_a, short iOffset_b );
;
;On Start:   Must call InitMeasCompCurr.
;
;On Entry:   MeasCurrParm structure must contain qKa & qKb. ADC channels 1 & 2
;            must contain signed fractional value.
;
;On Exit:    ParkParm will contain qIa & qIb.
;
;Parameters: 
; Input arguments: None
;
; Return:
;   Void
;
; SFR Settings required:
;         CORCON.SATA  = 0
;     If there is any chance that Accumulator will overflow must set
;         CORCON.SATDW = 1  
;
; Support routines required: None
;
; Local Stack usage: None
;
; Registers modified: w0,w1,w4,w5
;
; Timing: 29 cycles
;
;*******************************************************************

          .include "general.inc"

; External references
          .include "meascurr.inc"
          .include "park.inc"

; Register usage
          .equ Work0W,   w4
          .equ Work1W,   w5
          .equ OffsetHW, w3

          .global   _MeasCompCurr
          .global   MeasCompCurr

_MeasCompCurr:
MeasCompCurr:

     ;; CorrADC1 = ADCBUF2 - iOffsetHa/2^16
     ;; qIa = 2 * qKa * CorrADC1
          mov.w     _MeasCurrParm+ADC_iOffsetHa,w0
	      sub.w     _BufferAdc+Adc1_Ch1,WREG               ; w0 = ADC - Offset
   
		  clr.w     w1
          btsc      w0,#15
          setm      w1
          mov.w     w0,w5
          mov.w     _MeasCurrParm+ADC_qKa,w4
          mpy       w4*w5,A
          sac       A,w4
          mov.w     w4,_ParkParm+Park_qIa

     ;; iOffset += (ADC-Offset)
          add       _MeasCurrParm+ADC_iOffsetLa    
          mov.w     w1,w0
          addc      _MeasCurrParm+ADC_iOffsetHa


     ;; CorrADC2 = ADCBUF3 - iOffsetHb/2^16
     ;; qIb = 2 * qKb * CorrADC2
          mov.w     _MeasCurrParm+ADC_iOffsetHb,w0
    	  sub.w     _BufferAdc+Adc1_Ch0,WREG               ; w0 = ADC - Offset         

		  clr.w     w1
          btsc      w0,#15
          setm      w1
          mov.w     w0,w5
          mov.w     _MeasCurrParm+ADC_qKb,w4
          mpy       w4*w5,A
          sac       A,#-1,w4
          mov.w     w4,_ParkParm+Park_qIb

     ;; iOffset += (ADC-Offset)
          add       _MeasCurrParm+ADC_iOffsetLb    
          mov.w     w1,w0
	    addc      _MeasCurrParm+ADC_iOffsetHb   

          return

          .global   _InitMeasCompCurr
          .global   InitMeasCompCurr

_InitMeasCompCurr:
InitMeasCompCurr:

          clr.w     _MeasCurrParm+ADC_iOffsetLa    
          mov.w     w0,_MeasCurrParm+ADC_iOffsetHa    
          clr.w     _MeasCurrParm+ADC_iOffsetLb    
          mov.w     w1,_MeasCurrParm+ADC_iOffsetHb    
          return

          .end





