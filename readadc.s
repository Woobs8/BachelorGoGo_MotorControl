; ReadADC0 and ReadSignedADC0
;  
;Description:        
;  Read Channel 0 of ADC, scale it using qK and put results in qADValue.
;  Do not call this routine until conversion is completed.
;
;  ReadADC0 range is qK*(0.0 ->0.9999).
;  ReadSignedADC0 range is qK*(-1.0 ->0.9999).
;
;  Scaling constant, qK, must be set elsewhere such that
;         iResult = 2 * qK * ADCBUF0
;  The factor of 2 is designed to allow qK to be given in 1.15.
;
;
;Functional prototype:
; 
; void ReadADC0( tReadADCParm* pParm )       : Calculates unsigned value 0 -> 2*qK
; void ReadSignedADC0( tReadADCParm* pParm ) : Calculates signed value -2*qK -> 2*qK
;
;On Entry:   ReadADCParm structure must contain qK. ADC channel 0
;            must contain signed fractional value.
;
;On Exit:    ReadADCParm will contain qADValue
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
; Registers modified: w0,w4,w5
;
; Timing: 13 cycles
;
;*******************************************************************
;
          .include "general.inc"

; External references
          .include "ReadADC.inc"

; Register usage
          .equ ParmBaseW,w0  ; Base of parm structure
          .equ Work0W,   w4
          .equ Work1W,   w5



;=================== CODE =====================

          .section  .text  
	   	 .global   _ReadADC0
         .global   ReadADC0
		 

_ReadADC0:
ReadADC0:

     ;; iResult = 2 * qK * ADCBUF0

          mov.w     [ParmBaseW+ADC_qK],Work0W
		  mov.w		_ADC1BUF0, Work1W


     ;; change from signed fractional to fractional, i.e. convert
     ;; from -1->.9999 to 0 -> 0.9999
          btg       Work1W,#15
          lsr.w     Work1W,Work1W

          mpy       Work0W*Work1W,A
          sac       A,#-1,Work0W
          mov.w     Work0W,[ParmBaseW+ADC_qADValue]
          return


          .global   _ReadSignedADC0
          .global   ReadSignedADC0

_ReadSignedADC0:
ReadSignedADC0:

     ;; iResult = 2 * qK * ADCBUF0
      
		  mov.w     [ParmBaseW+ADC_qK],Work0W
          mov.w		_ADC1BUF0, Work1W

          mpy       Work0W*Work1W,A
          sac       A,#-1,Work0W
          mov.w     Work0W,[ParmBaseW+ADC_qADValue]

		return

          .end


