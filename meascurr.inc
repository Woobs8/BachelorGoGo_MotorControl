;;------------------  ASM30 API for MeasCurr routines ---------------------
; MeasCurrParm stucture:

          .equ ADC_qKa,0          ; qKa - 1.15
          .equ ADC_iOffsetLa,2
          .equ ADC_iOffsetHa,4

          .equ ADC_qKb,6          ; qKb - 1.15
          .equ ADC_iOffsetLb,8
          .equ ADC_iOffsetHb,10
	  
	  .equ Adc1_Ch0,0
	  .equ Adc1_Ch1,2
	  .equ Adc1_Ch2,4
	  .equ Adc1_Ch3,6

          .extern _MeasCurrParm
	  .extern _BufferAdc
	  