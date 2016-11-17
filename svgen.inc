;;------------------  ASM30 API for SVGen routines ---------------------
; SVGenParm stucture:
;
        .equ SVGen_iPWMPeriod,0  

        .equ SVGen_qVr1,2        
        .equ SVGen_qVr2,4        
        .equ SVGen_qVr3,6

        .extern _SVGenParm

		.extern _dPWM1Copy
		.extern _dPWM2Copy
		.extern _dPWM3Copy