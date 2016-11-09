#include "pwm.h"
    ; Register usage                    
.equ WorkDLoW,  w4  ; double word (multiply results)
.equ WorkDHiW,  w5  ; double word (multiply results)

.global _dutyCycle
.global dutyCycle
 
_dutyCycle:
dutyCycle:
    mul.uu	w0,w1,WorkDLoW
    sl		WorkDHiW,WorkDHiW
    mov.w	WorkDHiW,PDC1
return
.end


