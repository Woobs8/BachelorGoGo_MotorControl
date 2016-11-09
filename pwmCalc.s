#include "pwm.h"
    ; Register usage                    
.equ WorkDLoW,  w4  ; double word (multiply results)
.equ WorkDHiW,  w5  ; double word (multiply results)

.global _dutyCyclePDC1
.global dutyCyclePDC1
.global _dutyCyclePDC2
.global dutyCyclePDC2
.global _dutyCyclePDC3
.global dutyCyclePDC3
 
_dutyCyclePDC1:
dutyCyclePDC1:
    mul.uu	w0,w1,WorkDLoW
    sl		WorkDHiW,WorkDHiW
    mov.w	WorkDHiW,PDC1
return
 
_dutyCyclePDC2:
dutyCyclePDC2:
    mul.uu	w0,w1,WorkDLoW
    sl		WorkDHiW,WorkDHiW
    mov.w	WorkDHiW,PDC2
return

_dutyCyclePDC3:
dutyCyclePDC3:
    mul.uu	w0,w1,WorkDLoW
    sl		WorkDHiW,WorkDHiW
    mov.w	WorkDHiW,PDC3
return
.end


