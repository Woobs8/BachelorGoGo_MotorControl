;;------------------  ASM30 API for Control routines ---------------------
; CtrlParm stucture:
;
          .equ Ctrl_qVelRef,0      ; Reference velocity       
          .equ Ctrl_qVdRef ,2      ; Vd torque reference value
          .equ Ctrl_qVqRef ,4      ; Vq flux reference value  
          .equ Ctrl_qRefRamp,6	   ; Ramp for speed reference value
          .equ Ctrl_qDiff,8		   ; Speed of the ramp

          .extern _CtrlParm
