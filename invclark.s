; InvClarke
;  
;Description:        
;  Calculate the inverse Clarke transform. Assumes the Cos and Sin values 
;  are in the ParkParm structure.
;
;         V1 = Valpha
;         V2 = -Valpha/2 + Vbeta * Sq3/2
;         V3 = -Valpha/2 - Vbeta * Sq3/2
;
;  This routine works the same for both integer scaling and 1.15 scaling.
;
;Functional prototype:
; 
; void InvClarke( void )
;
;On Entry:   The ParkParm structure must contain qCos, qSin, qValpha and qVbeta.
;
;On Exit:    ParkParm will contain qV1, qV2, qV3.
;
;Parameters: 
; Input arguments: None
;
; Return:
;   Void
;
; SFR Settings required:
;         CORCON.SATA  = 0

; Support routines required: None
;
; Local Stack usage: none
;
; Registers modified: w0, w4,w5,w6
;
; Timing:
;  About 15 instruction cycles
;
;*******************************************************************
;
          .include "general.inc"

; External references
          .include "park.inc"

; Register usage
          .equ WorkW,   w0    ; working

          .equ ValphaW, w4    ; copy of qValpha  and later -Valpha
          .equ VbetaW,  w5    ; copy of qVbeta
          .equ Sq3By2W, w6    ; copy of qVbeta

; Constants
          .equ Sq3By2,0x6EDA      ; sqrt(3)/2 in 1.15 format


;=================== CODE =====================

          .section  .text
          .global   _InvClarke
          .global   InvClarke

_InvClarke:
InvClarke:
     ;; Get qValpha, qVbeta from ParkParm structure
          mov.w     _ParkParm+Park_qValpha,ValphaW
          mov.w     _ParkParm+Park_qVbeta,VbetaW

     ;; Load Sq(3)/2
          mov.w     #Sq3By2,Sq3By2W

     ;; Put qV1 = qValpha
          mov.w     ValphaW,_ParkParm+Park_qV1

     ;; AccA = -Valpha/2
          neg.w     ValphaW,ValphaW
          lac       ValphaW,#1,A

     ;; V2 = -Valpha/2 + Vbeta * Sq3/2
          mac       VbetaW*Sq3By2W,A   ; add Vbeta*sq(3)/2 to A
          sac       A,WorkW
          mov.w     WorkW,_ParkParm+Park_qV2

          lac       ValphaW,#1,A        ; -Valpha/2

     ;; V3 = -Valpha/2 - Vbeta * Sq3/2
          msc       VbetaW*Sq3By2W,A   ; sub Vbeta*sq(3)/2 from A
          sac       A,WorkW
          mov.w     WorkW,_ParkParm+Park_qV3
          return
          .end



