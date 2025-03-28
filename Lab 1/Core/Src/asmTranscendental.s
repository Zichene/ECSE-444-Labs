.syntax unified

// .global exports the label asmTranscendental, which is expected by lab1math.h
.global asmTranscendental

// Setup to call cosine
.extern arm_cos_f32
.extern arm_sin_f32

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * void asmTranscendental(float omega, float phi, float *pOut);
 *
 * S0 = omega input/general purpose
 * S1 = phi input
 * R0 = output
 *
 * S2 = sin_arg*omega result, later numerator
 * S3 = tolerance
 * S4 = x0 and starting guess
 * S5 = omega holder
 * S6 = omega*x0+phi/general purpose
 * S7 = current
 * R1 = iterator
 */

asmTranscendental:
PUSH {LR, V1}
VMOV S3, #1
VMOV S4, #1
VMOV S7, #10
VDIV.F32 S4, S4, S7
VMUL.F32 S7, S7, S7
VMUL.F32 S7, S7, S7
VDIV.F32 S3, S3, S7
VMOV S5, S0
VMOV S7, #-1
// VMOV S8, #20
MOV V1, #60

loop:
//tewt
//sin and cosin prep
//objective is to create sin_arg*omega for quicker processing
VMOV        S6, S1
// FIX: S4 wasnt getting updated to x0 after first iteration
VMOV S4, S0
VMLA.F32     S6, S5, S4 //this gives us omega*x0+phi
VMOV         S0, S6
BL arm_sin_f32
VMUL.F32    S2, S0, S5
VMOV         S0, S6
BL arm_cos_f32

//Now we have S2=sin_arg*omega, S0=cos_arg, S4=x0, S6 is now freed up
//time to do more math
VMLA.F32     S0, S4, S4 //takes care of x0^2+cos_arg
VMLA.F32    S0, S2, S4 //takes care of x0^2+cos_arg+x_0*omega*sin_arg
VMOV        S6, #2
VMLA.F32    S2, S6, S4  //
VDIV.F32    S0, S0, S2
//Check to see if stabilization has occured using tolerance

VSUB.F32    S6, S0, S7
VABS.F32    S6, S6
VCMP.F32    S6, S3
FMSTAT
BGT no_soln
BLE            done

//iteration loop
/*
VMOV        S7, #1
VSUB.F32     S8, S8, S7
VMOV         S7, S0
VCMP.F32    S8, #0
FMSTAT
*/
VMOV S7, S0
SUBS V1, V1, #1
BGT loop
//TODO: if no solution is found
no_soln:
VSUB.F32 S0, S0, S0

done:
// VMOV S0, S5
VSTR.32        S0, [R0]
POP {LR, V1}
BX LR                    // return
