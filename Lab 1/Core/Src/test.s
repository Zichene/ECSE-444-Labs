/*
 * test.s
 *
 *  Created on: Sep 9, 2023
 *      Author: Administrator
 */

 // unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global test

// trying to call library function (sine)
.extern arm_sin_f32
// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata



/**
* void test(float32_t *pIn);
* R0: pointer to input float
*/

test:
PUSH {R4, R5, LR}
VLDR.f32 S0, [R0] // move the input float in register S0
BL arm_sin_f32 // trying to call arm_sin_f32
MOV R4, #1
MOV R5, #2
POP {R4, R5, LR}
BX LR

