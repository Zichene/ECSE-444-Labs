/*
 * asmDiv.s
 *
 *  Created on: Sep 7, 2023
 *      Author: Armor
 */
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmMax

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * void asmDiv(*input, *output);
 *
 * R0 = pointer to input
 * R1 = output
 */

 asmDiv:
 	VLDR.f32 	S0, [R0]
 	VSQRT.F32 	S0, S0

 done:
 	VSTR.f32	S0, [R1]
 	BX LR
