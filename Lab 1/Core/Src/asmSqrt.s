/*
 * asmDiv.s
 *
 *  Created on: Sep 7, 2023
 *      Author: Armor
 */
.syntax unified

// .global exports the label asmDiv, which is expected by lab1math.h
.global asmSqrt

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * void asmSqrt(input, *output);
 *
 * S0 = input float
 * R0 = pointer to output
 * if input is negative, will return the input itself
 */

 asmSqrt:
 	VMOV 		S1, #1
 	VSUB.F32	S1, S1
 	VCMP.F32	S0, S1
 	FMSTAT // copies FPSCR flags into CPSR so that we can use BLT
 	BLT			done
 	VSQRT.F32 	S0, S0

 done:
 	VSTR.32	S0, [R0]
 	BX LR
