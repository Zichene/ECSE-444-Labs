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
 * void asmDiv(*input, *output);
 *
 * S0 = pointer to input
 * R0 = output
 */

 asmSqrt:
 	VMOV 		S1, #1
 	VNEG.F32	S1, S1
 	VCMP.F32	S0, S1
 	BLE			done
 	VSQRT.F32 	S0, S0

 done:
 	VSTR.32	S0, [R0]
 	BX LR
