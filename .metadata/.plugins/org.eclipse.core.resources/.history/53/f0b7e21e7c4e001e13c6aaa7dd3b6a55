/*
 * c_sqrt.c
 *
 *  Created on: Sep 7, 2023
 *      Author: agree
 */
#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#define NUM_ITERATIONS 30
/**
 * Implementation of the square root function using Newton-Raphson method.
 * Returns -1 if parameter 'in' is negative.
 */
void c_sqrt(float32_t in, float32_t *pOut) {
	// edge cases
	if (in < 0) {
		(*pOut) = -1;
		return;
	}

	if (in == 0) {
		(*pOut) = 0;
		return;
	}

	// initial guess x_0
	float32_t x_0 = in/2; // divide by 2
	float32_t cur = -1;

	// iteration loops
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		x_0 = (x_0*x_0 + in)/(2*x_0);
		// checking to see if we have stabilized on a value
		if (cur == x_0) {
			break;
		}
		cur = x_0;
	}
	(*pOut) = x_0;
}
