/*
 * c_trans.c
 *
 *  Created on: Sep 8, 2023
 *      Author: Administrator
 */
#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#define NUM_ITERATIONS 60
#define MAX_TOLERANCE 0.0001 // maximum tolerance to determine stabilization

float32_t abs_f32(float32_t val);

/**
 * Finds the solution of the transcendental equation: x^2 = cos(omega*x + phi) using the Newton-Raphson Method,
 * where x is a solution to this equation, and omega,phi are constants. If there are
 * no solutions, returns 0. Will be precise up to 0.0001.
 */
void c_trans(const float32_t omega, const float32_t phi, float32_t *pOut) {

	// edge cases TODO


	// initial guess, using desmos on the function f(x) = x^2 - cos(wx + phi)
	// it is clear that a solution is often near x = 0, therefore initial guess is close to 0
	float32_t x_0 = 0.1;
	float32_t sin_arg;
	float32_t cos_arg;
	float32_t cur = -1;
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		sin_arg = arm_sin_f32(omega*x_0+phi);
		cos_arg = arm_cos_f32(omega*x_0+phi);
		// iteration step, this formula is computed using x_next = x_prev - f(x_prev)/f'(x_prev), according to the newton-raphson method
		x_0 = (x_0*x_0 + x_0*omega*sin_arg + cos_arg)/(2*x_0 + omega*sin_arg);
		// checking to see if we have stabilized at a value
		if (abs_f32(x_0-cur) <= MAX_TOLERANCE) {
			(*pOut) = x_0;
			return;
		}
		cur = x_0;
	}
	// if we have not stabilized according to MAX_TOLERANCE, then we assume that some error has occured (e.g., no solutions), and we return 0.
	(*pOut) = 0;
}


/**
 * Returns the float32_t absolute value of the input
 */
float32_t abs_f32(float32_t val) {
	if (val < 0) {
		return -val;
	} else {
		return val;
	}
}
