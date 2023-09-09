/*
 * lab1math.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Zichen Gao
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_


/**
 * Finds the maximum value in an array [*array] and its associated index, and returns them
 * in [*max] and [*maxIndex] respectively.
 */
void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

/**
 * Assembly implementation of the same function above.
 */
extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmSqrt(float32_t in, float32_t *pOut);

/**
 * Finds the square root of a number using the Newton-Raphson method
 */
void c_sqrt(const float32_t in, float32_t *pOut);

#endif /* INC_LAB1MATH_H_ */

