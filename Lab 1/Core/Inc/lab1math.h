/*
 * lab1math.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Zichen Gao
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_


void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmSqrt(float32_t in, float32_t *pOut);

void c_sqrt(const float32_t in, float32_t *pOut);

extern void asmTranscendental(const float32_t omega, const float32_t phi, float32_t *pOut);

void c_trans(const float32_t omega, const float32_t phi, float32_t *pOut);

#endif /* INC_LAB1MATH_H_ */

