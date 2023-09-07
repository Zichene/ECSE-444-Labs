/*
 * cmax.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Zichen Gao
 */

#include "main.h"

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex) {
	*max = array[0];
	*maxIndex = 0;

	for (int i = 1; i < size; i++) {
		if (array[i] >= *max) {
			*max = array[i];
			*maxIndex = i;
		}
	}
}

