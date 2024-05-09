/*
 * 16Bitmanipulator.h
 *
 *  Created on: Apr 1, 2023
 *      Author: mthudaa
 */

#ifndef INC_16BITMANIPULATOR_H_
#define INC_16BITMANIPULATOR_H_

#include "main.h"
#include "stdio.h"
#include "stdint.h"

void intTo16Bit(int num, int ind, uint8_t* buffer);
int bit16ToInt(int ind, uint8_t* buffer);
void merge16(int16_t *values, uint8_t *merged_data, size_t num_values);


#endif /* INC_16BITMANIPULATOR_H_ */
