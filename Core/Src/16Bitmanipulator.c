/*
 * 16Bitmanipulator.c
 *
 *  Created on: Apr 1, 2023
 *      Author: mthudaa
 */
#include "16Bitmanipulator.h"

void intTo16Bit(int num, int ind, uint8_t* buffer) {
  buffer[ind] = (num >> 8) & 0xFF;  // Byte pertama
  buffer[ind+1] = num & 0xFF;         // Byte kedua
}

int bit16ToInt(int ind, uint8_t* buffer) {
  int num = ((int) buffer[ind] << 8) | (int) buffer[ind+1];
  if (num > 32767) {
    num -= 65536;
  }
  return num;
}

void merge16(int16_t *values, uint8_t *merged_data, size_t num_values) {
    for (size_t i = 0; i < num_values; i++) {
        merged_data[i * 2] = (uint8_t)(values[i] & 0xFF);
        merged_data[i * 2 + 1] = (uint8_t)((values[i] >> 8) & 0xFF);
    }
}

