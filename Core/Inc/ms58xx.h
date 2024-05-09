/*
 * MS58xx.h
 *
 *  Created on: Feb 22, 2021
 *      Author: Ravi Kirschner
 */

#ifndef SRC_MS58xx_H_
#define SRC_MS58xx_H_

#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"

typedef enum
{
	MS5803 = 1,
	MS5837 = 2
} MS58xx_type;

typedef enum
{
	PRESSURE = 0x00,
	TEMPERATURE = 0x10
} measurement; //whether to measure pressure or temperature

typedef enum
{
	ADC_256  = 0x00,
	ADC_512  = 0x02,
	ADC_1024 = 0x04,
	ADC_2048 = 0x06,
	ADC_4096 = 0x08
}  precision; //what level of precision do we want

#define MS58xx_RESET 0x1E // reset command
#define MS58xx_ADC_READ 0x00 // ADC read command
#define MS58xx_ADC_CONV 0x40 // ADC conversion command
#define MS58xx_PROM 0xA0 // Coefficient location
#define MS58xx_I2C_ADDRESS(address)  ((address) << 1)

////#define MS58xx_ADDR 0x76 << 1 //MS58xx Address Lo
//#define MS58xx_ADDR 0x77 << 1 //MS58xx Address Hi

extern uint16_t MS58xx_coefficient[6]; //coefficients

HAL_StatusTypeDef MS58xx_reset(void *handle, uint8_t address);
HAL_StatusTypeDef MS58xx_coeff(void *handle, uint16_t* coeff, uint8_t address, uint8_t value);
uint32_t MS58xx_ADC(void *handle, measurement type, precision prec, uint8_t address);
void MS58xx_get_values(void *handle, precision prec, uint16_t coeff[6], float *temperature, float *pressure, uint8_t address, MS58xx_type sensor_type);
HAL_StatusTypeDef MS58xx_read(void *handle, uint8_t *bufp, uint16_t len, uint8_t address);
HAL_StatusTypeDef MS58xx_write(void *handle, uint8_t *bufp, uint16_t len, uint8_t address);
//HAL_StatusTypeDef MS58xx_reset(void *handle); //reset function
//HAL_StatusTypeDef MS58xx_coeff(void *handle, uint16_t* coeff, uint8_t value); //coefficient function
//uint32_t MS58xx_ADC(void *handle, measurement type, precision prec); //ADC function
//void MS58xx_get_values(void *handle, precision prec, uint16_t coeff[6], float *temperature, float *pressure); //get temperature and pressure.
//HAL_StatusTypeDef MS58xx_read(void *handle, uint8_t *bufp, uint16_t len); //read from MS58xx
//HAL_StatusTypeDef MS58xx_write(void *handle, uint8_t *bufp, uint16_t len); //write command to MS58xx


#endif /* SRC_MS58xx_H_ */
