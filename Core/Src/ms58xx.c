/*
 * MS58xx.c
 *
 *  Created on: Feb 22, 2021
 *      Author: Ravi Kirschner
 */

#include "ms58xx.h"
#include "cmsis_os.h"


uint8_t c[3];
/**
 * @brief Reads from MS58xx
 * @param handle The I2C handle being used
 * @param bufp The buffer to be read into
 * @param len The length of the buffer in 8-bit increments
 * @retval HAL Status
 */
HAL_StatusTypeDef MS58xx_read(void *handle, uint8_t *bufp, uint16_t len, uint8_t address) {
    return HAL_I2C_Master_Receive(handle, MS58xx_I2C_ADDRESS(address), bufp, len, 100);
}

//HAL_StatusTypeDef MS58xx_read(void *handle, uint8_t *bufp, uint16_t len) {
//	return HAL_I2C_Master_Receive(handle, MS58xx_ADDR, bufp, len, 100);
//}

/**
 * @brief Writes to MS58xx
 * @param handle The I2C handle being used
 * @param bufp The buffer to read from
 * @param len The length of the buffer in 8-bit increments
 * @retval HAL Status
 */
HAL_StatusTypeDef MS58xx_write(void *handle, uint8_t *bufp, uint16_t len, uint8_t address) {
    return HAL_I2C_Master_Transmit(handle, MS58xx_I2C_ADDRESS(address), bufp, len, 100);
}

//HAL_StatusTypeDef MS58xx_write(void *handle, uint8_t *bufp, uint16_t len) {
//	return HAL_I2C_Master_Transmit(handle, MS58xx_ADDR, bufp, len, 100);
//}

/**
 * @brief Resets the MS58xx
 * @param handle The I2C Handle being used
 * @retval HAL Status
 */

HAL_StatusTypeDef MS58xx_reset(void *handle, uint8_t address) {
    uint8_t buf[12];
    buf[0] = MS58xx_RESET;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(handle, MS58xx_I2C_ADDRESS(address), buf, 1, 1000);
    osDelay(3);
    return ret;
}

//HAL_StatusTypeDef MS58xx_reset(void *handle) {
//	uint8_t buf[12];
//	buf[0] = MS58xx_RESET;
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(handle, MS58xx_ADDR, buf, 1, 1000);
//	osDelay(3);
//	return ret;
//}

/**
 * @brief Gets the 6 Coefficients from the MS58xx and reads them into the MS58xx_coefficient array.
 * @param handle The I2C Handle being used
 * @param coeff The pointer to the coefficient being read in to
 * @param value The coefficient number
 * @return HAL Status
 */

HAL_StatusTypeDef MS58xx_coeff(void *handle, uint16_t *coeff, uint8_t address, uint8_t value) {
    uint8_t buf[12];
    buf[0] = MS58xx_PROM + (value << 1);
    HAL_StatusTypeDef x = MS58xx_write(handle, buf, 1, address);
    osDelay(2);
    uint8_t c[2];
    x = MS58xx_read(handle, c, 2, address);
    *coeff = (c[0] << 8) + c[1];
    return x;
}

//HAL_StatusTypeDef MS58xx_coeff(void *handle, uint16_t *coeff, uint8_t value) {
//	uint8_t buf[12];
//	buf[0] = MS58xx_PROM + (value << 1); //coefficient to read
//	HAL_StatusTypeDef x = MS58xx_write(handle, buf, 1); //tell MS58xx that we want it
//	osDelay(2); //delay until it is ready
//	uint8_t c[2];
//	x = MS58xx_read(handle, c, 2); //read the coefficient
//	*coeff = (c[0] << 8) + c[1]; //turn the two 8-bit values into one coherent value.
//	return x;
//}

/**
 * @brief Reads the MS58xx ADC
 * @param handle The I2C Handle being used
 * @param type The measurement type, chosen from measurement enum
 * @param prec The precision to use, chosen from precision enum
 * @retval Raw 24-bit data from the ADC
 */
uint32_t MS58xx_ADC(void *handle, measurement type, precision prec, uint8_t address) {
    uint32_t result;
    uint8_t buf[12];
    buf[0] = MS58xx_ADC_CONV + type + prec; // Tell the ADC to convert along with the precision and type
    MS58xx_write(handle, buf, 1, address);
    osDelay(2);

    switch (prec) {
        case ADC_256: osDelay(1); break;
        case ADC_512: osDelay(3); break;
        case ADC_1024: osDelay(4); break;
        case ADC_2048: osDelay(6); break;
        case ADC_4096: osDelay(10); break;  // Delay longer if higher precision, as conversion takes longer.
    }

    buf[0] = MS58xx_ADC_READ; // Tell the MS58xx that we want to read the ADC
    MS58xx_write(handle, buf, 1, address);
    osDelay(2);

    uint8_t c[3];
    MS58xx_read(handle, c, 3, address); // Read out the ADC
    result = (c[0] << 16) + (c[1] << 8) + c[2]; // Convert the three 8-bit values into one value.
    return result;
}

//uint32_t MS58xx_ADC(void *handle, measurement type, precision prec) {
//	uint32_t result;
//	uint8_t buf[12];
//	buf[0] = MS58xx_ADC_CONV + type + prec; //tell the ADC to convert along with the precision and type
//	MS58xx_write(handle, buf, 1);
//	osDelay(2);
//	switch(prec) {
//		case ADC_256: osDelay(1);
//		case ADC_512: osDelay(3);
//		case ADC_1024: osDelay(4);
//		case ADC_2048: osDelay(6);
//		case ADC_4096: osDelay(10); //Delay longer if higher precision, as conversion takes longer.
//	}
//	buf[0] = MS58xx_ADC_READ; //Tell the MS58xx that we want to read the ADC
//	MS58xx_write(handle, buf, 1);
//	osDelay(2);
//	//uint8_t c[3];
//	MS58xx_read(handle, c, 3); //Read out the ADC
//	result = (c[0] << 16) + (c[1] << 8) + c[2]; //Convert the three 8-bit values into one value.
//	return result;
//}

/**
 * @brief Gets temperature and pressure values from the MS58xx
 * @param handle The I2C Handle being used
 * @param prec The precision to be used
 * @param temperature The pointer to the temperature variable being read in to.
 * @param pressure The pointer to the pressure variable being read in to.
 */
void MS58xx_get_values(void *handle, precision prec, uint16_t coeff[6], float *temperature, float *pressure, uint8_t address, MS58xx_type sensor_type) {
    uint32_t temperature_raw = MS58xx_ADC(handle, TEMPERATURE, prec, address);
    uint32_t pressure_raw = MS58xx_ADC(handle, PRESSURE, prec, address); // get temperature and pressure raw values

    int32_t sub = coeff[4] * 256;
    int32_t dT = temperature_raw - sub;

    int64_t add = ((int64_t)coeff[4])*((int64_t)dT)/128;
    int64_t OFF = ((int64_t)coeff[2])*65536+add;
    int64_t SENS = coeff[1] * (32768) + (coeff[3]*dT)/(256);
    int64_t mult = pressure_raw*SENS/2097152;

    int32_t pres;
    if (sensor_type == MS5803) {
        pres = (mult-OFF)/32768;
    } else if (sensor_type == MS5837) {
        pres = (mult-OFF)/8192;
    } else {
        // Handle unknown sensor type
        pres = 0; // Set a default value or handle the case appropriately
    }

    // determine pressure according to datasheet
    if (sensor_type == MS5803) {
        *pressure = (pres/100.f) + 819;
    } else if (sensor_type == MS5837) {
        *pressure = (pres/100.f);
    }
}

//void MS58xx_get_values(void *handle, precision prec, uint16_t coeff[6], float *temperature, float *pressure) {
//	uint32_t temperature_raw = MS58xx_ADC(handle, TEMPERATURE, prec);
//	uint32_t pressure_raw = MS58xx_ADC(handle, PRESSURE, prec); //get temperature and pressure raw values
//
//	int32_t sub = coeff[4] * 256;
//	int32_t dT = temperature_raw - sub;
//
//	int64_t add = ((int64_t)coeff[4])*((int64_t)dT)/128;
//	int64_t OFF = ((int64_t)coeff[2])*65536+add;
//	int64_t SENS = coeff[1] * (32768) + (coeff[3]*dT)/(256);
//	int64_t mult = pressure_raw*SENS/2097152;
//	int32_t pres = (mult-OFF)/32768;
//	*pressure = (pres/100.f) + 819; //determine pressure according to datasheet
//}
