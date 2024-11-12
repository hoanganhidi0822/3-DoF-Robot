/************************************************************************
Title:	  as5600.c - Driver for AMS AS5600 12-Bit Programmable Contactless
                                         Potentiometer
Author:   Nicholas Morrow <nickhudspeth@gmail.com>
File:     as5600.c
Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
Hardware: STM32Fxxx
License:  The MIT License (MIT)
Usage:    Refer to the header file as5600.h for a description of the routines.
          See also example test_as5600.c, if available.
LICENSE:
    Copyright (C) 2018 Nicholas Morrow

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/

#include "as5600.h"
/**********************    GLOBAL VARIABLES    ***********************/

/*******************    FUNCTION IMPLEMENTATIONS    ********************/
AS5600_TypeDef *AS5600_New(void) {
    AS5600_TypeDef *a = (AS5600_TypeDef *)calloc(1, sizeof(AS5600_TypeDef));
    return a;
}

HAL_StatusTypeDef AS5600_Init(AS5600_TypeDef *a) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t pwm = 0;
    uint8_t mag_status = 0;
    /* Set configuration defaults for uninitialized values. */
    if (!(a->PositiveRotationDirection)) {
        a->PositiveRotationDirection = AS5600_DIR_CW;
    }

    /* Write configuration settings.
       Do this in single write instead of using functions below to avoid
       overhead of multiple calls to HAL_I2C_Mem_Write_IT */

    if (HAL_I2C_Mem_Write_IT(a->i2cHandle, a->i2cAddr,
                             AS5600_REGISTER_CONF_HIGH, I2C_MEMADD_SIZE_8BIT,
                             a->confRegister, 2) != HAL_OK) {
        status = HAL_ERROR;
        return status;
    }

    /* Write */

    return status;
}

HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef *const a, const float degrees) {
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t pos = 0;
    uint8_t data[2] = {0};

    // Ensure the degrees are within a valid range
    float clamped_degrees = fmodf(degrees, 360.0f); // Wraps degrees to [0, 360)

    // Convert degrees to 12-bit position
    pos = (uint16_t)((clamped_degrees / 360.0f) * 4096.0f);

    // Prepare data to send (split 12-bit value into two 8-bit values)
    data[0] = (uint8_t)((pos & AS5600_12_BIT_MASK) >> 8); // High byte
    data[1] = (uint8_t)pos;                               // Low byte

    // Write the position to the AS5600 ZPOS register
    if (HAL_I2C_Mem_Write_IT(a->i2cHandle, AS5600_SLAVE_ADDRESS << 1, AS5600_REGISTER_ZPOS_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}
HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef *const a,
                                         const uint16_t pos) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data[2] = {0};
    data[0] = (uint8_t)(
        (pos & AS5600_12_BIT_MASK) >>
        8); /* Zero out upper four bits of argument and shift out lower four
               bits */
    data[1] = (uint8_t)pos;
    if (HAL_I2C_Mem_Write_IT(a->i2cHandle, a->i2cAddr,
                             AS5600_REGISTER_MPOS_HIGH, I2C_MEMADD_SIZE_8BIT,
                             data, 2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef *const a,
                                     const uint16_t angle) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data[2] = {0};
    data[0] = (uint8_t)((angle & AS5600_12_BIT_MASK) >>
                        8); /* Zero out upper four bits of
                                argument and shift out lower four
                                bits */
    data[1] = (uint8_t)angle;
    if (HAL_I2C_Mem_Write_IT(a->i2cHandle, a->i2cAddr,
                             AS5600_REGISTER_MANG_HIGH, I2C_MEMADD_SIZE_8BIT,
                             data, 2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_SetPositiveRotationDirection(AS5600_TypeDef *const a,
                                                      const uint8_t dir) {
    HAL_StatusTypeDef status = HAL_OK;
    if (dir == AS5600_DIR_CW) {
        HAL_GPIO_WritePin(a->DirPort, a->DirPin, GPIO_PIN_RESET);
    } else if (dir == AS5600_DIR_CCW) {
        HAL_GPIO_WritePin(a->DirPort, a->DirPin, GPIO_PIN_SET);
    } else {
        /* Invalid rotation direction specified. */
        status = HAL_ERROR;
    }
    return status;
}



HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef *as5600, float *angle) {
    uint8_t data[2];  // To store the raw angle data

    // Read the raw angle register from the AS5600 sensor
    // Raw angle register starts at 0x0C (two bytes)
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(as5600->i2cHandle, AS5600_SLAVE_ADDRESS << 1, AS5600_REGISTER_RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
        // Error handling if the I2C read fails
        return ret;
    }

    // Combine the two 8-bit values into a single 16-bit value
    *angle = ((data[0] << 8) | data[1]);

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef *as5600, float *angle) {
    uint8_t data[2];  // To store the raw angle data

    // Read the raw angle register from the AS5600 sensor
    // Raw angle register starts at 0x0C (two bytes)
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(as5600->i2cHandle, AS5600_SLAVE_ADDRESS << 1, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
        // Error handling if the I2C read fails
        return ret;
    }

    // Combine the two 8-bit values into a single 16-bit value
    *angle = ((data[0] << 8) | data[1]);

    return HAL_OK;
}





