/* Copyright 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Eric Pernia.
 * Copyright 2018, Sergio Alberino
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2020-09-21 */

#ifndef _SAPI_IMU_L3G4200D_H_
#define _SAPI_IMU_L3G4200D_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
// physical constants
#define L3G4200D_G                     9.807f
#define L3G4200D_D2R                   3.14159265359f/180.0f

// L3G4200D registers
#define WHO_AM_I_L3G4200D       	0x0F  // Should return 0xD3
#define WHO_RESPONSE_L3G4200D       0xD3
#define L3G4200D_CTRL_REG1      	0x20
#define L3G4200D_CTRL_REG2      	0x21
#define L3G4200D_CTRL_REG3      	0x22
#define L3G4200D_CTRL_REG4      	0x23
#define L3G4200D_CTRL_REG5      	0x24
#define L3G4200D_REFERENCE      	0x25
#define L3G4200D_OUT_TEMP       	0x26
#define L3G4200D_STATUS_REG     	0x27
#define L3G4200D_OUT_X_L        	0x28
#define L3G4200D_OUT_X_H        	0x29
#define L3G4200D_OUT_Y_L        	0x2A
#define L3G4200D_OUT_Y_H        	0x2B
#define L3G4200D_OUT_Z_L        	0x2C
#define L3G4200D_OUT_Z_H        	0x2D
#define L3G4200D_FIFO_CTRL_REG  	0x2E
#define L3G4200D_FIFO_SRC_REG   	0x2F
#define L3G4200D_INT1_CFG       	0x30
#define L3G4200D_INT1_SRC       	0x31
#define L3G4200D_INT1_TSH_XH    	0x32
#define L3G4200D_INT1_TSH_XL    	0x33
#define L3G4200D_INT1_TSH_YH    	0x34
#define L3G4200D_INT1_TSH_YL    	0x35
#define L3G4200D_INT1_TSH_ZH    	0x36
#define L3G4200D_INT1_TSH_ZL    	0x37
#define L3G4200D_INT1_DURATION  	0x38
#define L3G4200D_CTRL_REG1_INIT		0x8F	// Reg1 Enable Axis reading
#define L3G4200D_CTRL_REG4_INIT		0x08	// Reg4 set gyro  scale

// I2C baudrate
#define L3G4200D_I2C_RATE              400000 // 400 kHz

/*==================[typedef]================================================*/

//Different options for basic L3G4200D setting registers

typedef enum {
   L3G4200D_ADDRESS_0 = 0x68,
   L3G4200D_ADDRESS_1 = 0x69 // Device address when ADO = 0
} L3G4200D_address_t;

typedef enum
{
   L3G4200D_GYRO_RANGE_250DPS,
   L3G4200D_GYRO_RANGE_500DPS,
   L3G4200D_GYRO_RANGE_1000DPS,
   L3G4200D_GYRO_RANGE_2000DPS
} L3G4200D_GyroRange_t;

typedef enum
{
	L3G4200D_ODR_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
	L3G4200D_ODR_100_25,
	L3G4200D_ODR_100_25a,
	L3G4200D_ODR_100_25b,
	L3G4200D_ODR_200_125,
	L3G4200D_ODR_200_25,
	L3G4200D_ODR_200_50,
	L3G4200D_ODR_200_70,
	L3G4200D_ODR_400_20,
	L3G4200D_ODR_400_25,
	L3G4200D_ODR_400_50,
	L3G4200D_ODR_400_110,
	L3G4200D_ODR_800_30,
	L3G4200D_ODR_800_35,
	L3G4200D_ODR_800_50,
	L3G4200D_ODR_800_110  // 800 Hz ODR, 110 Hz bandwidth
} L3G4200D_ODR_t;

//Control structure for L3G4200D operation (only one IMU per project)
typedef struct {
   L3G4200D_address_t address; //L3G4200D address can be configured through AD0 pin

   // scale factors
   float _gyroScale;
   float _tempScale;
   float _tempOffset;
   
   // configuration
   L3G4200D_GyroRange_t     _gyroRange;

   // buffer for reading from sensor
   uint8_t _buffer[7];

   // data buffer
   float _gx, _gy, _gz;
   float _t;

   // gyro bias estimation
   uint8_t _numSamples;
   float _gxb, _gyb, _gzb;


   // data counts
   int16_t _gxcounts, _gycounts, _gzcounts;
   int16_t _tcounts;


   // track success of interacting with sensor
   int8_t _status;

   // track success of interacting with sensor
   int8_t _reg1bits;

   // 16-bit signed gyro sensor output
   int16_t gyroCount[3];

} L3G4200D_control_t;

/*==================[external functions declaration]=========================*/

// Initialize L3G4200D (Only I2C)
int8_t l3g4200dInit( L3G4200D_address_t address );

// Read sensor registers and store data at control structure
bool_t l3g4200dRead(void);

// Returns the gyroscope measurement in the x direction, rad/s
float l3g4200dGetGyroX_dps( void );

// Returns the gyroscope measurement in the y direction, rad/s
float l3g4200dGetGyroY_dps( void );

// Returns the gyroscope measurement in the z direction, rad/s
float l3g4200dGetGyroZ_dps( void );


/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_IMU_L3G4200D_H_ */
