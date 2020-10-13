/* Copyright 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Eric Pernia.
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




// Registros heredados
#define L3G4200D_GYRO_OUT              0x43
#define L3G4200D_TEMP_OUT              0x41
#define L3G4200D_EXT_SENS_DATA_00      0x49
#define L3G4200D_ACCEL_CONFIG 	       0x1C
#define L3G4200D_ACCEL_FS_SEL_2G       0x00
#define L3G4200D_ACCEL_FS_SEL_4G       0x08
#define L3G4200D_ACCEL_FS_SEL_8G       0x10
#define L3G4200D_ACCEL_FS_SEL_16G      0x18
#define L3G4200D_GYRO_CONFIG           0x1B
#define L3G4200D_GYRO_FS_SEL_250DPS    0x00
#define L3G4200D_GYRO_FS_SEL_500DPS    0x08
#define L3G4200D_GYRO_FS_SEL_1000DPS   0x10
#define L3G4200D_GYRO_FS_SEL_2000DPS   0x18
#define L3G4200D_ACCEL_CONFIG2         0x1D
#define L3G4200D_ACCEL_DLPF_184        0x01
#define L3G4200D_ACCEL_DLPF_92         0x02
#define L3G4200D_ACCEL_DLPF_41         0x03
#define L3G4200D_ACCEL_DLPF_20         0x04
#define L3G4200D_ACCEL_DLPF_10         0x05
#define L3G4200D_ACCEL_DLPF_5          0x06
#define L3G4200D_CONFIG                0x1A
#define L3G4200D_GYRO_DLPF_184         0x01
#define L3G4200D_GYRO_DLPF_92          0x02
#define L3G4200D_GYRO_DLPF_41          0x03
#define L3G4200D_GYRO_DLPF_20          0x04
#define L3G4200D_GYRO_DLPF_10          0x05
#define L3G4200D_GYRO_DLPF_5           0x06
#define L3G4200D_SMPDIV                0x19
#define L3G4200D_INT_PIN_CFG           0x37
#define L3G4200D_INT_ENABLE            0x38
#define L3G4200D_INT_DISABLE           0x00
#define L3G4200D_INT_PULSE_50US        0x00
#define L3G4200D_INT_WOM_EN            0x40
#define L3G4200D_INT_RAW_RDY_EN        0x01
#define L3G4200D_PWR_MGMNT_1           0x6B
#define L3G4200D_PWR_CYCLE             0x20
#define L3G4200D_PWR_RESET             0x80
#define L3G4200D_CLOCK_SEL_PLL         0x01
#define L3G4200D_PWR_MGMNT_2           0x6C
#define L3G4200D_SEN_ENABLE            0x00
#define L3G4200D_DIS_GYRO              0x07
#define L3G4200D_USER_CTRL             0x6A

#define L3G4200D_ACCEL_INTEL_EN        0x80
#define L3G4200D_ACCEL_INTEL_MODE      0x40
#define L3G4200D_LP_ACCEL_ODR          0x1E
#define L3G4200D_WOM_THR               0x1F
#define L3G4200D_WHO_AM_I              0x75
#define L3G4200D_FIFO_EN               0x23
#define L3G4200D_FIFO_TEMP             0x80
#define L3G4200D_FIFO_GYRO             0x70
#define L3G4200D_FIFO_ACCEL            0x08
#define L3G4200D_FIFO_MAG              0x01
#define L3G4200D_FIFO_COUNT            0x72
#define L3G4200D_FIFO_READ             0x74



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
   L3G4200D_ACCEL_RANGE_2G,
   L3G4200D_ACCEL_RANGE_4G,
   L3G4200D_ACCEL_RANGE_8G,
   L3G4200D_ACCEL_RANGE_16G
} L3G4200D_AccelRange_t;

typedef enum
{
   L3G4200D_GYRO_RANGE_250DPS,
   L3G4200D_GYRO_RANGE_500DPS,
   L3G4200D_GYRO_RANGE_1000DPS,
   L3G4200D_GYRO_RANGE_2000DPS
} L3G4200D_GyroRange_t;

typedef enum
{
   L3G4200D_DLPF_BANDWIDTH_184HZ,
   L3G4200D_DLPF_BANDWIDTH_92HZ,
   L3G4200D_DLPF_BANDWIDTH_41HZ,
   L3G4200D_DLPF_BANDWIDTH_20HZ,
   L3G4200D_DLPF_BANDWIDTH_10HZ,
   L3G4200D_DLPF_BANDWIDTH_5HZ
} L3G4200D_DlpfBandwidth_t;

typedef enum
{
   L3G4200D_LP_ACCEL_ODR_0_24HZ  = 0,
   L3G4200D_LP_ACCEL_ODR_0_49HZ  = 1,
   L3G4200D_LP_ACCEL_ODR_0_98HZ  = 2,
   L3G4200D_LP_ACCEL_ODR_1_95HZ  = 3,
   L3G4200D_LP_ACCEL_ODR_3_91HZ  = 4,
   L3G4200D_LP_ACCEL_ODR_7_81HZ  = 5,
   L3G4200D_LP_ACCEL_ODR_15_63HZ = 6,
   L3G4200D_LP_ACCEL_ODR_31_25HZ = 7,
   L3G4200D_LP_ACCEL_ODR_62_50HZ = 8,
   L3G4200D_LP_ACCEL_ODR_125HZ   = 9,
   L3G4200D_LP_ACCEL_ODR_250HZ   = 10,
   L3G4200D_LP_ACCEL_ODR_500HZ   = 11
} L3G4200D_LpAccelOdr_t;

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

   float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

   // 16-bit signed gyro sensor output
   int16_t gyroCount[3];

} L3G4200D_control_t;

/*==================[external functions declaration]=========================*/

// Initialize L3G4200D (Only I2C)
int8_t l3g4200dInit( L3G4200D_address_t address );

// Read sensor registers and store data at control structure
bool_t l3g4200dRead(void);




// Returns the gyroscope measurement in the x direction, rad/s
float l3g4200dGetGyroX_rads( void );

// Returns the gyroscope measurement in the y direction, rad/s
float l3g4200dGetGyroY_rads( void );

// Returns the gyroscope measurement in the z direction, rad/s
float l3g4200dGetGyroZ_rads( void );


/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_IMU_L3G4200D_H_ */
