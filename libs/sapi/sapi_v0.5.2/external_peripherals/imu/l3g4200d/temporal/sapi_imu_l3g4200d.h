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

/* Date: 2018-07-06 */

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
#define L3G4200D_GYRO_OUT              0x43
#define L3G4200D_GYRO_CONFIG           0x1B
#define L3G4200D_GYRO_FS_SEL_250DPS    0x00
#define L3G4200D_GYRO_FS_SEL_500DPS    0x08
#define L3G4200D_GYRO_FS_SEL_1000DPS   0x10
#define L3G4200D_GYRO_FS_SEL_2000DPS   0x18

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
#define L3G4200D_I2C_MST_EN            0x20
#define L3G4200D_I2C_MST_CLK           0x0D
#define L3G4200D_I2C_MST_CTRL          0x24
#define L3G4200D_I2C_SLV0_ADDR         0x25
#define L3G4200D0_I2C_SLV0_REG          0x26
#define L3G4200D_I2C_SLV0_DO           0x63
#define L3G4200D_I2C_SLV0_CTRL         0x27
#define L3G4200D_I2C_SLV0_EN           0x80
#define L3G4200D_I2C_READ_FLAG         0x80
#define L3G4200D_MOT_DETECT_CTRL       0x69
#define L3G4200D_ACCEL_INTEL_EN        0x80
#define L3G4200D_ACCEL_INTEL_MODE      0x40
#define L3G4200D_LP_ACCEL_ODR          0x1E
#define L3G4200D_WOM_THR               0x1F
#define L3G4200D_WHO_AM_I              0x75
#define L3G4200D_FIFO_EN               0x23
#define L3G4200D_FIFO_GYRO             0x70
#define L3G4200D_FIFO_COUNT            0x72
#define L3G4200D_FIFO_READ             0x74


// I2C baudrate
#define L3G4200D_I2C_RATE              400000 // 400 kHz

/*==================[typedef]================================================*/

//Different options for basic MPU9250 setting registers

typedef enum {
	L3G4200D_ADDRESS_0 = 0xD1,
	L3G4200D_ADDRESS_1 = 0xD3
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
	L3G4200D_DLPF_BANDWIDTH_184HZ,
	L3G4200D_DLPF_BANDWIDTH_92HZ,
	L3G4200D_DLPF_BANDWIDTH_41HZ,
	L3G4200D_DLPF_BANDWIDTH_20HZ,
	L3G4200D_DLPF_BANDWIDTH_10HZ,
	L3G4200D_DLPF_BANDWIDTH_5HZ
} L3G4200D_DlpfBandwidth_t;


//Control structure for MPU9250 operation (only one IMU per project)
typedef struct {
	L3G4200D_address_t address; //L3G4200D address can be configured through AD0 pin
   
   // scale factors
   float _accelScale;
   float _gyroScale;
   float _magScaleX;
   float _magScaleY;
   float _magScaleZ;
   float _tempScale;
   float _tempOffset;
   
   // configuration
   L3G4200D_GyroRange_t     _gyroRange;
   L3G4200D_DlpfBandwidth_t _bandwidth;
   uint8_t _srd;

   // buffer for reading from sensor
   uint8_t _buffer[21];

   // data buffer
   float _ax, _ay, _az;
   float _gx, _gy, _gz;
   float _hx, _hy, _hz;
   float _t;

   // gyro bias estimation
   uint8_t _numSamples;
   double _gxbD, _gybD, _gzbD;
   float _gxb, _gyb, _gzb;

   // accel bias and scale factor estimation
   double _axbD, _aybD, _azbD;
   float _axmax, _aymax, _azmax;
   float _axmin, _aymin, _azmin;
   float _axb, _ayb, _azb;
   float _axs;
   float _ays;
   float _azs;

   // magnetometer bias and scale factor estimation
   uint16_t _maxCounts;
   float _deltaThresh;
   uint8_t _coeff;
   uint16_t _counter;
   float _framedelta, _delta;
   float _hxfilt, _hyfilt, _hzfilt;
   float _hxmax, _hymax, _hzmax;
   float _hxmin, _hymin, _hzmin;
   float _hxb, _hyb, _hzb;
   float _hxs;
   float _hys;
   float _hzs;
   float _avgs;

   // data counts
   int16_t _axcounts, _aycounts, _azcounts;
   int16_t _gxcounts, _gycounts, _gzcounts;
   int16_t _hxcounts, _hycounts, _hzcounts;
   int16_t _tcounts;

   // transformation matrix
   /* transform the accel and gyro axes to match the magnetometer axes */
   int16_t tX[3];
   int16_t tY[3];
   int16_t tZ[3];

   // track success of interacting with sensor
   int8_t _status;

} L3G4200D_control_t;

/*==================[external functions declaration]=========================*/

// Initialize L3G4200D (Only I2C)
int8_t l3g4200dInit( L3G4200D_address_t address );


// Read sensor registers and store data at control structure
bool_t L3G4200DRead(void);


// Returns the gyroscope measurement in the x direction, rad/s
float l3g4200dGetGyroX_rads( void );

// Returns the gyroscope measurement in the y direction, rad/s
float L3G4200DGetGyroY_rads( void );

// Returns the gyroscope measurement in the z direction, rad/s
float L3G4200DGetGyroZ_rads (void );



/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_IMU_L3G4200D_H_ */
