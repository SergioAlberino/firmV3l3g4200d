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

/*==================[inclusions]=============================================*/

#include "sapi_imu_l3g4200d.h"   /* <= sAPI HMC5883L header */
#include "sapi_i2c.h"           /* <= sAPI I2C header */
#include "sapi_delay.h"         /* <= sAPI Delay header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int8_t l3g4200dInitializeControlStructure( void );
static int8_t l3g4200dWriteRegister( uint8_t subAddress, uint8_t data );
static int8_t l3g4200dReadRegisters( uint8_t subAddress, uint8_t count );
static int8_t l3g4200dWhoAmI( void );


static int8_t l3g4200dCalibrateGyro( void );
static int8_t l3g4200dSetGyroRange( L3G4200D_GyroRange_t range );
static int8_t l3g4200dSetDlpfBandwidth( L3G4200D_DlpfBandwidth_t bandwidth );
static int8_t l3g4200dSetSrd( uint8_t srd );

/*==================[internal data definition]===============================*/

//MPU control structure
static L3G4200D_control_t control;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static int8_t l3g4200dInitializeControlStructure( void )
{
	control._tempScale = 333.87f;
	control._tempOffset = 21.0f;
	control._numSamples = 100;
	control._axs = 1.0f;
	control._ays = 1.0f;
	control._azs = 1.0f;
	control._maxCounts = 1000;
	control._deltaThresh = 0.3f;
	control._coeff = 8;
	control._hxs = 1.0f;
	control._hys = 1.0f;
	control._hzs = 1.0f;
	control.tX[0] = 0;
	control.tX[1] = 1;
	control.tX[2] = 0;
	control.tY[0] = 1;
	control.tY[1] = 0;
	control.tY[2] = 0;
	control.tZ[0] = 0;
	control.tZ[1] = 0;
	control.tZ[2] = -1;
}

static int8_t l3g4200dWriteRegister( uint8_t subAddress, uint8_t data )
{
	uint8_t transmitDataBuffer[2];
	transmitDataBuffer[0] = subAddress;
	transmitDataBuffer[1] = data;
	i2cWrite(I2C0, control.address, transmitDataBuffer, 2, TRUE);

	delay(10);

	/* read back the register */
	l3g4200dReadRegisters(subAddress,1);
	/* check the read back register against the written register */
	if(control._buffer[0] == data) {
      return 1;
	}
	else{
      return -1;
	}
}

static int8_t l3g4200dReadRegisters( uint8_t subAddress, uint8_t count )
{
	if( i2cRead( I2C0,control.address,&subAddress,1,TRUE,control._buffer,count,TRUE) ){
		return 1;
	} else {
		return -1;
	}
}



static int8_t l3g4200dWhoAmI( void )
{
	// read the WHO AM I register
	if (l3g4200dReadRegisters(L3G4200D_WHO_AM_I,1) < 0) {
		return -1;
	}
	// return the register value
	return control._buffer[0];
}



//static int8_t l3g4200dCalibrateGyro( void )
//{
//	// set the range, bandwidth, and srd
//	if (l3g4200dSetGyroRange(L3G4200D_GYRO_RANGE_250DPS) < 0) {
//		return -1;
//	}
//	if (l3g4200dSetDlpfBandwidth(L3G4200D_DLPF_BANDWIDTH_20HZ) < 0) {
//		return -2;
//	}
//	if (l3g4200dSetSrd(19) < 0) {
//		return -3;
//	}
//
//	// take samples and find bias
//	control._gxbD = 0;
//	control._gybD = 0;
//	control._gzbD = 0;
//	for (uint8_t i=0; i < control._numSamples; i++) {
//		l3g4200dRead();
//		control._gxbD += ((l3g4200dGetGyroX_rads() + control._gxb)/control._numSamples);
//		control._gybD += ((l3g4200dGetGyroY_rads() + control._gyb)/control._numSamples);
//		control._gzbD += ((l3g4200dGetGyroZ_rads() + control._gzb)/control._numSamples);
//		delay(20);
//	}
//	control._gxb = (float)control._gxbD;
//	control._gyb = (float)control._gybD;
//	control._gzb = (float)control._gzbD;
//
//	// set the range, bandwidth, and srd back to what they were
//	if (l3g4200dSetGyroRange(control._gyroRange) < 0) {
//		return -4;
//	}
//	if (l3g4200dSetDlpfBandwidth(control._bandwidth) < 0) {
//		return -5;
//	}
//	if (l3g4200dSetSrd(control._srd) < 0) {
//		return -6;
//	}
//	return 1;
//}

//static int8_t l3g4200dSetGyroRange( L3G4200D_GyroRange_t range )
//{
//	switch(range) {
//		case L3G4200D_GYRO_RANGE_250DPS: {
//		  // setting the gyro range to 250DPS
//		  if(l3g4200dWriteRegister(L3G4200D_GYRO_CONFIG, L3G4200D_GYRO_FS_SEL_250DPS) < 0){
//			return -1;
//		  }
//        // setting the gyro scale to 250DPS
//		  control._gyroScale = 250.0f/32767.5f * L3G4200D_D2R;
//		  break;
//		}
//		case L3G4200D_GYRO_RANGE_500DPS: {
//		  // setting the gyro range to 500DPS
//		  if(l3g4200dWriteRegister(L3G4200D_GYRO_CONFIG, L3G4200D_GYRO_FS_SEL_500DPS) < 0){
//			return -1;
//		  }
//        // setting the gyro scale to 500DPS
//		  control._gyroScale = 500.0f/32767.5f * L3G4200D_D2R;
//		  break;
//		}
//		case L3G4200D_GYRO_RANGE_1000DPS: {
//		  // setting the gyro range to 1000DPS
//		  if(l3g4200dWriteRegister(L3G4200D_GYRO_CONFIG, L3G4200D_GYRO_FS_SEL_1000DPS) < 0){
//			return -1;
//		  }
//        // setting the gyro scale to 1000DPS
//		  control._gyroScale = 1000.0f/32767.5f * L3G4200D_D2R;
//		  break;
//		}
//		case L3G4200D_GYRO_RANGE_2000DPS: {
//		  // setting the gyro range to 2000DPS
//		  if(l3g4200dWriteRegister(L3G4200D_GYRO_CONFIG, L3G4200D_GYRO_FS_SEL_2000DPS) < 0){
//			return -1;
//		  }
//        // setting the gyro scale to 2000DPS
//		  control._gyroScale = 2000.0f/32767.5f * L3G4200D_D2R;
//		  break;
//		}
//	}
//	control._gyroRange = range;
//	return 1;
//}


/*==================[external functions definition]==========================*/

//Initialize l3g4200dInit (TODO: include SPI communication)
//int8_t l3g4200dInit( L3G4200D_address_t address ) {
//
//	l3g4200dInitializeControlStructure();
//
//	control.address = address;
//
//	// using I2C for communication
//	// starting the I2C bus
//	i2cInit(I2C0, L3G4200D_I2C_RATE);
//
//	// select clock source to gyro
//	if (l3g4200dWriteRegister(L3G4200D_PWR_MGMNT_1, L3G4200D_CLOCK_SEL_PLL) < 0) {
//		return -1;
//	}
//	// enable I2C master mode
//	if (l3g4200dWriteRegister(L3G4200D_USER_CTRL, L3G4200D_I2C_MST_EN) < 0) {
//		return -2;
//	}
//	// set the I2C bus speed to 400 kHz
//	if (l3g4200dWriteRegister(L3G4200D_I2C_MST_CTRL, L3G4200D_I2C_MST_CLK) < 0) {
//		return -3;
//	}
////
////
////   // setting the gyro scale to 2000DPS
////	control._gyroScale = 2000.0f / 32767.5f * L3G4200D_D2R;
////	control._gyroRange = L3G4200D_GYRO_RANGE_2000DPS;
//
//
//	// enable I2C master mode
//	if (l3g4200dWriteRegister(L3G4200D_USER_CTRL, L3G4200D_I2C_MST_EN) < 0) {
//		return -12;
//	}
//	// set the I2C bus speed to 400 kHz
//	if (l3g4200dWriteRegister(L3G4200D_I2C_MST_CTRL, L3G4200D_I2C_MST_CLK) < 0) {
//		return -13;
//	}
//
//	// estimate gyro bias
//	if (l3g4200dCalibrateGyro() < 0) {
//		return -20;
//	}
//	// successful init, return 1
//	return 1;
//}

////Read sensor registers and store data at control structure
//bool_t l3g4200d0Read(void)
//{
////	// grab the data from the L3G4200D
////	if( !l3g4200dReadRegisters(L3G4200D_ACCEL_OUT, 21) ){
////		return 0;
////	}
////	// combine into 16 bit values
////	control._axcounts = (((int16_t)control._buffer[0]) << 8)  | control._buffer[1];
////	control._aycounts = (((int16_t)control._buffer[2]) << 8)  | control._buffer[3];
////	control._azcounts = (((int16_t)control._buffer[4]) << 8)  | control._buffer[5];
////	control._tcounts  = (((int16_t)control._buffer[6]) << 8)  | control._buffer[7];
////	control._gxcounts = (((int16_t)control._buffer[8]) << 8)  | control._buffer[9];
////	control._gycounts = (((int16_t)control._buffer[10]) << 8) | control._buffer[11];
////	control._gzcounts = (((int16_t)control._buffer[12]) << 8) | control._buffer[13];
////	control._hxcounts = (((int16_t)control._buffer[15]) << 8) | control._buffer[14];
////	control._hycounts = (((int16_t)control._buffer[17]) << 8) | control._buffer[16];
////	control._hzcounts = (((int16_t)control._buffer[19]) << 8) | control._buffer[18];
////	// transform and convert to float values
////	control._ax = (((float)(control.tX[0]*control._axcounts + control.tX[1]*control._aycounts + control.tX[2]*control._azcounts) * control._accelScale) - control._axb)*control._axs;
////	control._ay = (((float)(control.tY[0]*control._axcounts + control.tY[1]*control._aycounts + control.tY[2]*control._azcounts) * control._accelScale) - control._ayb)*control._ays;
////	control._az = (((float)(control.tZ[0]*control._axcounts + control.tZ[1]*control._aycounts + control.tZ[2]*control._azcounts) * control._accelScale) - control._azb)*control._azs;
////	control._gx = ((float) (control.tX[0]*control._gxcounts + control.tX[1]*control._gycounts + control.tX[2]*control._gzcounts) * control._gyroScale) -  control._gxb;
////	control._gy = ((float) (control.tY[0]*control._gxcounts + control.tY[1]*control._gycounts + control.tY[2]*control._gzcounts) * control._gyroScale) -  control._gyb;
////	control._gz = ((float) (control.tZ[0]*control._gxcounts + control.tZ[1]*control._gycounts + control.tZ[2]*control._gzcounts) * control._gyroScale) -  control._gzb;
////	control._hx = (((float)(control._hxcounts) * control._magScaleX) - control._hxb)*control._hxs;
////	control._hy = (((float)(control._hycounts) * control._magScaleY) - control._hyb)*control._hys;
////	control._hz = (((float)(control._hzcounts) * control._magScaleZ) - control._hzb)*control._hzs;
////	control._t = ((((float) control._tcounts)  - control._tempOffset)/ control._tempScale) + control._tempOffset;
////	return 1;
//}


// Returns the gyroscope measurement in the x direction, rad/s
//float l3g4200dGetGyroX_rads( void )
//{
//	return control._gx;
//}
//
//// Returns the gyroscope measurement in the y direction, rad/s
//float L3G4200DGetGyroY_rads( void )
//{
//	return control._gy;
//}
//
//// Returns the gyroscope measurement in the z direction, rad/s
//float L3G4200DGetGyroZ_rads( void )
//{
//	return control._gz;
//}


/*==================[end of file]============================================*/
