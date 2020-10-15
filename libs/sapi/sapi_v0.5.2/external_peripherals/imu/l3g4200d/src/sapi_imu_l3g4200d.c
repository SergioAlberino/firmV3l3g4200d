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

static int8_t l3g4200dReadRegisters (uint8_t subAddress, uint8_t count )
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
	if (l3g4200dReadRegisters(WHO_AM_I_L3G4200D ,1) < 0) {
		return -1;
	}
	return control._buffer[0];
}

static int8_t l3g4200dCalibrateGyro( void )
{
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
}

static int8_t l3g4200dSetGyroRange( L3G4200D_GyroRange_t range )
{
	switch(range) {
		case L3G4200D_GYRO_RANGE_250DPS: {
		  // setting the gyro range to 250DPS
		  control._gyroScale = 8.75f; // 8.75 mdps per digit for +/-250dps full scale using 16 bit digital output
		  control._reg1bits = 0b00000000;

		  break;
		}
		case L3G4200D_GYRO_RANGE_500DPS: {
		  // setting the gyro range to 500DPS
		  control._gyroScale = 17.50f; // 17.50 mdps per digit for +/-500dps full scale using 16 bit digital output
		  control._reg1bits = 0b00010000;

		  break;
		}
		case L3G4200D_GYRO_RANGE_2000DPS: {
		  // setting the gyro range to 2000DPS
		  control._gyroScale = 70.00f; // 70.00 mdps per digit for +/-2000dps full scale using 16 bit digital output
		  control._reg1bits = 0b00100000;

		  break;
		}
	}
	control._gyroRange = range;
	return 1;
}

/*==================[external functions definition]==========================*/

//Initialize L3G4200D (TODO: include SPI communication)
int8_t l3g4200dInit( L3G4200D_address_t address )
{
	l3g4200dInitializeControlStructure();

	control.address = address;

	// using I2C for communication
	// starting the I2C bus
	i2cInit(I2C0, L3G4200D_I2C_RATE);

	// Startup sequence as AN3393


	//CTRL_REG2 = |0 |0 |HPM1|HPM0|HPCF3|HPCF2|HPCF1|HPCF0|
	//Default =   |0 |0 |0   |0   |0    |0    |0    |0    |
	if (l3g4200dWriteRegister(L3G4200D_CTRL_REG3, 0x00) < 0) {
		return -2;
	}


	//CTRL_REG3 = |I1_Int1|I1_Boot|H_Lactive|PP_OD|I2DRDY|I2_WTM|I2_ORun|I2_Empty|
	//Default =   |0      |0      |0        |0    |0     |0     |0      |0       |
	if (l3g4200dWriteRegister(L3G4200D_CTRL_REG3, 0x08) < 0) {
		return -3;
	}

	// Set Range 250DPS
	l3g4200dSetGyroRange(L3G4200D_GYRO_RANGE_250DPS);
	// Set bits for Reg1 to select Range
	control._reg1bits= (L3G4200D_CTRL_REG4_INIT & 0b11001111) | control._reg1bits;

	//CTRL_REG4 = |BDU|BLE|FS1|FS0| - |ST1|ST0|SIM|
	//Default =   |0  |0  |0  |0  |0  |0  |0  |0  |
	//BDU = 1 => Block Data Update
	//BLE = 0 => Little endian
	//FS = xx => scale selection
	//ST = 000 => Disable Self test
	// Reg4: set gyro  scale
	if (l3g4200dWriteRegister(L3G4200D_CTRL_REG4, control._reg1bits) < 0) {
		return -4;
	}

	// Set ODR and cut-off
	// L3G4200D_ODR_100_125
	control._reg1bits= (L3G4200D_CTRL_REG1_INIT & 0b00001111) | L3G4200D_ODR_100_125 << 4;

	//CTRL_REG1 = |DR1|DR0|BW1|BW0|PD|Zen|Yen|Xen|
	//Default =   |0  |0  |0  |0  |0 |1  |1  |1  |
    //DR => ODR (output data rate)
	//BW => select Range
	//PD = 1 => Normal
	//Zen = Yen = Xen = 1 => Enable
	if (l3g4200dWriteRegister(L3G4200D_CTRL_REG1, control._reg1bits) < 0) {
		return -1;
	}

	//CTRL_REG5 = |BOOT|FIFO_EN| - |HPen|INT1_Sel1|INT1_Sel0|Out_Sel1|Out_Sel0|
	//Default =   |0   |0      |0  |0   |0        |0        |0       |0       |
	// Reg5: Disable FIFO
	if (l3g4200dWriteRegister(L3G4200D_CTRL_REG5, 0x00) < 0) {
		return -5;
	}

	// check the WHO AM I byte, expected value is 0xD3
	l3g4200dWhoAmI();
	if (control._buffer[0] != WHO_RESPONSE_L3G4200D) {
		printf("who am I?    (%i)   [Wrong answer]\r\n",
		control._buffer[0]);
		return -6;
		}

	// successful init, return 1
	return 1;
}


//Read sensor registers and store data at control structure
bool_t l3g4200dRead(void)
{
	static uint8_t aux;

	// reading  the X data from the l3g4200
	if( !l3g4200dReadRegisters (L3G4200D_OUT_X_L,1) ){
			return 0;
	}
	aux= control._buffer[0];

	if( !l3g4200dReadRegisters (L3G4200D_OUT_X_H,1) ){
			return 0;
	}
	  // combine into 16 bit values to obtain the angular velocity
	control._gxcounts  = (((int16_t)control._buffer[0]) << 8)  | aux;


	// reading  the Y data from the l3g4200
	if( !l3g4200dReadRegisters (L3G4200D_OUT_Y_L,2) ){
			return 0;
	}
	aux= control._buffer[0];

	if( !l3g4200dReadRegisters (L3G4200D_OUT_Y_H,1) ){
				return 0;
	}
	// combine into 16 bit values to obtain the angular velocity
	control._gycounts  = (((int16_t)control._buffer[0]) << 8)  | aux;


	// reading  the Z data from the l3g4200
	if( !l3g4200dReadRegisters (L3G4200D_OUT_Z_L,1) ){
			return 0;
	}
	aux= control._buffer[0];

	if( !l3g4200dReadRegisters (L3G4200D_OUT_Z_H,1) ){
			return 0;
	}
	  // combine into 16 bit values to obtain the angular velocity
	control._gzcounts  = (((int16_t)control._buffer[0]) << 8)  | aux;


	// convert count into dps
	control._gx = ((float) control._gxcounts * control._gyroScale)- control._gxb;
	control._gy = ((float) control._gycounts * control._gyroScale)- control._gyb;
	control._gz = ((float) control._gzcounts * control._gyroScale)- control._gzb;

	return 1;
}

// Returns the gyroscope measurement in the x direction, rad/s
float l3g4200dGetGyroX_dps( void )
{
	return control._gx;
}

// Returns the gyroscope measurement in the y direction, rad/s
float l3g4200dGetGyroY_dps( void )
{
	return control._gy;
}

// Returns the gyroscope measurement in the z direction, rad/s
float l3g4200dGetGyroZ_dps( void )
{
	return control._gz;
}


/*==================[end of file]============================================*/
