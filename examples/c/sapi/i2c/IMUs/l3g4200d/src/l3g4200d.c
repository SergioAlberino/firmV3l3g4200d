/* Copyright 2016, Alejandro Permingeat.
 * Copyright 2016, Eric Pernia.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Sergio Alberino
 *
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

/* Date: 2020-10-10 */

/*==================[inclusions]=============================================*/

#include "sapi.h"               // <= sAPI header

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

// L3G4200D Address
L3G4200D_address_t addr = L3G4200D_ADDRESS_1; // If MPU9250 AD0 pin is connected to GND

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void) {
	/* ------------- INICIALIZACIONES ------------- */

	boardConfig();

	//  Gyro Init
	printf("Inicializando L3G4200D...\r\n");
	int8_t status;
	status = l3g4200dInit(addr);

	if (status < 0) {
		printf("L3G4200D no inicializado, chequee las conexiones:\r\n\r\n");
		printf("L3G4200D ---- EDU-CIAA-NXP\r\n\r\n");
		printf("    VCC ---- 3.3V\r\n");
		printf("    GND ---- GND\r\n");
		printf("    SCL ---- SCL\r\n");
		printf("    SDA ---- SDA\r\n");
		//printf("    AD0 ---- GND\r\n\r\n"); // not needed in GY-80
		printf("Se detiene el programa.\r\n");
		printf("Error (%i) \r\n",status) ;

		while (1)
			;
	}
	printf("L3G4200D inicializado correctamente.\r\n\r\n");

	/* ------------- REPETIR POR SIEMPRE ------------- */
	while (TRUE) {

		//Leer el sensor y guardar en estructura de control
		l3g4200dRead();

		// Imprimir resultados

		printf("Giroscopo:      (%f, %f, %f)   [dps]\r\n",
				l3g4200dGetGyroX_dps(),  l3g4200dGetGyroY_dps(),
				l3g4200dGetGyroZ_dps());

		delay(1000);
	}

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	 por ningun S.O. */
	return 0;
}

/*==================[end of file]============================================*/
