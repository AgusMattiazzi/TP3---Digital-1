/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKL46Z4_Project.c
 * @brief   Application entry point.
 */

// Práctica 4 - Ejercicio 11

// Includes
#include <stdio.h>
#include "MKL46Z4.h"
#include "Inicializacion.h"

// Función que maneja las interrupciones en el TPM0
void TPM0_IRQHandler(){	// No se le puede cambiar el nombre
	TPM0->CONTROLS[5].CnSC |= TPM_CnSC_CHF_MASK;	// Al escribir un 1 limpia la bandera IF

	if(TPM0->CONTROLS[5].CnV == 16383){		// Si alcanza 1/2 segundo
		// Setea la cuenta para que haga toggle de nuevo en 1 segundo
		TPM0->CONTROLS[5].CnV = 32767;
	}
	// Cuando alcanza 1 segundo setea el valor de cuenta al original para reiniciar el ciclo
	else TPM0->CONTROLS[5].CnV = 16383;
}

int main(void) {

	Init_IO();			// Inicializado de pines
	// Init_CAD();		// Inicializado del conversor Analógico-Digital
	Init_MCG();			// Iniciar Reloj
	Init_TPM0();		// Inicializado del modulo Timer/PWM

	// Lazo infinito
	for(;;){
	}

	return 0 ;
}




