// TP 3 - Implementación de Redes de Petri en el KL46Z
// Implementación mediante inspección directa de la red

// Includes
#include <stdio.h>
#include "MKL46Z4.h"
#include "Inicializacion.h"

// Definir Enumeraciones
typedef enum{	// Contiene los valores para activar las entradas del ADC
	Sensor_temp = 26, Sensor_luz = 3, PTE17 = 5, Disable = 31
}Entrada_ADC;

typedef enum{	// Contiene los valores para activar las entradas del ADC
	Off = 0, On = 1
}Estado;

uint16_t Luz, Temp;
int8_t CT10,FT10;

void Petri(){
	// Definición de variables
	int8_t P_1,P_2;		// Entradas
	int8_t T_1, T_2, T_3, T_4, T_5, T_6, T_7, T_8, T_9, T_10, T_11;	// Transiciones
	static Estado Leds = Off;

	// Marcado inicial
	static int8_t Q_1 = 1, Q_2 = 0, Q_3 = 0, Q_4 = 0, Q_5 = 0, Q_6 = 0, Q_7 = 0, Q_8 = 0;
	// Definir lugares en forma estática

	// Adquisición de entradas
	P_1 = GET_SW1;
	P_2 = GET_SW3;

	// Transiciones
	// Transiciones la subred 1
	T_1 = Q_1 && P_1 && !(P_2);
	T_2 = Q_2 && FT10 && !(P_2);
	T_3 = Q_3 && !(FT10);
	T_4 = Q_2 && P_2;
	T_5 = Q_4 && !(P_2);
	// Transiciones la subred 2
	T_6 = Q_1 && P_2 && !(P_1);
	T_7 = Q_5 && FT10 && !(P_1);
	T_8 = Q_6 && !(FT10);
	T_9 = Q_5 && P_1;
	T_10 = Q_7 && !(P_1);
	// Transicion final
	T_11 = Q_8 && !(P_1) && !(P_2);

	// Marcado y desmarcado
	// Subred 1
	if (T_1) 	{Q_1 = 0; Q_2 = 1;}
	if (T_2) 	{Q_2 = 0; Q_3 = 1;}
	if (T_3) 	{Q_3 = 0; Q_8 = 1;}
	if (T_4) 	{Q_2 = 0; Q_4 = 1;}
	if (T_5) 	{Q_4 = 0; Q_8 = 1;}
	// Subred 2
	if (T_6) 	{Q_1 = 0; Q_5 = 1;}
	if (T_7) 	{Q_5 = 0; Q_6 = 1;}
	if (T_8) 	{Q_6 = 0; Q_8 = 1;}
	if (T_9) 	{Q_5 = 0; Q_7 = 1;}
	if (T_10) 	{Q_7 = 0; Q_8 = 1;}
	// Final
	if (T_11) 	{Q_8 = 0; Q_1 = 1;}

	// Salidas
	if (Q_2 || Q_4 || Q_5 || Q_7){
		if (Leds == Off){	// Activa los leds una vez
			// Lanzo una conversion en PTE17
			ADC0->SC1[0] = ADC_SC1_ADCH(PTE17);
			// Habilita las interrupciones en el ADC0
			ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
			Leds = On;
		}
	}
	else{
		if (Leds == On){	// Desactiva los led una vez
			// Deshabilito el ADC hasta la siguiente conversión
			ADC0->SC1[0] = ADC_SC1_ADCH(Disable);
			// También desactivo interrupciones /**/
			Leds = Off;
		}
		LED_VERDE_OFF;
		LED_ROJO_OFF;
	}

	if (Q_2 || Q_5)	CT10 = 1;
	else CT10 = 0;
}

void Temp_10(){
	if(CT10){	// Si el temporizador está activado
		// Activa el contador y selecciona el clock interno
		SysTick->CTRL  = 3;
	}
	else{		// Si no lo está
		FT10 = 0;
		SysTick->VAL   = 0;		// Llevo la cuenta a cero
		SysTick->CTRL  = 0;		// Desactivo el timer
	}
}

// Manejo de interrupciones
void SysTick_Handler(){		// SysTick Handler
	SysTick->CTRL;			// Limpio la flag del timer
	if(!FT10) FT10 = 1;		// Activo la bandera del temporizador
}

// Función que maneja las interrupciones en el ADC0
void ADC0_IRQHandler(){	// (no se le puede cambiar el nombre)
	static Entrada_ADC Conversion = PTE17;

	// Selección de canal
	if(Conversion == Sensor_luz){	// Conversion : Sensor_luz
		// Leo el valor (esto también limpia la flag de interrupción del ADC)
		TPM0->CONTROLS[2].CnV = ADC0->R[0];	// Led rojo

		if (ADC0->R[0] < MAX_LUZ_ADC){	// No supera el rango superior
			TPM0->CONTROLS[2].CnV = 0;
		}
		else TPM0->CONTROLS[2].CnV = K_LUZ * ADC0->R[0] - MIN_LUZ_ADC;	// Led verde

		Luz = ADC0->R[0];
		Conversion = PTE17;	// Alterno canal
	}
	else{	// Conversion: PTE17
		// Leo el valor (esto también limpia la flag de interrupción del ADC)
		if (ADC0->R[0] > MAX_TEMP_ADC){
			TPM0->CONTROLS[5].CnV = 0;
		}
		else if (ADC0->R[0] < MIN_TEMP_ADC){
			TPM0->CONTROLS[5].CnV = MAX_VAL;
		}
		else TPM0->CONTROLS[5].CnV = K_TEMP*(MAX_TEMP_ADC - ADC0->R[0]);	// Led verde

		Temp = K_TEMP*(MAX_TEMP_ADC - ADC0->R[0]);
		Conversion = Sensor_luz;	// Alterno canal
	}

	// Lanza conversión
	ADC0->SC1[0] = ADC_SC1_ADCH(Conversion);	// Lanzo una conversion con el canal seleccionado
	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;			// Habilita las interrupciones en el ADC0
	// Como no está activado el modo continuo, esto se hace cada
	// vez que lanzamos una conversion

}

int main(void) {
	Init_IO();				// Inicializado de pines
	Init_Systick(10);		// Inicializado del Systick (Este para el petri)
	Init_CAD();				// Inicializado del ADC
	Init_MCG();				// Habilita el generador de clock multipropósito
	Init_TPM0();			// Inicializado del Timer/PWM

	Petri();

	// Lazo infinito
	for(;;){
		Petri();		// Ejecución de la red de petri
		Temp_10();		// Actualización del estado del temporizador
	}

	return 0 ;
}
