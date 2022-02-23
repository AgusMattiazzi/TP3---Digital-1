/*
 * Inicializacion.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Agustin
 */

#ifndef INICIALIZACION_H_
#define INICIALIZACION_H_

// Macros Entrada-Salida
#define LED_VERDE_ON 		GPIOD->PCOR |= (1<<5)
#define LED_VERDE_OFF 		GPIOD->PSOR |= (1<<5)
#define LED_VERDE_TOGGLE 	GPIOD->PTOR |= (1<<5)
#define LED_ROJO_ON 		GPIOE->PCOR |= (1<<29)
#define LED_ROJO_OFF 		GPIOE->PSOR |= (1<<29)
#define LED_ROJO_TOGGLE 	GPIOE->PTOR |= (1<<29)
#define GET_SW1				!(GPIOC->PDIR & (1<<3))
#define GET_SW3				!(GPIOC->PDIR & (1<<12))

// Macros Sensor de Looz
#define LUZ_MAX 		0
#define LUZ_MIN			65535

// Funciones
void Init_IO(){
	// Configuración del Led rojo
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;	// Habilita el clock del puerto E
	// Pagina 216 del manual FRDM-KL46Z

	// El led rojo se ubica en el pin 29 del puerto E
	PORTE->PCR[29] |= 1<<8;		// Asigna funcionalidad GPIO (entrada/salida) al pin E29
	GPIOE->PCOR |= (1<<29);		// Enciende el led rojo
	GPIOE->PDDR |= (1<<29);		// Setea el led rojo como salida
	GPIOE->PSOR |= (1<<29);		// Apaga el led rojo si está encendido

	// Configuración del Led verde
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;// Habilita el clock del puerto D
	// Pagina 216 del manual FRDM-KL46Z

	// El led verde se ubica en el pin 5 del puerto D
	PORTD->PCR[5] |= PORT_PCR_MUX(1);	// Asigna funcionalidad GPIO al pin D5
	GPIOD->PCOR |= (1<<5);				// Enciende el led verde
	GPIOD->PDDR |= (1<<5);				// Setea el led verde como salida
	GPIOD->PSOR |= (1<<5);				// Apaga el led verde si está encendido

	// Configuración del pulsador SW1;
	SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;	// Habilita el clock del puerto C
	// Pagina 216 del manual FRDM-KL46Z

	// El pulsador SW1 se ubica en el pin 3 del puerto C
	PORTC->PCR[3] |= PORT_PCR_MUX(1);		// Asigna funcionalidad GPIO al pin C3
	GPIOC->PDDR &= ~(1<<3);					// Setea el pulsador SW1 como entrada
	PORTC->PCR[3] |= 1<<1;					// Habilito las resistencias internas
	PORTC->PCR[3] |= 1;						// Activo la resistencia pull-up

	// Configuración del pulsador SW3;
	// El pulsador SW3 se ubica en el pin 12 del puerto C
	PORTC->PCR[12] |= PORT_PCR_MUX(1);		// Asigna funcionalidad GPIO al pin C12
	GPIOC->PDDR &= ~(1<<12);				// Setea el pulsador SW3 como entrada
	PORTC->PCR[12] |= 1<<1;					// Habilito las resistencias internas
	PORTC->PCR[12] |= 1; 					// Activo la resistencia pull-up
}

void Init_Interrupciones(){
	// Habilito interrupciones en el pin C3
	PORTC->PCR[3] |= PORT_PCR_IRQC(9);
	// El 10 es para la opción de flanco descendente

	// Habilito interrupciones en el pin C12
	PORTC->PCR[12] |= PORT_PCR_IRQC(9);
	// El 10 es para la opción de flanco descendente

	// Seteo prioridades para los dos pulsadores
	NVIC_SetPriority(PORTC_PORTD_IRQn,0);

	// Habilito el vector de interrupciones
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void Init_CAD(){
	// El sensor de luz se ubica en el pin 22 del puerto E
	// Esto se ve en el esquemático
	PORTE->PCR[22] |= PORT_PCR_MUX(0);		// Activo la función analógica en el pin E22

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;		// Clock al modulo del CAD

	ADC0->CFG1  = ADC_CFG1_ADICLK(1);		// Selecciono el Clock = Bus/2
	ADC0->CFG1 |= ADC_CFG1_MODE(3);			// Resolución de 16 bits (Depende de DIFF)
	ADC0->CFG1 |= ADC_CFG1_ADLSMP_MASK; 	// Tiempo de muestreo largo
	ADC0->CFG1 |= ADC_CFG1_ADIV(3);			// División de ratio igual a 8
	// El resto de bits son 0 debido a que la primera expresión es una asignación directa
	// y coloca un 1 en el primer bit, dejando el resto de los bits del registro en 0
	// El bit 8 es cero y eso implica un consumo normal

	// El puerto E22 está conectado a ADC0_DP3
	// Selecciona el canal 3 como entrada, que recibe los datos desde ADC0_DP3
	ADC0->SC1[0] = ADC_SC1_ADCH(3);

	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;		// Habilita la interrupción del ADC
	ADC0->SC1[0] &= !ADC_SC1_DIFF(1);		// Adquisición simple (DIFF = 0)

	//ADC0->SC2 = ADC_SC2_REFSEL(1);		// Selecciono como tensiones de referencia ALTH y ALTL (esto para el KL43)
	ADC0->SC3 |= ADC_SC3_ADCO_MASK;			// Conversión continua

	// Interrupciones
	NVIC_SetPriority(ADC0_IRQn,0);	// (y acá que onda?)
	NVIC_EnableIRQ(ADC0_IRQn);
}

void Init_TPM0(){
	// Siempre activo los clocks de los leds y pulsadores antes
	// asi que no les voy a dar clock acá
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;		// Clock al modulo del puerto D
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;		// Clock al modulo del puerto E
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;		// Clock al modulo del TPM0

	// Seleccionar la función de los pines para TPM
	// Paginas 172 y 175 del manual de referencia
	PORTD->PCR[5] = PORT_PCR_MUX(4);	// Asigna funcionalidad TPM0:Canal 5 al pin D5
	PORTE->PCR[29] = PORT_PCR_MUX(3);	// Asigna funcionalidad TPM0:Canal 2 al pin E29

	// Programo los canales como output-compare para alternar la salida
	// Pagina 570 del manual de referencia
	TPM0->CONTROLS[5].CnSC |= 0x14;		// Canal 5: Led Verde
	TPM0->CONTROLS[2].CnSC |= 0x14;		// Canal 2: Led Rojo

	// Habilito interrupciones
	TPM0->CONTROLS[5].CnSC |= TPM_CnSC_CHIE_MASK;	// Canal 5: Led Verde
	//TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHIE_MASK;	// Canal 2: Led Rojo

	// Programo valores de cuentas
	TPM0->CONTROLS[5].CnV = 16383;		// Led verde: 1/2 segundo
	TPM0->CONTROLS[2].CnV = 32767;		// Led rojo: 1 segundo

	// Programo el valor de modulo para reiniciar la cuenta
	TPM0->MOD = 32767;		// 1 segundo

	// Esta la usamos para comparar si el contador es igual a la cuenta establecida en "modulo"
	//TPM0->SC |= TPM_SC_TOF_MASK

	TPM0->SC |= TPM_SC_CMOD(1);	// El contador de TPM aumenta con cada pulso de clock
	NVIC_EnableIRQ(TPM0_IRQn);

}

void Init_MCG(){
	MCG->C1 = MCG_C1_IRCLKEN_MASK;	// Activa el MCGIRCLK (clock de referencia interno)
	// Selecciono el reloj de 32KHz para el MCGIRCLK:IRCS de MCG_C2
	// No hace falta hacer nada porque ya está seleccionado
	// Pagina 385 del manual FRDM-KL46Z

	// Selecciono el MCGIRCLK para el módulo TPM: SIM_SOPT2
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC_MASK;

	//MCG->C2 = MCG_C2_IRCS_MASK;
	//MCG->C2 = MCG_C2_LP_MASK;
}

#endif /* INICIALIZACION_H_ */
