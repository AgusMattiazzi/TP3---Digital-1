/*
 * Inicializacion.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Agustin
 */

#ifndef INICIALIZACION_H_
#define INICIALIZACION_H_

// Macros -------------------------------------------------------------------------------------------- /
#define MAX_VAL 1023
#define LED_ROJO_ON		TPM0->CONTROLS[2].CnV = MAX_VAL	// Low-true pulses
#define LED_ROJO_OFF 	TPM0->CONTROLS[2].CnV = 0

#define LED_VERDE_ON 	TPM0->CONTROLS[5].CnV = MAX_VAL	// Low-true pulses
#define LED_VERDE_OFF	TPM0->CONTROLS[5].CnV = 0

// Sensor de temperatura
#define MAX_VOLT 3.3

#define MIN_TEMP 25	// 25 °C
#define MAX_TEMP 50	// 50 °C

#define MAX_TEMP_ADC	MAX_TEMP*MAX_VAL/(100*MAX_VOLT)
#define MIN_TEMP_ADC	MIN_TEMP*MAX_VAL/(100*MAX_VOLT)
#define K_TEMP 			MAX_VAL/(MAX_TEMP_ADC - MIN_TEMP_ADC)

// Sensor de looz
#define MIN_LUZ_ADC 	MAX_VAL
#define MAX_LUZ_ADC 	3*MAX_VAL/4
#define K_LUZ			MIN_LUZ_ADC/(MIN_LUZ_ADC - MAX_LUZ_ADC)
// El sensor de luz da mayores valores de voltaje a menor iluminación
// Lógica inversa?

// Entrada-Salida digital
//#define LED_VERDE_ON 		GPIOD->PCOR |= (1<<5)
//#define LED_VERDE_OFF 		GPIOD->PSOR |= (1<<5)
//#define LED_VERDE_TOGGLE 	GPIOD->PTOR |= (1<<5)
//#define LED_ROJO_ON 		GPIOE->PCOR |= (1<<29)
//#define LED_ROJO_OFF 		GPIOE->PSOR |= (1<<29)
//#define LED_ROJO_TOGGLE 	GPIOE->PTOR |= (1<<29)
#define GET_SW1				!(GPIOC->PDIR & (1<<3))
#define GET_SW3				!(GPIOC->PDIR & (1<<12))
// --------------------------------------------------------------------------------------------------- /

// Entrada-Salida Digital ---------------------------------------------------------------------------- /

// Inicialización
void Init_IO(){
	// Configuración del Led rojo
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;	// Habilita el clock del puerto E (Pag 216)

	// El led rojo se ubica en el pin 29 del puerto E
	PORTE->PCR[29] |= PORT_PCR_MUX(1);	// Asigna funcionalidad GPIO (entrada/salida) al pin E29
	GPIOE->PDDR |= (1<<29);				// Setea el led rojo como salida
	GPIOE->PSOR |= (1<<29);				// Apaga el led rojo si está encendido

	// Configuración del Led verde
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;// Habilita el clock del puerto D (Pag 216)

	// El led verde se ubica en el pin 5 del puerto D
	PORTD->PCR[5] |= PORT_PCR_MUX(1);	// Asigna funcionalidad GPIO al pin D5
	GPIOD->PDDR |= (1<<5);				// Setea el led verde como salida
	GPIOD->PSOR |= (1<<5);				// Apaga el led verde si está encendido

	// Configuración del pulsador SW1;
	SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;	// Habilita el clock del puerto C (Pag 216)

	// El pulsador SW1 se ubica en el pin 3 del puerto C
	PORTC->PCR[3] |= PORT_PCR_MUX(1);		// Asigna funcionalidad GPIO al pin C3
	GPIOC->PDDR &= ~(1<<3);					// Setea el pulsador SW1 como entrada
	PORTC->PCR[3] |= PORT_PCR_PE_MASK;		// Habilito las resistencias internas
	PORTC->PCR[3] |= PORT_PCR_PS_MASK;		// Activo la resistencia pull-up

	// Configuración del pulsador SW3;
	// El pulsador SW3 se ubica en el pin 12 del puerto C
	PORTC->PCR[12] |= PORT_PCR_MUX(1);		// Asigna funcionalidad GPIO al pin C12
	GPIOC->PDDR &= ~(1<<12);				// Setea el pulsador SW3 como entrada
	PORTC->PCR[12] |= PORT_PCR_PE_MASK;		// Habilito las resistencias internas
	PORTC->PCR[12] |= PORT_PCR_PS_MASK; 	// Activo la resistencia pull-up
}
// --------------------------------------------------------------------------------------------------- /

// Interrupciones - Puertos C y D -------------------------------------------------------------------- /
void Init_Interrupciones(){		// Pagina 56
	// Habilito interrupciones en el pin C3
	PORTC->PCR[3] |= PORT_PCR_IRQC(9);
	// El 10 es para la opción de flanco descendente y el 9 para el ascendente

	// Habilito interrupciones en el pin C12
	PORTC->PCR[12] |= PORT_PCR_IRQC(9);
	// El 10 es para la opción de flanco descendente y el 9 para el ascendente

	// Seteo prioridades para los puertos C y D
	NVIC_SetPriority(PORTC_PORTD_IRQn,0);

	// Habilito el vector de interrupciones
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}
// --------------------------------------------------------------------------------------------------- /

// Conversor Analógico-Digital ----------------------------------------------------------------------- /

// Inicializacion
void Init_CAD(){
	// El sensor de luz se ubica en el pin 22 del puerto E
	// Esto se ve en el esquemático
	PORTE->PCR[22] |= PORT_PCR_MUX(0);		// Activo la función analógica en el pin E22 (Pag 172)

	PORTE->PCR[17] |= PORT_PCR_MUX(0);		// Activo la función analógica en el pin E17 (Pag 172)

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;		// Clock al modulo del CAD

	// Registro de configuración del ADC (Pag 480)
	ADC0->CFG1 = ADC_CFG1_ADICLK(1);		// Selecciono el Clock = Bus/2
	ADC0->CFG1 |= ADC_CFG1_MODE(2);			// Resolución de 10 bits (Depende de DIFF)
	ADC0->CFG1 |= ADC_CFG1_ADLSMP_MASK; 	// Tiempo de muestreo largo
	ADC0->CFG1 |= ADC_CFG1_ADIV(3);			// División de ratio igual a 8
	// El resto de bits son 0 debido a que la primera expresión es una asignación directa
	// y coloca un 1 en el primer bit, dejando el resto de los bits del registro en 0
	// El bit 8 es cero y eso implica un consumo normal

	// El puerto E22 está conectado a ADC0_DP3
	// Selecciona el canal 3 como entrada, que recibe los datos desde ADC0_DP3 (Pag 82)
	//ADC0->SC1[0] = ADC_SC1_ADCH(3);

	// El puerto E17 está conectado a ADC0_DP3
	// Selecciona el canal 5 como entrada, que recibe los datos desde ADC0_SE5a (Pag 82)
	ADC0->SC1[0] = ADC_SC1_ADCH(5);

	// Selecciono como entrada el sensor de temperatura interno
	//ADC0->SC1[0] = ADC_SC1_ADCH(0x1A);

	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;		// Habilita la interrupción del ADC
	ADC0->SC1[0] &= !ADC_SC1_DIFF(1);		// Adquisición simple (DIFF = 0)

	//ADC0->SC2 = ADC_SC2_REFSEL(1);		// Selecciono como tensiones de referencia ALTH y ALTL (esto para el KL43)
	//ADC0->SC3 |= ADC_SC3_ADCO_MASK;			// Conversión continua

	// Interrupciones
	NVIC_EnableIRQ(ADC0_IRQn);		// Habilito interrupciones en el ADC0
	NVIC_SetPriority(ADC0_IRQn,0);	// Seteo prioridad
}
// --------------------------------------------------------------------------------------------------- /

// Temporizador / Modulo PWM ------------------------------------------------------------------------- /
// Inicialización
void Init_TPM0(){
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;		// Clock al modulo del puerto D
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;		// Clock al modulo del puerto E
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;		// Clock al modulo del TPM0

	// Seleccionar la función de los pines para TPM
	// Paginas 172 y 175 del manual de referencia
	PORTD->PCR[5] = PORT_PCR_MUX(4);	// Asigna funcionalidad TPM0:Canal 5 al pin D5
	PORTE->PCR[29] = PORT_PCR_MUX(3);	// Asigna funcionalidad TPM0:Canal 2 al pin E29

	// Programo los canales como PWM, alineado a izquierda con pulsos ascendentes
	// Pagina 570 del manual de referencia
	TPM0->CONTROLS[5].CnSC |= 0x24;		// Canal 5: Led Verde. 	Low-True pulses
	TPM0->CONTROLS[2].CnSC |= 0x24;		// Canal 2: Led Rojo. 	Low-True pulses

	// Habilito interrupciones
	//TPM0->CONTROLS[5].CnSC |= TPM_CnSC_CHIE_MASK;	// Canal 5: Led Verde
	//TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHIE_MASK;	// Canal 2: Led Rojo

	// Programo valores de cuentas
	TPM0->CONTROLS[5].CnV = 0;		// Arranco con el led verde apagado
	TPM0->CONTROLS[2].CnV = 0;		// Led rojo: 1 segundo

	// Programo el valor de modulo para reiniciar la cuenta
	TPM0->MOD = MAX_VAL;		// Podria convenir el 4095 para  bits

	TPM0->SC |= TPM_SC_CMOD(1);	// El contador de TPM aumenta con cada pulso de clock
	// NVIC_EnableIRQ(TPM0_IRQn);
}
// --------------------------------------------------------------------------------------------------- /

// Temporizador Systick ------------------------------------------------------------------------------ /
// Macros
#define SysTick_1seg 1310720

// Inicializacion
void Init_Systick(float segundos){
	SysTick->LOAD  = SysTick_1seg*segundos;	// Cuenta final
	SysTick->VAL   = 0;			// Inicializo la cuenta
	SysTick->CTRL  = 1;			// Activa el contador
	//SysTick->CTRL  |= 	SysTick_CTRL_TICKINT_Msk;		// Habilita interrupciones?
	// NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
}
// ------------------------------------------------------------------------------------------------------ /

void Init_MCG(){
	MCG->C1 = MCG_C1_IRCLKEN_MASK;	// Activa el MCGIRCLK (clock de referencia interno)
	// Por defecto está seleccionado el reloj de 32KHz para el MCGIRCLK
	// Pagina 385 del manual FRDM-KL46Z

	// Selecciono el reloj de 4 MHz para el MCGIRCLK
	MCG->C2 = MCG_C2_IRCS_MASK;

	MCG->SC = MCG_SC_FCRDIV(3);	/* Factor de division de 16 (F = 250KHz)*/

	// Selecciono el MCGIRCLK para el módulo TPM: SIM_SOPT2
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC_MASK;
}

#endif /* INICIALIZACION_H_ */
