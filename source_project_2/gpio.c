#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#define RED_LED8 11 // PortC Pin 11
#define GREEN_LED1 30 // PortE Pin 30
#define GREEN_LED2 29 // PortE Pin 29
#define GREEN_LED3 3 // PortE Pin 3
#define GREEN_LED4 22 // PortE Pin 22
#define GREEN_LED5 21 // PortE Pin 21
#define GREEN_LED6 20 // PortE Pin 20
#define GREEN_LED7 5 // PortE Pin 5
#define GREEN_LED8 4 // PortE Pin 4
#define MASK(x) (1 << (x))

void turnOffGreen() {
	PTE->PCOR |= (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) |	MASK(GREEN_LED4) |
					MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8) );
}

void turnOffRed() {
	PTC->PCOR |= MASK(RED_LED8);
}

void turnOnGreen(void) {
	PTE->PDOR = (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) |	MASK(GREEN_LED4) |
					MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8) );
}

void toggleRed() {
	PTC->PTOR |= MASK(RED_LED8);
}

void sequenceGreen(uint8_t led_count) {
	switch (led_count) {
		case 0: PTE->PDOR = MASK(GREEN_LED1);
		break;
		case 1: PTE->PDOR = MASK(GREEN_LED2);
		break;
		case 2: PTE->PDOR = MASK(GREEN_LED3);
		break;
		case 3: PTE->PDOR = MASK(GREEN_LED4);
		break;
		case 4: PTE->PDOR = MASK(GREEN_LED5);
		break;
		case 5: PTE->PDOR = MASK(GREEN_LED6);
		break;
		case 6: PTE->PDOR = MASK(GREEN_LED7);
		break;
		case 7: PTE->PDOR = MASK(GREEN_LED8);
		break;
	}
}

void moving_led_display(void) {
	static uint8_t count = 0;
	toggleRed();
	sequenceGreen(count);
	osDelay(500);
	count++;
	count %= 8;
}

void static_led_display(void){
	toggleRed();
	turnOnGreen();
	osDelay(250);				
}

void blink_twice(void) {
	turnOnGreen();
	osDelay(1000);
	turnOffGreen();
	osDelay(1000);
	turnOnGreen();
	osDelay(1000);
	turnOffGreen();
	osDelay(1000);
}

//enum color_t{RED,GREEN,BLUE};
void initGPIO(void) {
		// Enable Clock to PORTB and PORTD
		SIM->SCGC5 |= ((SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTE_MASK));
		// Configure MUX settings to make all 3 pins GPIO
		PORTC->PCR[RED_LED8] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED8] |= PORT_PCR_MUX(1);
		
		PORTE->PCR[GREEN_LED1] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED1] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED2] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED2] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED3] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED3] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED4] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED4] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED5] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED5] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED6] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED6] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED7] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED7] |= PORT_PCR_MUX(1);
		PORTE->PCR[GREEN_LED8] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[GREEN_LED8] |= PORT_PCR_MUX(1);

		
		// Set Data Direction Registers for PortB and PortD
		PTE->PDDR |= (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) |	MASK(GREEN_LED4) |
					MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8) );
		PTC->PDDR |= MASK(RED_LED8);
		turnOffGreen();
		turnOffRed();
}
