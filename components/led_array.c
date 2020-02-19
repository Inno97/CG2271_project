#include "MKL25Z4.h"
#include "stdio.h"

#define RED_LED1 7 // PortC Pin 7
#define RED_LED2 0 // PortC Pin 0
#define RED_LED3 3 // PortC Pin 3
#define RED_LED4 4 // PortC Pin 4
#define RED_LED5 5 // PortC Pin 5
#define RED_LED6 6 // PortC Pin 6
#define RED_LED7 10 // PortC Pin 10
#define RED_LED8 11 // PortC Pin 11
#define GREEN_LED1 30 // PortE Pin 30
#define GREEN_LED2 29 // PortE Pin 29
#define GREEN_LED3 23 // PortE Pin 23
#define GREEN_LED4 22 // PortE Pin 22
#define GREEN_LED5 21 // PortE Pin 21
#define GREEN_LED6 20 // PortE Pin 20
#define GREEN_LED7 5 // PortE Pin 5
#define GREEN_LED8 4 // PortE Pin 4

#define MASK(x) (1 << (x))

int moving_state = 0;

// Set 1 to 8, 0 for all
void setRedLed(int ledNum) {
		switch(ledNum) {
				case 0:
						PTC->PDOR |= (MASK(RED_LED1) | MASK(RED_LED2) | MASK(RED_LED3) | MASK(RED_LED4) |
													MASK(RED_LED5) | MASK(RED_LED6) | MASK(RED_LED7) | MASK(RED_LED8));
						break;
				case 1:
						PTC->PDOR |= MASK(RED_LED1);
						break;
				case 2:
						PTC->PDOR |= MASK(RED_LED2);
						break;
				case 3:
						PTC->PDOR |= MASK(RED_LED3);
						break;
				case 4:
						PTC->PDOR |= MASK(RED_LED4);
						break;
				case 5:
						PTC->PDOR |= MASK(RED_LED5);
						break;
				case 6:
						PTC->PDOR |= MASK(RED_LED6);
						break;
				case 7:
						PTC->PDOR |= MASK(RED_LED7);
						break;
				case 8:
						PTC->PDOR |= MASK(RED_LED8);
						break;
		}
}

// Set 1 to 8, 0 for all
void setGreenLed(int ledNum) {
		switch(ledNum) {
				case 0:
						PTE->PDOR |= (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) | MASK(GREEN_LED4) |
													MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8));
						break;
				case 1:
						PTE->PDOR |= MASK(GREEN_LED1);
						break;
				case 2:
						PTE->PDOR |= MASK(GREEN_LED2);
						break;
				case 3:
						PTE->PDOR |= MASK(GREEN_LED3);
						break;
				case 4:
						PTE->PDOR |= MASK(GREEN_LED4);
						break;
				case 5:
						PTE->PDOR |= MASK(GREEN_LED5);
						break;
				case 6:
						PTE->PDOR |= MASK(GREEN_LED6);
						break;
				case 7:
						PTE->PDOR |= MASK(GREEN_LED7);
						break;
				case 8:
						PTE->PDOR |= MASK(GREEN_LED8);
						break;
		}
}

void setRedLedOff() {
		PTC->PDOR &= (~MASK(RED_LED1) & ~MASK(RED_LED2) & ~MASK(RED_LED3) & ~MASK(RED_LED4) &
									~MASK(RED_LED5) & ~MASK(RED_LED6) & ~MASK(RED_LED7) & ~MASK(RED_LED8));
}

void setGreenLedOff() {
		PTE->PDOR &= (~MASK(GREEN_LED1) & ~MASK(GREEN_LED2) & ~MASK(GREEN_LED3) & ~MASK(GREEN_LED4) &
									~MASK(GREEN_LED5) & ~MASK(GREEN_LED6) & ~MASK(GREEN_LED7) & ~MASK(GREEN_LED8));
}

void InitGPIO(void) {
		// Enable Clock to PORTC and PORTE
		SIM->SCGC5 |= ((SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTE_MASK));
	
		// Configure MUX settings to make all pins GPIO
		// RED LEDs
		PORTC->PCR[RED_LED1] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED1] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED2] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED2] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED3] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED3] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED4] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED4] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED5] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED5] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED6] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED6] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED7] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED7] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTC->PCR[RED_LED8] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[RED_LED8] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		
		// GREEN LEDS
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
	
		// Set Data Direction Registers for PortC and PortE
		PTC->PDDR |= (MASK(RED_LED1) | MASK(RED_LED2) | MASK(RED_LED3) | MASK(RED_LED4) |
									MASK(RED_LED5) | MASK(RED_LED6) | MASK(RED_LED7) | MASK(RED_LED8));
									
		PTE->PDDR |= (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) | MASK(GREEN_LED4) |
									MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8));
		
	  // Set LED to off
		setRedLedOff();
		setGreenLedOff();
}

void delay() {
		for (int i = 0; i < 1000000; ++i);
}

int main() {
		SystemCoreClockUpdate();
		InitGPIO();
	
		int counter = 1;
		
		while (1) {
				delay();
				setRedLedOff();
				setGreenLedOff();
				counter++;
				counter %= 8;
				delay();
				setRedLed(counter + 1);
				setGreenLed(counter + 1);
		}
		
		
}
