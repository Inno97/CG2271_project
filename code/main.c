/**
 * main.c
 * CG2271 Project
 * 
 * Main C file
 * Maybe one day we'll break up everything into proper header files
 */
 
 /**
	* Outstanding Problems:
  *  - Fix the UART2 somehow
	*  - Implement the motor controls
  */

#include<MKL25Z4.h>
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define LED_RED 2 // 0b00000010
#define LED_MASK(x) (x & 0x06)
#define BIT0_MASK(x) (x & 0x01)

#define RED_LED 18 //PORTB Pin18
#define GREEN_LED 19 //PORTB Pin19
#define BLUE_LED 1 //PORTD Pin1
#define MASK(x) (1 << (x))

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3

#define LEFT_MOTOR_COMP 100 // Left Motor Compensation (Out of 100)
#define RIGHT_MOTOR_COMP 100 // Right Motor Compensation

/* Global Variables */
volatile uint8_t rx_data = 0x01;
volatile int led_color = 0;

volatile int left_dir = 0; // 0 - forward, 1 - backward
volatile int right_dir = 0;

/* PWM */
void InitPWM(void) {
		SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
		
		PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
		
		PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);	
		
		PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
		
		PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
		SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

		TPM1->MOD = 7499;

		TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

		TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
		TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
		TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM2->MOD = 7499;
		
		TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

		TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
		TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
		TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

/* Motor Base Functions */
void setLeftSpeed(int percentage) {
		if (left_dir == 0) {
				TPM2_C0V = (int) (((percentage * LEFT_MOTOR_COMP) / 10000.0) * 7499.0);
				TPM2_C1V = 0;
		} else {
				TPM2_C0V = 0;
				TPM2_C1V = (int) (((percentage * LEFT_MOTOR_COMP) / 10000.0) * 7499.0);
		}
}

void setRightSpeed(int percentage) {
		if (right_dir == 0) {
				TPM1_C0V = (int) (((percentage * RIGHT_MOTOR_COMP)/ 10000.0) * 7499.0);
				TPM1_C1V = 0;
		} else {
				TPM1_C0V = 0;
				TPM1_C1V = (int) (((percentage * RIGHT_MOTOR_COMP) / 10000.0) * 7499.0);
		}
}

void switchDir(void) {
		left_dir = (left_dir == 0 ? 1 : 0);
		right_dir = (right_dir == 0 ? 1 : 0);
}

// stop all PWM value
void stop() {	
		TPM1_C0V = 0;
		TPM1_C1V = 0;
		TPM2_C0V = 0;
		TPM2_C1V = 0;
}

/* Advanced Motor Functions */
// basic forward / backward
void forward(int percentage) {
		left_dir = 0; // set forward direction
		right_dir = 0; 
		setLeftSpeed(percentage);
		setRightSpeed(percentage);
}

void reverse(int percentage) {
		left_dir = 1; // set backward direction
		right_dir = 1; 
		setLeftSpeed(percentage);
		setRightSpeed(percentage);
}

// keep both wheels spinning, the other at 50% power
void turnLeft(int percentage) {
		left_dir = 0;
		right_dir = 0; 
		setLeftSpeed(percentage / 2);
		setRightSpeed(percentage);
}

void turnRight(int percentage) {
		left_dir = 0;
		right_dir = 0; 
		setLeftSpeed(percentage);
		setRightSpeed(percentage / 2);
}

// keep the other wheel stationary
void swingLeft(int percentage) {
		left_dir = 0;
		right_dir = 0; 
		setLeftSpeed(0);
		setRightSpeed(percentage);
}

void swingRight(int percentage) {
		left_dir = 0;
		right_dir = 0; 
		setLeftSpeed(percentage);
		setRightSpeed(0);
}

// pivot on the spot, both wheels in opposite directions
void pivotLeft(int percentage) {
		left_dir = 1;
		right_dir = 0; 
		setLeftSpeed(percentage);
		setRightSpeed(percentage);
}

void pivotRight(int percentage) {
		left_dir = 0;
		right_dir = 1; 
		setLeftSpeed(percentage);
		setRightSpeed(percentage);	
}

/* LED */
void InitGPIO(void) {
		// Enable Clock to PORTB and PORTD
		SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	
		// Configure MUX settings to make all 3 pins GPIO
		PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
		PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
		PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
		// Set Data Direction Registers for PortB and PortD
		PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
		PTD->PDDR |= MASK(BLUE_LED);

		PTB->PDOR |= (MASK(RED_LED) | MASK(GREEN_LED));
		PTD->PDOR |= MASK(BLUE_LED);
}

/* Controls the LED color */
void led_control(int led_color){
		switch(led_color){
				case 0: PTB->PDOR = MASK(RED_LED) | MASK(GREEN_LED);
						PTD->PDOR = MASK(BLUE_LED);
						break;
				case 1: PTB->PDOR = !MASK(RED_LED) | MASK(GREEN_LED);
						PTD->PDOR = MASK(BLUE_LED);
						break;
				case 2: PTB->PDOR = MASK(RED_LED) | !MASK(GREEN_LED);
						PTD->PDOR = MASK(BLUE_LED);
						break;
				case 3: PTB->PDOR = MASK(RED_LED) | MASK(GREEN_LED);
						PTD->PDOR = !MASK(BLUE_LED);
						break;
		}
}

void offRGB() {
		PTB->PDOR = MASK(RED_LED) | MASK(GREEN_LED);
		PTD->PDOR = MASK(BLUE_LED);
}

/* UART2 */
void initUART2(uint32_t baud_rate) {
		uint32_t divisor, bus_clock;
		
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
		PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
		
		UART2->C2 &= ~(UART_C2_RE_MASK);
		
		bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
		divisor = bus_clock / (baud_rate * 16);
		UART2->BDH = UART_BDH_SBR(divisor >> 8);
		UART2->BDL = UART_BDL_SBR(divisor);
		
		UART2->C1 = 0;
		UART2->S2 = 0;
		UART2->C3 = 0;
		
		NVIC_SetPriority(UART2_IRQn, 128);
		NVIC_ClearPendingIRQ(UART2_IRQn);
		NVIC_EnableIRQ(UART2_IRQn);
		UART2->C2 |= UART_C2_RIE_MASK;
		
		UART2->C2 |= UART_C2_RE_MASK;
}

/* UART2 Interrupt Handler */
void UART2_IRQHandler(void) {
		NVIC_ClearPendingIRQ(UART2_IRQn);
		if (UART2->S1 & UART_S1_RDRF_MASK) {
		// received a character
				rx_data = UART2->D;
		}
}

void delay(int ms) {
		for (int i = 0; i < (ms * 3800); i++) {}
}

/* MAIN function */
int main() {
		InitGPIO();
		InitPWM();
	
		// temp variables
		int ledColor = 0;
		int speed = 80;
		int state = 0;
	
		while (1) {
				// temp code to test basic movement
				delay(5000);
			
				switch(state) {
					case 0:
						turnLeft(speed);
						break;
					case 1:
						turnRight(speed);
						break;
					case 2:
						swingLeft(speed);
						break;
					case 3:
						swingRight(speed);
						break;
					case 4:
						pivotLeft(speed);
						break;
					case 5:
						pivotRight(speed);
						break;
				}
				
				state++;
				state = state % 6;
				
				ledColor++;
				ledColor = ledColor % 3;
				led_control(ledColor);
		}
}
