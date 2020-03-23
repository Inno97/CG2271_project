/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <stdbool.h>
#define BAUD_RATE 9600
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define UART_RX_PORTE23 23 //rx_pin, I plan to move/change the pin so that we can pluck in our LEDs easily
#define PTE31_Pin 31 //Buzzer pin

#define led_on true
#define led_off false
	
#define LEFT_MOTOR_COMP 100 // Left Motor Compensation (Out of 100)
#define RIGHT_MOTOR_COMP 100 // Right Motor Compensation

/* Global Variables */
volatile bool has_new_data = false;
volatile uint8_t rx_data = 0x01;
volatile int led_color = 0;

volatile int left_dir = 0; // 0 - forward, 1 - backward
volatile int right_dir = 0;
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

void InitGPIO(void)
{
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
}

void initBuzzer(void) {
		SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

		PORTE->PCR[PTE31_Pin] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[PTE31_Pin] |= PORT_PCR_MUX(3);

		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;

		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

		TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

		TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
		TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initMotor(void) {
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
				has_new_data = true;
		}
}

void offRGB() {
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);
}

void ledControl(int color, bool led) {
	if (led) {
		PTB->PCOR = MASK(color);
	} else {
		offRGB();
	}
}

void setFrequency(int frequency){
		if (frequency != 0) {
				TPM0->MOD = (375000 / frequency) -1;
				TPM0_C4V = 375000 / (1.25*frequency);
		} else {
				TPM0->MOD = 0;
				TPM0_C4V = 0;
		}
		
}

//These notes works with 2 resistors in parallel
int music_hash_table(uint8_t note) {
	switch(note) {
		case 0: return 831;//466;
		case 1: return 988;//554;
		case 2: return 1109;//622;
		case 3: return 1245;//698;
		case 4: return 1319;//740;
		case 5: return 1480;//831;
		case 6: return 1661;//932;
		case 7: return 1760;//988;
		case 8: return 1976;//1109;
	}
	return 0; //error
}

uint8_t front_mask(uint8_t note) {
	return note >> 4;
}

uint8_t back_mask(uint8_t note) {
	return note & 0x0f;
}

void play_note(int note, int duration) {
	  setFrequency(note);
		osDelay(duration << 6);
		setFrequency(0);
    osDelay(duration << 4);
}

void play_music(uint8_t noteArr[], uint8_t durationArr[]) {
  for (uint8_t i = 0; i < 26; i++) {
		int note = music_hash_table(front_mask(noteArr[i]));
		int duration = front_mask(durationArr[i]) << 2;
		play_note(note, duration);
		
		note = music_hash_table(back_mask(noteArr[i]));
		duration = back_mask(durationArr[i]) << 2;
    play_note(note, duration);
  }
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

void led_red_thread (void *argument) {
  for (;;) {
		ledControl(RED_LED, led_on);
		osDelay(1000);
		ledControl(RED_LED, led_off);
		osDelay(1000);
	}
}

void play_music_thread(void *argument) {
	uint8_t note[] = {0x45, 0x61, 0x46, 0x65, 0x45, 0x88, 0x88, 0x43, 
										 0x44, 0x44, 0x43, 0x43, 0x43, 0x21, 0x11, 0x22,
										 0x22, 0x21, 0x01, 0x01, 0x54, 0x16, 0x66, 0x78,
										 0x44, 0x65};
	uint8_t duration[] = {0x11, 0x11, 0x11, 0x21, 0x11, 0x11, 0x12, 0x11,
												0x11, 0x11, 0x21, 0x11, 0x11, 0x12, 0x11, 0x11,
												0x11, 0x21, 0x11, 0x11, 0x13, 0x11, 0x11, 0x11,
												0x11, 0x18};
																
	for(;;) {
		play_music(note, duration);
	}
}

void main_thread(void *argument) {
	int speed = 80;
	for(;;) {
		if (has_new_data) {
			switch(rx_data) {
			case 0x00: stop(); //update flag for led blinking instructions
				break;
			case 0x01: forward(speed);
				break;
			case 0x02: reverse(speed);
				break;
			case 0x03: pivotLeft(speed);
				break;
			case 0x04: pivotRight(speed);
				break;	
			case 0x05: turnLeft(speed);
				break;
			case 0x06: swingLeft(speed);
				break;
			case 0x07: turnRight(speed);
				break;
			case 0x08: swingRight(speed);
				break;
			case 0x09: stop(); //placeholder, end challenage
				break;
			}
			has_new_data = false;
		}
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	offRGB();
	initBuzzer();
	initMotor();
	initUART2(BAUD_RATE);
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	osThreadNew(play_music_thread, NULL, NULL);
	osThreadNew(main_thread, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
