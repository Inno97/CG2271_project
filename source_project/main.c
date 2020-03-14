/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "gpio.h"
#include "motor.h"
#include "buzzer.h"
#include "communication.h"
#define BAUD_RATE 9600

#define FLAG_UART_MSK 0x00000001U
#define FLAG_RUNNING_MSK 0x00000002U
#define FLAG_MOTOR_MSK 0x00000004U
#define FLAG_CONNECT_MSK 0x00000008U
#define FLAG_END_MSK 0x00000010U
#define FLAG_STATIC_MSK 0x00000020U
#define FLAG_ALL 0xFFFFFFFFU


/* Global Variables */
volatile uint8_t rx_data = 0x01;

osEventFlagsId_t evt_id;

osMutexId_t led_mutex;

const osThreadAttr_t Thread_attr = {
	.priority = osPriorityBelowNormal
};

const osMutexAttr_t Thread_Mutex_attr = {
  "myThreadMutex",     // human readable mutex name
  osMutexPrioInherit,  // attr_bits
  NULL,                // memory for control block   
  0U                   // size for control block
};

/* UART2 Interrupt Handler */
void UART2_IRQHandler(void) {
		NVIC_ClearPendingIRQ(UART2_IRQn);
		if (UART2->S1 & UART_S1_RDRF_MASK) {
			rx_data = UART2->D;
			osEventFlagsSet(evt_id, FLAG_UART_MSK);
		}
}

/*----------------------------------------------------------------------------
 * Application threads
 *---------------------------------------------------------------------------*/

void tBuzzer(void *argument) {
	uint8_t note[] = {0x45, 0x61, 0x46, 0x65, 0x45, 0x88, 0x88, 0x43, 
			0x44, 0x44, 0x43, 0x43, 0x43, 0x21, 0x11, 0x22,
			0x22, 0x21, 0x01, 0x01, 0x54, 0x16, 0x66, 0x78,
			0x44, 0x65};
	uint8_t duration[] = {0x11, 0x11, 0x11, 0x21, 0x11, 0x11, 0x12, 0x11,
				0x11, 0x11, 0x21, 0x11, 0x11, 0x12, 0x11, 0x11,
				0x11, 0x21, 0x11, 0x11, 0x13, 0x11, 0x11, 0x11,
				0x11, 0x18};
	while (1) {
		play_main_music(note, duration);
	}
}

void tEndChallenge(void * argument) {
	while(1) {
		osEventFlagsWait(evt_id, FLAG_END_MSK, osFlagsWaitAny, osWaitForever);
		beep();
	}
}

void tConnect(void *argument) {
	while(1) {
		osEventFlagsWait(evt_id, FLAG_CONNECT_MSK, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(led_mutex, osWaitForever);
		blink_twice();
		osMutexRelease(led_mutex);
		beep();
	}
}

void tStaticLed(void *argument) {
	while(1) {
		osEventFlagsWait(evt_id, FLAG_STATIC_MSK, osFlagsNoClear, osWaitForever);
		osMutexAcquire(led_mutex, osWaitForever);
		static_led_display();
		osMutexRelease(led_mutex);
	}
}

void tRunningLed(void *argument) { //tRunning priority > tStatic
	while(1) {
		osEventFlagsWait(evt_id, FLAG_RUNNING_MSK, osFlagsNoClear, osWaitForever);
		osMutexAcquire(led_mutex, osWaitForever);
		moving_led_display();
		osMutexRelease(led_mutex);
	}
}

void tMotor(void *argument) {
	while(1) {
		osEventFlagsWait(evt_id, FLAG_MOTOR_MSK, osFlagsWaitAny, osWaitForever);
		switch(rx_data) {
			case 0x00: stop(); //update flag for led blinking instructions
				osEventFlagsClear(evt_id, FLAG_RUNNING_MSK);
				osEventFlagsSet(evt_id, FLAG_STATIC_MSK);
				break;
			case 0x01: forward();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x02: reverse();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x03: pivotLeft();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x04: pivotRight();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;	
			case 0x05: turnLeft();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x06: swingLeft();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x07: turnRight();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
			case 0x08: swingRight();
				osEventFlagsClear(evt_id, FLAG_STATIC_MSK);
				osEventFlagsSet(evt_id, FLAG_RUNNING_MSK);
				break;
		}
	}
}

void tBrain(void *argument) {
	while(1) {
		osEventFlagsWait(evt_id, FLAG_UART_MSK, osFlagsWaitAny, osWaitForever);
		switch(rx_data) {
			case 0x00: 
			case 0x01: 
			case 0x02:
			case 0x03:
			case 0x04:
			case 0x05:
			case 0x06:
			case 0x07:
			case 0x08:
				osEventFlagsSet(evt_id, FLAG_MOTOR_MSK);
				break;
			case 0x09: 
				osEventFlagsSet(evt_id, FLAG_END_MSK);
				break;
			case 0x0A:
				osEventFlagsSet(evt_id, FLAG_CONNECT_MSK);
				break;
			}
	}
}

int main (void) {
		// System Initialization
		SystemCoreClockUpdate();
		initGPIO();
		initBuzzer();
		initMotor();
		initUART2(BAUD_RATE);
		// ...
		led_mutex = osMutexNew(&Thread_Mutex_attr); //No need priority inheritance
	
		evt_id = osEventFlagsNew(NULL);
		osEventFlagsClear(evt_id, FLAG_ALL);
		osEventFlagsSet(evt_id, FLAG_STATIC_MSK);
	 
		osKernelInitialize();                 // Initialize CMSIS-RTOS
		osThreadNew(tBrain,	NULL, NULL);    // Create application main thread
		osThreadNew(tConnect, NULL, NULL);
		osThreadNew(tStaticLed, NULL, &Thread_attr); //No need thread priority at all, lol
	  osThreadNew(tRunningLed, NULL, NULL);
		osThreadNew(tEndChallenge, NULL, NULL);
		osThreadNew(tBuzzer, NULL, NULL);
		osThreadNew(tMotor, NULL, NULL);
	
		osKernelStart();                      // Start thread execution
		for (;;) {}
}
