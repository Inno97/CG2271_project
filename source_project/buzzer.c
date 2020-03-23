/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#define PTE31_Pin 31 //Buzzer pin

osMutexId_t buzzer_mutex;

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

void play_main_music(uint8_t noteArr[], uint8_t durationArr[]) {
	for (uint8_t i = 0; i < 26; i++) {
		osMutexAcquire(buzzer_mutex, osWaitForever);
		int note = music_hash_table(front_mask(noteArr[i]));
		int duration = front_mask(durationArr[i]) << 2;
		play_note(note, duration);
		
		note = music_hash_table(back_mask(noteArr[i]));
		duration = back_mask(durationArr[i]) << 2;
		play_note(note, duration);
		osMutexRelease(buzzer_mutex);
	}
}

void beep(void) {
	osMutexAcquire(buzzer_mutex, osWaitForever);
	osDelay(1000);
	play_note(2500, 4);
	play_note(2500, 4);
	osDelay(1000);
	osMutexRelease(buzzer_mutex);
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
		buzzer_mutex = osMutexNew(NULL);
}
