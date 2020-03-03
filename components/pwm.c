/**
 * pwm.c
 *
 * Test out the buzzer, play 7 frequencies
 */
#include<MKL25Z4.h>
#define PTB0_Pin 0
#define PTB1_Pin 1

#define FREQNUM 7
#define BUS_CLOCK 23.986176

// Buzzer Sounds [C to B]
int freq[FREQNUM] = {262, 294, 330, 349, 392, 440, 494};
volatile int freqCount = 0;

void Init_UART2(uint32_t baud_rate) {
	uint32_t divisor;

	// enable clock to UART and PORT A
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// enable UART to pins for PTE22, PTE23
	PORTE->PCR[22] = PORT_PCR_MUX(4);
	PORTE->PCR[23] = PORT_PCR_MUX(4);

	// ensure tx and rx are disabled before configuration
	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);

	// Set baud rate to 4800 baud
	divisor = BUS_CLOCK / (baud_rate * 16);

	UART2->BDH = UART_BDH_SBR(divisor>>8);
	UART2->BDL = UART_BDH_SBR(divisor);

	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;

	// Enable transmitter and receiver
	UART2->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}

void initPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);

	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void setFrequency(int frequency){
	if (frequency != 0) {
		TPM1->MOD = (375000 / frequency) -1;
		TPM1_C0V = 375000 / (1.25*frequency);
	} else {
		TPM1->MOD = 0;
		TPM1_C0V = 0;
	}
}

void delay(int ms){
	int max = ms * 3428;
	for (int i=0; i < max; i++);
}

int main(void){
	SystemCoreClockUpdate();
	initPWM();

	while (1) {
		freqCount++;
		freqCount = freqCount % 8;
		delay(2000);
		setFrequency(freq[freqCount]);
	}
}
