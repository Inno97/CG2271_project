void initUART1(uint32_t baud_rate) {
		uint32_t divisor, bus_clock;
		
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
		PORTE->PCR[UART_RX_PORTE1] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[UART_RX_PORTE1] |= PORT_PCR_MUX(3);
		
		UART1->C2 &= ~(UART_C2_RE_MASK);
		
		bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
		divisor = bus_clock / (baud_rate * 16);
		UART1->BDH = UART_BDH_SBR(divisor >> 8);
		UART1->BDL = UART_BDL_SBR(divisor);
		
		UART1->C1 = 0;
		UART1->S2 = 0;
		UART1->C3 = 0;
		
		NVIC_SetPriority(UART1_IRQn, 128);
		NVIC_ClearPendingIRQ(UART1_IRQn);
		NVIC_EnableIRQ(UART1_IRQn);
		UART1->C2 |= UART_C2_RIE_MASK;
		
		UART1->C2 |= UART_C2_RE_MASK;
}

void UART1_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART1_IRQn);
	if (UART1->S1 & UART_S1_RDRF_MASK) {
	// received a character
		rx_data = UART1->D;
	}
}
