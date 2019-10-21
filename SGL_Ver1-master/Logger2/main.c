// Header:
// File Name: main.c
// Author: Yakov Churinov
// Date:

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <ADUCM360.h>
#include "BoardInit.h"
#include "LoggerDef.h"
#include "crc8.h"

typedef  uint16_t (*type_MyFunct)(void);
static  const type_MyFunct funcAddress[] = {addrError, cmdError, setLoggerTime, getLoggerTime, getLoggerData, clrLoggerFlash, setLoggerLimits, 
																						setLoggerPeriod, getLoggerLimits, getLoggerPeriod, setTestMode, resetTestMode};

/***************************************************************************************************************
	
	UART RX timeout interrupt handler

****************************************************************************************************************/
void SysTick_Handler(){
	pUartTxBuffer = (char*)&uartTxBuffer;																			// set the pointer to the beginning of the RX buffer
	pUartRxBuffer = (char*)&uartRxBuffer;
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

/***************************************************************************************************************
	
	UART interrupt handler

****************************************************************************************************************/
void UART_Int_Handler(){
	
	volatile uint8_t comIntID = pADI_UART->COMIIR;
	volatile uint8_t comLineStatus = pADI_UART->COMLSR;

	switch(comIntID){

		case TX_INTERRUPT:

			pADI_UART->COMTX = *pUartTxBuffer;																		// load byte to transmiter
			if (*pUartTxBuffer == LAST_BYTE){																			// if the last byte is loaded
				Flags.UART_TX_END = 1;
				pADI_UART->COMIEN &= ~COMIEN_ETBEI;																	// disable Tx interrupt;	
				Flags.UART_BUSY = 0;																								// disable flag UART_BUSY
			} else { pUartTxBuffer++;}																						// else, shift buffer pointer
			break;

		case RX_INTERRUPT:

			SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); // disable SysTick

			*pUartRxBuffer = pADI_UART->COMRX;																			// read byte from UART
			if(*pUartRxBuffer == LAST_BYTE){																				// if the last byte is readed
				Flags.UART_RX_END = 1;																								// set flag
				pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
			} else { 
					pUartRxBuffer++;
					if(pUartRxBuffer > ((char*)&uartRxBuffer + sizeof(uartRxBuffer))){pUartRxBuffer = (char*)&uartRxBuffer;}
					SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); // enable SysTick
				}
			break;
	
		default:																																// if UART error occured
			*pUartRxBuffer = pADI_UART->COMRX;																		// read byte from UART
			pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
			pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI; 											// enable RX interrupts)
			break;
	}
} 

/***************************************************************************************************************


****************************************************************************************************************/
void WakeUp_Int_Handler(){ 
	
	pADI_WUT->T2CLRI = T2CLRI_WUFD;																						// clear interrupt flag
	Flags.UPDATE_RTC = 1;
	__DSB();
}
 

/***************************************************************************************************************


****************************************************************************************************************/
void Ext_Int1_Handler (){ 
	pADI_INTERRUPT->EICLR = EICLR_IRQ1;	
	Flags.EXTERN_PWR = updateExtiState();	
	__DSB(); 
} 

/***************************************************************************************************************


****************************************************************************************************************/
void SPI0_Int_Handler(){

	volatile uint16_t spiStatus = pADI_SPI0->SPISTA;

	switch (spiMode){
    case SPI_TX:																									// if SPI work mode is TX
			spiTxCount--;																								// decrease TX byte counter
			if (spiTxCount){																						// if the counter is not zero
					pADI_SPI0->SPITX = *pSpiBuffer++;												// load next byte for transmition from buffer 				
			} else {																										// if all necessary bytes have been transferred
					if (spiRxCount){																				// if the number of bytes to be received is not equal to zero
						pADI_SPI0->SPITX = 0xAA;															// load to transmitter any byte, for example 0xAA
						spiMode = SPI_RX;																			// switch SPI work mode to RX
						pSpiBuffer = (uint8_t*)&spiBuffer;										// set pointer to buffer begin				
          } else {Flags.SPI_BUSY = 0;}														// if receive bytes not need, clear SPI_BUSY flag
				}
      break;
  
    case SPI_RX:																									// if SPI work mode is TX
			if(spiRxCount){																							// if the number of bytes to be received is not equal to zero
				pADI_SPI0->SPITX = 0xAA;																	// load to transmitter any byte, for example 0xAA
				if(pADI_SPI0->SPICON & SPICON_RFLUSH) {										// if RX FLUSH enabled
					pADI_SPI0->SPICON &= ~SPICON_RFLUSH;										// disable RX FLUSH
				} else {							
						while (pADI_SPI0->SPISTA & SPISTA_RXFSTA_MSK){ 				// while RX_FIFO not empty
							*pSpiBuffer++ = pADI_SPI0->SPIRX; 									// store byte from RX_FIFO to buffer
							spiRxCount--;																				// decrease RX byte counter
						}
					}
			} else { Flags.SPI_BUSY = 0; }															// if all necessary bytes have been received, clear SPI_BUSY flag			
      break;
  }
}

/***************************************************************************************************************


****************************************************************************************************************/
void ADC0_Int_Handler(){
   volatile uint16_t adcStatus __attribute__((unused)) = 0;  
   adcStatus = pADI_ADC0->STA;               																// read ADC status register
   resultAdc0 = pADI_ADC0->DAT;            																	// read ADC result register
   Flags.ADC0_READY = 1;                  																	// Set flag to indicate ready
}

/***************************************************************************************************************


****************************************************************************************************************/
void ADC1_Int_Handler (){
   volatile uint16_t adcStatus __attribute__((unused)) = 0;  
   adcStatus = pADI_ADC1->STA;               																// read ADC status register
   resultAdc1 = pADI_ADC1->DAT;            																	// read ADC result register
   Flags.ADC1_READY = 1;                  																	// Set flag to indicate ready
}

/***************************************************************************************************************


****************************************************************************************************************/
int main (void){

	initCorePin();


//	Flags.EXTERN_PWR = initExti();
	
	enableADC();
	initADC();
	disableADC();
	enableUART();
	initUART();

	enableSpi_0();
	initSpi_0();
	disableSpi_0();

	initSysTick();
		
	recordsCount = 0;
	pageCount = 0;

	limitsArray[0] = 0x7FFF;
	limitsArray[1] = 0x7FFF;
	limitsArray[2] = 0x7FFF;	

uint16_t interval=1;

  initRtcWakeUpTimer2 (&interval);
	//startWakeUpOnEverySecond();
	utcTime = setUtcDateTime(2019, 1, 1, 0, 0, 0);
	utcTimeOld=utcTime;
	
//	goToSleep();

	while (1){
		
    if (
        (Flags.EXTERN_PWR == 0)
		 && (Flags.SPI_BUSY==0)
       )

          {goToSleep();} 
		
		
		
		if(Flags.GET_SENSOR_DATA && !Flags.TEST_MODE){
			Flags.GET_SENSOR_DATA = 0;	
			if(storeSensorData()){storeToFlash();}
			if(Flags.EXTERN_PWR == 0) {goToSleep();}															// if VEXT is present no sleep
		}

		if (Flags.UPDATE_RTC){
			Flags.UPDATE_RTC = 0;
			utcTime++;
			if (Flags.TEST_MODE){
				if(storeSensorData()){recordsCount = 0;}
				sendTestData();
      }
			else
			{
				if ((utcTime-utcTimeOld)>5) {Flags.GET_SENSOR_DATA=1;utcTimeOld=utcTime;}
			}
		}
		
		if (Flags.UART_TX_END == 1){
			Flags.UART_TX_END = 0;
			if(Flags.SEND_ARRAY){
				Flags.SEND_ARRAY = createDataRecord();
				uartSendPacket();
			} else {
					pUartTxBuffer = (char*)&uartTxBuffer;																	// set the pointer to the beginning of the RX buffer
//					pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI; 											// enable RX interrupts
				}
		}

		if(Flags.UART_RX_END && !Flags.UART_BUSY){
			Flags.UART_RX_END = 0;
			function = funcAddress[executeCommand((char*)&uartRxBuffer)];  
			if(function() != 0){sendMessage((char*)errStr);}
		}		
	}  
}

/***************************************************************************************************************


****************************************************************************************************************/
void goToSleep(void){

	disableSpi_0();
	disableUART();
	disableADC();
	pADI_CLKCTL->CLKCON0   = CLKCON0_CD_DIV8;            											// 1MHz output of UCLK divide

	SCB->SCR = 0x04;       																										// for deep sleep mode - write to the Cortex-m3 System Control register bit2

	pADI_PWRCTL->PWRKEY = 0x4859;   																					// key1 
	pADI_PWRCTL->PWRKEY = 0xF27B;   																					// key2  
	pADI_PWRCTL->PWRMOD = PWRMOD_MOD_TOTALHALT; 															// deep sleep mode 

	uint32_t i = 0;
	for(i = 0; i < 2; i++){}

	__WFI(); 
		
	for(i = 0; i < 2; i++){}

	pADI_CLKCTL->CLKCON0   = CLKCON0_CD_DIV4;            											// 2MHz output of UCLK divide

}

/***************************************************************************************************************


****************************************************************************************************************/
void storeToFlash(void){
//	pADI_GP0->GPOEN &= ~CPU_FET;																							// enable VBAT switch
	
	/* Store data to FLASH memory */

	enableSpi_0();
	if(pageCount < 256){
		flashWriteEnable();
		spiStartTransfer((uint8_t*)&sPage, PAGE_PROGRAM, (pageCount * 256), 256, 0);
		while (Flags.SPI_BUSY){}																								// wait while SPI busy
		flashWriteDisable();
		pageCount++;
		recordsCount = 0;
	}
	disableSpi_0();
/*
	if(Flags.EXTERN_PWR == 1) {																								// if VEXT is present 
		pADI_GP0->GPOEN |= CPU_FET;																							// config CPU_FET as output																
		pADI_GP0->GPSET	= CPU_FET;																							// disable VBAT switch 
	}
*/
}

/***************************************************************************************************************


****************************************************************************************************************/
uint32_t setUtcDateTime (uint16_t year, uint16_t mon, uint16_t day, uint16_t hh, uint16_t mm, uint16_t ss ){

	struct tm dt;	
	dt.tm_year = year - 1900;
	dt.tm_mon = mon - 1;
	dt.tm_mday = day;
	dt.tm_hour = hh;
	dt.tm_min = mm;
	dt.tm_sec = ss;	
	time_t utcTime = mktime(&dt);
	return (uint32_t)utcTime;
}


/***************************************************************************************************************


****************************************************************************************************************/
uint32_t getDateTimeValue(char* buff){
	uint32_t utc = utcTime;
	struct tm dt = *localtime(&utc);
	return sprintf(buff, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
}

/***************************************************************************************************************


****************************************************************************************************************/
int16_t getAdcResult(TAdcResult ar){

	uint32_t tmpAdcReg = pADI_ADC0->CON & 0xFFFFFC00;
	uint16_t chanNum = ar;
	switch (chanNum){

		case AR_CHAN_1:
			tmpAdcReg |= ADC0CON_ADCCP_AIN0 | ADC0CON_ADCCN_AIN1;									// set for ADC0: AIN0 as POS input and as AIN1 as NEG input
			pADI_GP0->GPSET = SEL_1;																							// out HIGH to SEL_1
			break;
		
		case AR_CHAN_2:
			tmpAdcReg |= ADC0CON_ADCCP_AIN2 | ADC0CON_ADCCN_AIN3;									// set for ADC0: AIN2 as POS input and as AIN3 as NEG input
			pADI_GP0->GPSET = SEL_2;																							// out HIGH to SEL_2
			break;
		
		case AR_CHAN_3:
			tmpAdcReg |= ADC0CON_ADCCP_AIN8 | ADC0CON_ADCCN_AIN9;									// set for ADC0: AIN8 as POS input and as AIN9 as NEG input
			pADI_GP0->GPSET = SEL_1 | SEL_2;																			// out HIGH to SEL_1 and SEL_2
			break;	
	}

	uint32_t waitTime = 32000;
	while (waitTime){waitTime--;}
	
	pADI_ADC0->CON = tmpAdcReg;
	Flags.ADC0_READY = 0;
	uint16_t tmp = pADI_ADC0->MDE & ~ADC0MDE_ADCMD_MSK; 
	pADI_ADC0->MDE = (tmp | ADC0MDE_ADCMD_SINGLE);	
	while(Flags.ADC0_READY == 0){}

	int64_t result = (int64_t)resultAdc0;
	//result = result * 1200000 / 268435455;
  result = NormalizationADC0(result); 
	pADI_GP0->GPCLR = SEL_1 | SEL_2;																					// out LOW to SEL_1 and SEL_2

	return (int16_t)result;
}

/***************************************************************************************************************


****************************************************************************************************************/
int16_t getTemperature(void){

	Flags.ADC1_READY = 0;
	uint16_t tmp = pADI_ADC1->MDE & 0xFFF8; 
	pADI_ADC1->MDE = (tmp | ADC1MDE_ADCMD_SINGLE);
	while(Flags.ADC1_READY == 0){}

	int64_t result = (int64_t)resultAdc1;
	result = result * 1200000 / 268435455;
	result = (result - 82100) + 25000;
	return (int16_t)result; 
}
 
/***************************************************************************************************************


****************************************************************************************************************/
void uartSendPacket(void){	
	Flags.UART_BUSY = 1;
	pUartTxBuffer = (char*)&uartTxBuffer;																			// set the pointer to the beginning of the buffer
	pADI_UART->COMIEN |= COMIEN_ETBEI;																				// enable Tx interrupt
}

/***************************************************************************************************************


****************************************************************************************************************/
void spiStartTransfer(uint8_t* buff, enSpiInstr_t instr, uint32_t data, uint16_t txCount, uint16_t rxCount){

	while (Flags.SPI_BUSY == 1){}																							// wait while SPI busy
	while (pADI_SPI0->SPISTA & SPISTA_TXFSTA_MSK){}														// wait while TX_FIFO emty
	pSpiBuffer = buff;																												// set the pointer to the beginning of the buffer
	*pSpiBuffer++ = (uint8_t)instr;																						// write instruction to buffer
	*pSpiBuffer++ = (uint8_t)(data >> 16);																		// write bytes to Flash address positions
	*pSpiBuffer++ = (uint8_t)(data >> 8);
	*pSpiBuffer 	= (uint8_t)data;
	pSpiBuffer = buff;																												// set pointer to buffer begin
	spiTxCount = txCount;																											// remember RX and TX bytes count
	spiRxCount = rxCount;
	Flags.SPI_BUSY = 1;																												// set SPI_BUSY flag
	spiMode = SPI_TX;																													// set Spi mode to SPI_TX
	pADI_SPI0->SPICON |= SPICON_RFLUSH;																				// set the ban on receiving bytes to the RX_FIFO buffer
	pADI_SPI0->SPITX = *pSpiBuffer++;																					// load first byte to spi transmitter		
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t storeSensorData(void){
	

	lastRecordIndex = recordsCount;
	
	if (recordsCount < RECORDS_ON_PAGE) {
		
		enableADC();
		sPage.buffer[recordsCount].dateTime = utcTime;
		sPage.buffer[recordsCount].temperatute = getTemperature();
		sPage.buffer[recordsCount].gauge_1 = getAdcResult(AR_CHAN_1);
		sPage.buffer[recordsCount].gauge_2 = getAdcResult(AR_CHAN_2);
		sPage.buffer[recordsCount].gauge_3 = getAdcResult(AR_CHAN_3);
		disableADC();

		Flags.ALARM_LIMITS = ((sPage.buffer[recordsCount].gauge_1 > limitsArray[0]) || (sPage.buffer[recordsCount].gauge_1 < (-1 * limitsArray[0]))) ? 1 : 0;
		Flags.ALARM_LIMITS |= ((sPage.buffer[recordsCount].gauge_2 > limitsArray[1]) || (sPage.buffer[recordsCount].gauge_1 < (-1 * limitsArray[1]))) ? 1 : 0;
		Flags.ALARM_LIMITS |= ((sPage.buffer[recordsCount].gauge_3 > limitsArray[2]) || (sPage.buffer[recordsCount].gauge_1 < (-1 * limitsArray[2]))) ? 1 : 0;
		
		recordsCount++; 
	}
	return (recordsCount == RECORDS_ON_PAGE) ? 1 : 0;	
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t executeCommand(char* buff){
	char* p = strstr(buff, devAddress);
	if(p == NULL){return 0;}
	if(p != buff){return 0;}
	p = strchr(buff, ';');
	if((p - buff) != sizeof(devAddress)){return 0;}
	if(strstr(p, setTime) != NULL){return 2;}
	if(strstr(p, getTime) != NULL){return 3;}
	if(strstr(p, getData) != NULL){return 4;}
	if(strstr(p, clrFlash) != NULL){return 5;}
	if(strstr(p, setLimits) != NULL){return 6;}
	if(strstr(p, setPeriod) != NULL){return 7;}
	if(strstr(p, getLimits) != NULL){return 8;}
	if(strstr(p, getPeriod) != NULL){return 9;}
	if(strstr(p, testSet) != NULL){return 10;}
	if(strstr(p, testReset) != NULL){return 11;}
	return 1;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerTime(void){
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	uint16_t year = atoi((pUartRxBuffer + 1));
	if((year < 1900) || (year > 2105)) {return 1;}
	
	pUartRxBuffer = strchr(pUartRxBuffer, '/');
	if (pUartRxBuffer == NULL){return 1;}				
	uint16_t mon = atoi((pUartRxBuffer + 1));
	if((mon < 1) || (mon > 12)) {return 1;}

	pUartRxBuffer = strchr(pUartRxBuffer + 1, '/');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t day = atoi((pUartRxBuffer + 1));
	if((day < 1) || (day > 31)) {return 1;} 
	
	pUartRxBuffer = strchr(pUartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t hh = atoi((pUartRxBuffer + 1));
	if(hh > 23) {return 1;} 

	pUartRxBuffer = strchr(pUartRxBuffer, ':');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t mm = atoi((pUartRxBuffer + 1)); 
	if(mm > 59) {return 1;} 
	
	pUartRxBuffer = strchr(pUartRxBuffer, ':');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t ss = atoi((pUartRxBuffer + 25));
	if(ss > 59) {return 1;} 

	T2CON_ENABLE_BBA = 0; 																										// disable the RTC timer	
	Flags.GET_SENSOR_DATA = 0;
	utcTime = setUtcDateTime(year, mon, day, hh, mm, ss);
	pADI_WUT->T2CLRI = T2CLRI_WUFD;																						// clear interrupt flag	
	T2CON_ENABLE_BBA = 1;																											// enable the RTC timer	
	Flags.UPDATE_RTC = 0;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	getLoggerTime(void){
	pUartTxBuffer = (char*)&uartTxBuffer;
	pUartTxBuffer += getDateTimeValue(pUartTxBuffer);
	*pUartTxBuffer++ = '\r';
	*pUartTxBuffer = '\n';
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	getLoggerData(void){
	sendsRecordsCount = 0;
	Flags.SEND_ARRAY = createDataRecord();
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t createDataRecord(void){
		
	while (Flags.SPI_BUSY){}
	struct tm dt;
	uint32_t flashRecords = pageCount * RECORDS_ON_PAGE;

	if (flashRecords > sendsRecordsCount){
		
		uint32_t pagesTransmitted = sendsRecordsCount / RECORDS_ON_PAGE;	
		uint32_t flashAddr = (pagesTransmitted * 256) + ((sendsRecordsCount - (pagesTransmitted * RECORDS_ON_PAGE)) * RECORDS_SIZE);		
		spiStartTransfer((uint8_t*)&spiBuffer, READ_FLASH, flashAddr, 4, 12);
		while (Flags.SPI_BUSY){}
		
		TDataRecord* pDataRecord = (TDataRecord*)&spiBuffer;
		dt = *localtime((uint32_t*)&pDataRecord->dateTime);
		sprintf((char*)&uartTxBuffer, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d;", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
		pUartTxBuffer = (char*)&uartTxBuffer + 20;
		sprintf(pUartTxBuffer,"%i;%i;%i;%i\r\n", pDataRecord->gauge_1, pDataRecord->gauge_2, pDataRecord->gauge_3, pDataRecord->temperatute);

	} else {
		
			uint16_t offset = sendsRecordsCount - flashRecords;
			if(offset < recordsCount){				 
				dt = *localtime((uint32_t*)&sPage.buffer[offset].dateTime);
				sprintf((char*)&uartTxBuffer, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d;", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
				pUartTxBuffer = (char*)&uartTxBuffer + 20;
				sprintf(pUartTxBuffer,"%i;%i;%i;%i\r\n", sPage.buffer[offset].gauge_1, sPage.buffer[offset].gauge_2, sPage.buffer[offset].gauge_3, sPage.buffer[offset].temperatute);
			
			}
		}
	sendsRecordsCount++;
	return (sendsRecordsCount < (flashRecords + recordsCount)) ? 1 : 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	sendTestData(void){
	struct tm dt;
	dt = *localtime((uint32_t*)&sPage.buffer[lastRecordIndex].dateTime);
	sprintf((char*)&uartTxBuffer, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d;", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
	pUartTxBuffer = (char*)&uartTxBuffer + 20;
	sprintf(pUartTxBuffer,"%i;%i;%i;%i\r\n", sPage.buffer[lastRecordIndex].gauge_1, sPage.buffer[lastRecordIndex].gauge_2, sPage.buffer[lastRecordIndex].gauge_3, sPage.buffer[lastRecordIndex].temperatute);
	uartSendPacket();
	return 0;
}


/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	clrLoggerFlash(void){

	flashWriteEnable();
	writeFlashRegister(ERASE_CHIP);	
	flashWriteDisable();
	recordsCount = 0;
	pageCount = 0;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerLimits(void){
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	limitsArray[0] = (int16_t)atoi((pUartRxBuffer + 1));

	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}				
	limitsArray[1] = (int16_t)atoi((pUartRxBuffer + 1));

	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}
	limitsArray[2] = (int16_t)atoi((pUartRxBuffer + 1));
		
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerPeriod(void){
	uint16_t i1;
	uint16_t i2;
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	i1 = (int16_t)atoi((pUartRxBuffer + 1));
	if(i1 < 2){return 1;}
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}				
	i2 = (int16_t)atoi((pUartRxBuffer + 1));
	if(i2 < 1 ){return 1;}
	if(i2 >= i1 ){return 1;}
	
	normalInterval = i1;
	alarmInterval = i2;
  sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t getLoggerLimits(void){
	sprintf((char*)&uartTxBuffer, "%d;%d;%d\r\n", limitsArray[0], limitsArray[1], limitsArray[2]);
	uartSendPacket();
	return 0;
} 

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t getLoggerPeriod(void){
	sprintf((char*)&uartTxBuffer, "%d;%d\r\n", normalInterval, alarmInterval);
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t setTestMode(void){
	Flags.TEST_MODE = 1;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t resetTestMode(void){
	Flags.TEST_MODE = 0;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t 	addrError(void){
	pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
	pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI; 									// enable RX interrupts)
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t 	cmdError(void){
	return 1;
}

/***************************************************************************************************************


****************************************************************************************************************/
void	sendMessage(char* str){
	sprintf((char*)&uartTxBuffer, "%s\r\n", str);
	uartSendPacket();
}

/***************************************************************************************************************


****************************************************************************************************************/
uint8_t readFlashRegister(enSpiInstr_t reg){
	spiStartTransfer((uint8_t*)&spiBuffer, reg, 0, 1, 1);
	while (Flags.SPI_BUSY == 1){}
	pSpiBuffer = (uint8_t*)&spiBuffer;		
	return *pSpiBuffer;
}

/***************************************************************************************************************


****************************************************************************************************************/
void writeFlashRegister(enSpiInstr_t reg){
	spiStartTransfer((uint8_t*)&spiBuffer, reg, 0, 1, 0);
	while (Flags.SPI_BUSY == 1){}
}

/***************************************************************************************************************


****************************************************************************************************************/
void flashWriteEnable(void){
	while (readFlashRegister(READ_STATUS) & FLASH_STATUS_WIP);
	writeFlashRegister(WRITE_ENABLE);
	while ((readFlashRegister(READ_STATUS) & FLASH_STATUS_WEL) != FLASH_STATUS_WEL);
}

/***************************************************************************************************************


****************************************************************************************************************/
void flashWriteDisable(void){
	while (readFlashRegister(READ_STATUS) & FLASH_STATUS_WIP);
	writeFlashRegister(WRITE_DISABLE);
}

