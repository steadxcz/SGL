#ifndef __LoggerDef_h
#define __LoggerDef_h

#include <stdint.h>

//#define SEL_2								(uint8_t)0x01						// for pin P0.0
//#define SEL_1								(uint8_t)0x08						// for pin P0.3
//#define CPU_FET							(uint8_t)0x10						// for P0.4
//#define VEXT_PRES						(uint8_t)0x20						// for pin P0.5
//#define	CNTR_PER						(uint8_t)0x01						// for pin P2.0	
	
#define DEFAULT_INTERVAL		(uint16_t)60
#define ALARM_INTERVAL			(uint16_t)1
#define RECORDS_ON_PAGE			(uint16_t)21
#define RECORDS_SIZE				(uint16_t)12

#define	TX_INTERRUPT				(uint8_t)0x02
#define	RX_INTERRUPT				(uint8_t)0x04
#define	RX_ERROR						(uint8_t)0x06
#define LAST_BYTE						(uint8_t)0x0A

#define FLASH_STATUS_WIP		(uint8_t)0x01    				// FLASH write in progress bit
#define FLASH_STATUS_WEL		(uint8_t)0x02						// FLASH write enable latch

#define NormalizationADC0(res) ((res*10000000)/268435456)


static const char*	okStr = "OK";
static const char*	errStr = "ERROR";
static const char*	devAddress = "1234";
static const char*	setTime 	= ";ST;";
static const char*	getTime 	= ";GT;";
static const char*	getData 	= ";GD;";
static const char*	clrFlash  = ";CF;";
static const char*	setLimits = ";SL;";
static const char*	getLimits = ";GL;";
static const char*	setPeriod = ";SP;";
static const char*	getPeriod = ";GP;";
static const char*	testSet 	= ";TS;";
static const char*	testReset	= ";TR;";

typedef enum{
	AR_CHAN_1,
	AR_CHAN_2,
	AR_CHAN_3	
}TAdcResult;

typedef enum {
	NOT_OPERATION,						// 0x00
	WRITE_STATUS,							// 0x01
	PAGE_PROGRAM,
	READ_FLASH,
	WRITE_DISABLE,
 	READ_STATUS ,
	WRITE_ENABLE,							// 0x06
	
	ERASE_SECTOR = 0x20,
	ENABLE_RST = 0x66,
	CHIP_RESET = 0x99,
	READ_CHIPID = 0x9F,
	DEEP_SLEEP = 0xB9,
	ERASE_CHIP	= 0xC7,
}enSpiInstr_t;

typedef enum {
	SPI_RX,
	SPI_TX
}enSpiMode_t;


typedef struct{
	uint16_t	lo;
	uint16_t	hi;
}wutInterval_t;

typedef struct {
	uint8_t lowByte;
	uint8_t midByte;
	uint8_t highByte;
	uint8_t instruction;
}spiCmd_t;

typedef union{
	uint32_t	flashAddr;
	spiCmd_t		txSeq;
} unSpiTxSeq_t;

typedef struct{
	uint32_t dateTime;
	int16_t gauge_1;
	int16_t gauge_2;
	int16_t gauge_3;
	int16_t temperatute;	
} TDataRecord;


/** Main Status Flags  ************************************************/
volatile struct {
  uint16_t  FLASH_ERASE_PROGRESS	: 1;								
  uint16_t  GET_SENSOR_DATA    		: 1;   	
  uint16_t  EXTERN_PWR		      	: 1; 
	uint16_t	UART_RX_END						: 1;
	uint16_t	UART_TX_END						:	1;
	uint16_t	UART_BUSY							: 1;
//	uint16_t	SLEEP_READY						:	1;
	uint16_t	ALARM_LIMITS					: 1;
	uint16_t	SPI_BUSY							: 1;	
	uint16_t	ADC0_READY						: 1;
	uint16_t	ADC1_READY						: 1;
	uint16_t	TEST_MODE							: 1;
	uint16_t	SEND_ARRAY						: 1;
	uint16_t	UPDATE_RTC						:	1;
}Flags;

volatile char uartTxBuffer[64];
char*	pUartTxBuffer = (char*)&uartTxBuffer;

volatile char uartRxBuffer[32];
char*	pUartRxBuffer = (char*)&uartRxBuffer;

volatile  struct {
	uint32_t spiHeader;
	TDataRecord buffer[21];
} sPage;

uint8_t		spiBuffer[16];
uint8_t* 	pSpiBuffer;
volatile uint16_t	spiTxCount;
volatile uint16_t	spiRxCount;
volatile enSpiMode_t	spiMode;
volatile uint32_t	flashAddress;

volatile int16_t	limitsArray[3];
volatile int32_t 	resultAdc0;
volatile int32_t 	resultAdc1;

volatile uint32_t utcTime;
volatile uint32_t utcTimeOld;
volatile uint16_t	recordsCount;
volatile uint32_t	pageCount;
volatile uint32_t	sendsRecordsCount;
volatile uint32_t	lastRecordIndex;

//volatile uint16_t	interval;
volatile uint16_t	normalInterval;
volatile uint16_t	alarmInterval;
/*
union{
	uint32_t		interval;
	wutInterval_t period;
} wakeUp_un;
*/
uint16_t (*function)(void);									// 


/*********************************************************************************************************/
void uartSendPacket(void);
int16_t getAdcResult(TAdcResult ar);
int16_t getTemperature(void);
void goToSleep(void);
uint32_t getDateTimeValue(char* buff);
uint32_t setUtcDateTime (uint16_t year, uint16_t mon, uint16_t day, uint16_t hh, uint16_t mm, uint16_t ss );
void storeToFlash(void);
void spiStartTransfer(uint8_t* buff, enSpiInstr_t instr, uint32_t data, uint16_t txCount, uint16_t rxCount);
uint16_t storeSensorData(void);
uint16_t executeCommand(char* buff);
uint16_t addrError(void);
uint16_t cmdError(void);
uint16_t setLoggerTime(void);
uint16_t getLoggerTime(void);
uint16_t getLoggerData(void);
uint16_t clrLoggerFlash(void);
uint16_t setLoggerLimits(void);
uint16_t setLoggerPeriod(void);
uint16_t getLoggerLimits(void);
uint16_t getLoggerPeriod(void);
uint16_t setTestMode(void);
uint16_t resetTestMode(void);
void sendMessage(char* str);
void timeToStr(char* buff, uint32_t* utc);
uint16_t createDataRecord(void);
uint8_t flashReadRegister(enSpiInstr_t reg);
uint8_t readFlashRegister(enSpiInstr_t reg);
void writeFlashRegister(enSpiInstr_t reg);
void flashWriteEnable(void);
void flashWriteDisable(void);
uint16_t sendTestData(void);


#endif

