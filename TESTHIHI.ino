#include "sam3xa/include/sam3xa.h"
#include <Arduino.h>
#include <SPI\SPI.h>
#include "softspi.h"
//#include "SdFat.h"
//#include "sdfat\src\spidriver\softspi.h"
//#include "SdFat\src\SpiDriver\DigitalPin.h"
#define SPI_DMAC_RX_CH			1																	//DMAC transmit channel
#define SPI_DMAC_TX_CH			0																	//DMAC receive channel
#define SPI_RX_IDX				2																	//DMAC Channel HW Interface Number for SPI RX
#define SPI_CHIP_SEL			3																	//SPI chip select (RX1)
#define SDCARD_SEL				22																	//SD card chip select
#define DS1302_CE				5																	//DS1302 Chip enable
#define DS1302_SDA				6																	//DS1302 Data
#define DS1302_SCK				7																	//DS1302 Clock
/*Define command for SD card*/
#define CMD0					0x00																//Go to Idle state, reset SD card 
#define CMD1					0x01																//Send operation condition
#define CMD16					0x10																//Set length of block
#define CMD17					0x11																//Read single block
#define CMD24					0x18																//Write single block
/*Define status of SD card*/
#define stt_ok					0																	//Status: OK
#define stt_timeout				1																	//Status: timeout mean while reset
#define stt_CMD1_error			2																	//Status: error when using command CMD1
#define stt_len_block_error		3																	//Status: error when setting block length
#define stt_CMD24_error			4																	//Status: timeout when using command CMD24
#define stt_write_error			5																	//Status: error when writing data
#define stt_busy				6																	//Status: card is busy
#define stt_CMD17_error			7																	//Status: error when using command 17
#define stt_read_error			8																	//Status: error when reading data
#define SDcard_Block_Len		512
char SDCard_stt = 0;
const uint8_t SOFT_SPI_MISO_PIN_SDCARD = 38;
const uint8_t SOFT_SPI_MOSI_PIN_SDCARD = 39;
const uint8_t SOFT_SPI_SCK_PIN_SDCARD = 40;
const uint8_t SPI_MODE_SDCARD = 0;
//const uint8_t SOFT_SPI_MISO_PIN = 52;
//const uint8_t SOFT_SPI_MOSI_PIN = 26;
//const uint8_t SOFT_SPI_SCK_PIN = 27;
//#define SOFT_SPI_CS_PIN 2
#define SPI_CS 37
#define CS_PIN 2
#define SCK_PIN 27
#define DATA_PIN 26
const uint8_t SPI_MODE = 0;
//SoftSPI<SOFT_SPI_MISO_PIN, SOFT_SPI_MOSI_PIN, SOFT_SPI_SCK_PIN, SPI_MODE> spi;
typedef struct																						//Define struct DMA linker list	
{
	uint32_t Saddr;
	uint32_t Daddr;
	uint32_t CtrlA;
	uint32_t CtrlB;
	uint32_t Dscr;
} LLI;

#define BUFF_SIZE 3000
#define BUFF_ELEMENT 28
LLI LLI_array[BUFF_ELEMENT];																		//Create Linker list array for DMA transfer
uint8_t rx_buffer[BUFF_ELEMENT][BUFF_SIZE];															//Create buffers for DMA transfer
void enable_DMAC()																//Enable DMA
{
	DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
void disable_DMAC()																//Disable DMA
{
	DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
void enable_channel_DMAC(uint32_t ul)											//Enable channel DMA
{
	DMAC->DMAC_CHER |= DMAC_CHER_ENA0 << ul;
}
void disable_channel_DMAC(uint32_t ul)											//Disable channel DMA
{
	DMAC->DMAC_CHDR |= DMAC_CHDR_DIS0 << ul;
}
bool dma_transfer_done(uint32_t ul)												//Check transfer DMA
{
	return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul)) ? false : true;
}
void config_DMAC_Linker_List()													//Config DMA linker list DMAC.DSCR(0) -> LLI(0).DSCR(0) -.....->LLI(n-1)DSCR(n-1)-> 0
{
	disable_channel_DMAC(SPI_DMAC_RX_CH);
	/*LLI(0)*/
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_SADDR = (uint32_t)&SPI0->SPI_RDR;
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DADDR = (uint32_t)rx_buffer[0];
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DSCR = (uint32_t)&LLI_array[0];
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLA = (uint16_t)BUFF_SIZE
		| DMAC_CTRLA_DST_WIDTH_BYTE
		| DMAC_CTRLA_SRC_WIDTH_BYTE
		| (~DMAC_CTRLA_DONE);
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR_FETCH_FROM_MEM
		| DMAC_CTRLB_DST_DSCR_FETCH_FROM_MEM
		| DMAC_CTRLB_FC_PER2MEM_DMA_FC
		| DMAC_CTRLB_SRC_INCR_FIXED
		| DMAC_CTRLB_DST_INCR_INCREMENTING;
	DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CFG = DMAC_CFG_SRC_H2SEL
		| DMAC_CFG_SOD
		| DMAC_CFG_FIFOCFG_ASAP_CFG
		| DMAC_CFG_SRC_PER(SPI_RX_IDX);
	/*LLI(1..3)*/
	for (int i = 0; i < BUFF_ELEMENT; i++)
	{
		LLI_array[i].Saddr = (uint32_t)&SPI0->SPI_RDR;
		LLI_array[i].Daddr = (uint32_t)rx_buffer[i];
		LLI_array[i].CtrlA = (uint16_t)BUFF_SIZE
			| DMAC_CTRLA_DST_WIDTH_BYTE
			| DMAC_CTRLA_SRC_WIDTH_BYTE
			| (0x0u << 31);
		LLI_array[i].CtrlB = DMAC_CTRLB_SRC_DSCR_FETCH_FROM_MEM
			| DMAC_CTRLB_DST_DSCR_FETCH_FROM_MEM
			| DMAC_CTRLB_FC_PER2MEM_DMA_FC
			| DMAC_CTRLB_SRC_INCR_FIXED
			| DMAC_CTRLB_DST_INCR_INCREMENTING;
		LLI_array[i].Dscr = (uint32_t)&LLI_array[i + 1];
	}
	LLI_array[BUFF_ELEMENT - 1].Dscr = 0;
	enable_channel_DMAC(SPI_DMAC_RX_CH);
}
void init_SPI0_slave()															//Config SPI on Slave Mode
{
	SPI.begin(PIN_SD_CD);
	REG_SPI0_CR = SPI_CR_SWRST;													// reset SPI
	REG_SPI0_CR = SPI_CR_SPIEN;													// enable SPI
	REG_SPI0_MR = SPI_MR_MODFDIS;												// slave and no modefault
	REG_SPI0_CSR = 0x00;														// DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer
	//REG_SPI0_CSR |= (0 < SPI_CSR_CPOL) | (1 < SPI_CSR_NCPHA);
	pmc_enable_periph_clk(ID_DMAC);
	disable_DMAC();
	DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
	enable_DMAC();
}
int spiRecv()																	//SPI Receive throw DMA channel
{
	Spi* Spi0 = SPI0;
	int rtn = 0;
	uint32_t s;
	s = Spi0->SPI_SR;
	config_DMAC_Linker_List();
	while (!dma_transfer_done(SPI_DMAC_RX_CH))									//wait for receive success
	{
		SerialUSB.print(1);
		delay(1000);
	};
	if (Spi0->SPI_SR & SPI_SR_OVRES)											//check status register of SPI
		rtn |= 1;
	return rtn;
}
void writehigh(uint8_t Pin)
{
	digitalWrite(Pin, 1);
}
void writelow(uint8_t Pin)
{
	digitalWrite(Pin, 0);
}
void latch()
{
	digitalWrite(SCK_PIN, 1);
	delay(5);
	digitalWrite(SCK_PIN, 0);
	delay(5);
}
void shiftdata(uint32_t data)
{
	writelow(CS_PIN);
	for (int i = 0; i < 32; i++)
	{
		if (data & 0x80000000)
		{
			writehigh(DATA_PIN);
		}
		else
		{
			writelow(DATA_PIN);
		}
		data <<= 1;
		latch();
	}
	writehigh(CS_PIN);
}
//void init_soft_SPI()														//Create soft SPI for data transfer on SDcard interface
//{
//	Serial.println(1);
//	Serial.print("Initializing soft spi...");
//	if (!spi1.begin(SOFT_SPI_CS_PIN, SD_SCK_HZ(4)))
//	{
//		Serial.println("initialization failed!");
//	}
//	Serial.println("initialization done.");
//}
void setup()
{
	SerialUSB.begin(115200);
	SerialUSB.print("waiting for init spi dma");
	init_SPI0_slave();
	SerialUSB.print("init spi dma success");
	//spi.begin();
	//init_soft_SPI();
	pinMode(CS_PIN, OUTPUT);
	pinMode(SCK_PIN, OUTPUT);
	pinMode(DATA_PIN, OUTPUT);
	delay(1000);
	for (int i = 0; i < 3; i++)
	{
		shiftdata(0xA2939A30);
		delay(5);
		shiftdata(0x855020C1);
		delay(5);
		shiftdata(0xEAFF1DC2);
		delay(5);
		shiftdata(0x9EC00083);
	}
	SerialUSB.print("Send command success");
	//fastDigitalWrite(SOFT_SPI_SCK_PIN, 0);
	////fastDigitalWrite(SOFT_SPI_CS_PIN, 1);
	//SerialUSB.print("Send command success");
	//fastDigitalWrite(SOFT_SPI_CS_PIN, 1);
 // /* add setup code here */
	
}
int i = 0;
void loop()
{
		SerialUSB.print("starting receive...");
		spiRecv();
		SerialUSB.println("receive success...");
		String strout;
		for (int i = 0; i < BUFF_ELEMENT; i++)
		{
			strout = "";
			for (int j = 0; j < BUFF_SIZE; j++)
			{
				strout += (String(rx_buffer[i][j], HEX) + " ");
			}
			SerialUSB.print(strout);
		}
		//i++;
	delay(5000);

	//SerialUSB.print(fastDigitalRead(SPI_CS));

  /* add main program code here */

}
