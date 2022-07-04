#include <stdio.h>
#include "stm32f4xx.h"
#include "log.h"
#include "dispatcher.h"
#include "power.h"
#include "bridge.h"
#include "program.h"
#include "cc2500.h"
#include "config.h"
#include "debug.h"

uint8_t SPI_buffer_RX[TX_SPI_BUFFER_LENGTH];
uint8_t SPI_buffer_TX[TX_SPI_BUFFER_LENGTH];
uint8_t SPI_GET_PACKET[RX_SPI_BUFFER_LENGTH];

static command_list LastCmd = COMMAND_NOP;
static uint32_t lastQueryTime = 0;

volatile uint32_t nCount;
volatile uint8_t maxPeriod;

// Init settings to write to CC2500 using SPI with DMA
uint8_t Settings[] = {
		    TI_CCxxx0_IOCFG2, 	0x01,  				// GDO0Output Pin Configuration
	        TI_CCxxx0_IOCFG0, 	0x06,  				// GDO0Output Pin Configuration
			TI_CCxxx0_FIFOTHR, 	0x0F,  				// FIFO Threshold
			TI_CCxxx0_PKTLEN, 	PACKET_LENGTH,  	// Packet length
			TI_CCxxx0_PKTCTRL0, 0x44,  				// Packet Automation Control
			TI_CCxxx0_PKTCTRL1, 0x0F,  				// Packet Automation Control
			TI_CCxxx0_FSCTRL1, 	0x0A, 				// Frequency Synthesizer Control
			TI_CCxxx0_FSCTRL0, 	0x00, 				//
			TI_CCxxx0_ADDR, 	TABLET_ADDR, 		// Address
			TI_CCxxx0_CHANNR, 	CHANNEL,			// Channel number
			TI_CCxxx0_FREQ2, 	0x5C,   			// Frequency Control Word, High Byte
			TI_CCxxx0_FREQ1, 	0x4E,   			// Frequency Control Word, Middle Byte
			TI_CCxxx0_FREQ0, 	0xC3,   			// Frequency Control Word, Low Byte
			TI_CCxxx0_MDMCFG4, 	0x1D, 				// Modem Configuration
			TI_CCxxx0_MDMCFG3, 	0x3B, 				// Modem Configuration
			TI_CCxxx0_MDMCFG2, 	0x73, 				// Modem Configuration
			TI_CCxxx0_MDMCFG1, 	0xA3, 				// Modem Configuration
			TI_CCxxx0_MDMCFG0, 	0xFF, 				// Modem Configuration
			TI_CCxxx0_DEVIATN, 	0x00, 				// Modem Deviation Setting
			TI_CCxxx0_MCSM1, 	0x03, 				// auto RX after TX
			TI_CCxxx0_MCSM0, 	0x18, 				// Main Radio Control State Machine Configuration
			TI_CCxxx0_FOCCFG, 	0x1D, 				// Frequency Offset Compensation Configuration
			TI_CCxxx0_BSCFG, 	0x1C,   			// Bit Synchronization Configuration
			TI_CCxxx0_AGCCTRL2, 0xC7,   			// AGC Control
			TI_CCxxx0_AGCCTRL1, 0x00,   			// AGC Control
			TI_CCxxx0_AGCCTRL0, 0xB0,   			// AGC Control
			TI_CCxxx0_FREND1, 	0xB6,  				// Front End RX Configuration
			TI_CCxxx0_FSCAL3, 	0xEA,  				// Frequency Synthesizer Calibration
			TI_CCxxx0_FSCAL1, 	0x00,  				// Frequency Synthesizer Calibration
			TI_CCxxx0_FSCAL0, 	0x11,  				// Frequency Synthesizer Calibration
			TI_CCxxx0_PATABLE, 	0xFF, 				// Output Power = +1 dBm
			TI_CCxxx0_SRX 							// start RX
};
const size_t sizeof_Settings = sizeof Settings;

// this settings write to CC2500 using SPI with DMA if CRC fail/ Packet content fail
uint8_t flush_and_rx[] = {
TI_CCxxx0_SIDLE, 	// go to IDLE, it is necessary to use FIFO FLUSH commands
TI_CCxxx0_SFRX, 	// flush RX FIFO
TI_CCxxx0_SFTX, 	// flush TX FIFO
TI_CCxxx0_SRX 		// start RX
};
const size_t sizeof_flush_and_rx = sizeof flush_and_rx;

FunctionalState ledState;

command_list Radio_GetCommand()
{
	return LastCmd;
}

ErrorStatus CC2500_Init(void)
{
	if (CC2500_struct_init() == ERROR)
	{
		return ERROR;
	}

	// Reset CC2500 and init spi
	if (CC2500_powerup_reset() == ERROR)
	{
		return ERROR;
	}

	// write settings to CC2500
	SPI_WR_CC2500_DMA(SPI_buffer_RX, Settings, sizeof_Settings);

	// EN EXTI from CC2500 pin GDO2
	EXTI_GDO2_Init();

	ledState = DISABLE;
	return SUCCESS;
}

ErrorStatus CC2500_struct_init(void)
{
	status_st.status = TABLET_IDLE;

	status_st.act_period = 1;
	status_st.act_time = 0;

	SPI_GET_PACKET[0] = TI_CCxxx0_READ_BURST | TI_CCxxx0_RXFIFO; // burst read from RX FIFO command

	spi_tx_st.header.sidle = TI_CCxxx0_SIDLE;
	spi_tx_st.header.sfrx = TI_CCxxx0_SFRX;
	spi_tx_st.header.sftx = TI_CCxxx0_SFTX;
	spi_tx_st.header.stx = TI_CCxxx0_STX;
	spi_tx_st.header.txfifo = TI_CCxxx0_WRITE_BURST | TI_CCxxx0_TXFIFO;

	spi_tx_st.packet.addrs.d_addr = DONGLE_ADDR;
	spi_tx_st.packet.addrs.t_addr = TABLET_ADDR;

	if (RX_SPI_BUFFER_LENGTH != sizeof(spi_rx) || TX_SPI_BUFFER_LENGTH != sizeof(spi_tx))
	{
	  return ERROR; // несоответствие размеров длин пакета и структур
	}

	return SUCCESS;
}

ErrorStatus CC2500_powerup_reset(void)
{
    uint8_t cc2500_reply = 0;

	spi_pin_setup(); // to have access to spi pins
	Delay_init();

	// according DS 19.1 page 39 and DN503 page 8
	CC2500_DESELECT;
	Delay_us(1);
	CC2500_SELECT;
	SCK_HI;
	MOSI_LOW;
	Delay_us(1);
	CC2500_DESELECT;
	Delay_us(50);
	spi_setup(); // init HW SPI with DMA
	CC2500_SELECT;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	while (GPIO_ReadInputDataBit(MISO_PORT, MISO_PIN));
	// Send SRES command
	spi_tx_rx(TI_CCxxx0_SRES);
	// Wait for chip to finish internal reset
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	while (GPIO_ReadInputDataBit(MISO_PORT, MISO_PIN));
	cc2500_reply = spi_tx_rx(TI_CCxxx0_SNOP);
	CC2500_DESELECT;
	// normal state cc2500 after reset is "0x0F": chip ready, IDLE state, TX FIFO free
	// otherwise, cc2500 not connected/broken/SPI timings were too low/power source problem
	if (cc2500_reply == 0x0F)
	{
		return SUCCESS;
	}
	else
	{
		return ERROR; // there is no cc2500 connected
	}
}

void RADIO_TX (void)
{
	// add status
	memcpy(&spi_tx_st.packet.status, &status_st, sizeof(status_st));

	//add received packet
	memcpy(&spi_tx_st.packet.copy_rx, &spi_rx_st.packet, sizeof(spi_rx_st.packet));

	SPI_WR_CC2500_DMA (SPI_buffer_RX, (uint8_t*)&spi_tx_st, TX_SPI_BUFFER_LENGTH);
}

void SPI_WR_CC2500_DMA (uint8_t* buffer_in, uint8_t* buffer_out, uint16_t data_length)
{
	// clear TC flags, if it was not cleared yet
	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
	// set numbers of bytes to TX/RX
	DMA1_Stream3->NDTR = data_length;
	DMA1_Stream4->NDTR = data_length;
	// set buffers to TX/RX
	DMA1_Stream3->M0AR = (uint32_t) &buffer_in[0]; // RX (MISO)
	DMA1_Stream4->M0AR = (uint32_t) &buffer_out[0]; // TX (MOSI)
	cc_select_and_wait_for_rdy();
	DMA_Cmd(DMA1_Stream3, ENABLE); // RX start
	DMA_Cmd(DMA1_Stream4, ENABLE); // TX start
}

void processing(void)
{
	uint32_t i;
	char line[160];

	if (packet_filter() == ERROR){ return; }

	status_st.signal = spi_rx_st.rssi;

	// cmd processing
	switch (spi_rx_st.packet.command)
	{
		case COMMAND_SYNC:
			break;
		case COMMAND_GET_STATUS:
			get_Status();
			break;
		case COMMAND_PROGRAMM_LOAD:
			status_st.status = TABLET_LOADING;
			programm_Load();
			break;
		case COMMAND_PROGRAMM_START:
			if (((1<<(TABLET_ADDR-1)) & spi_rx_st.packet.param) != 0)
			{
				LastCmd = COMMAND_PROGRAMM_START;
				Dispatcher_SetFlag(FLAG_RADIO_CMD);
				status_st.status = TABLET_STIMUL;
			}

			CC2500_FLUSH_AND_RX();
			break;
		case COMMAND_PROGRAMM_STOP:
			LastCmd = COMMAND_PROGRAMM_STOP;
			Dispatcher_SetFlag(FLAG_RADIO_CMD);
			status_st.status = TABLET_IDLE;
			CC2500_FLUSH_AND_RX();
			break;
		case COMMAND_PROGRAMM_PAUSE:
			status_st.status = TABLET_PAUSED;
			CC2500_FLUSH_AND_RX();
			break;
		case COMMAND_PROGRAMM_NEXT_PERIOD:
			break;
		case COMMAND_PROGRAMM_PREV_PERIOD:
			break;
		case COMMAND_CHANGE_CHANNEL:
			set_Channel(spi_rx_st.packet.param);
			break;
		case COMMAND_TABLET_MCU_RESET:
			Log_W("RESET requested remotely!");
			Log_Flush();
			NVIC_SystemReset();
			break;
		default:
			status_st.status = TABLET_ERROR;
			sprintf(line, "Unknown command: 0x%02x", (int) spi_rx_st.packet.command);
			Log_E(line);
			CC2500_FLUSH_AND_RX(); // in all unclear situations - flush FIFOs!
	}
}

// change channel after reply
void set_Channel(uint8_t channel)
{
	cc_write_reg(TI_CCxxx0_CHANNR, channel);
    RADIO_TX();
}

ErrorStatus packet_filter(void)
{
	// flush FIFOs if CRC16 failed
	if (spi_rx_st.reply.FIFO == 0)
	{
		CC2500_FLUSH_AND_RX();
		return ERROR;
	}

	// source address control
	if (spi_rx_st.packet.source != DONGLE_ADDR)
	{
		CC2500_FLUSH_AND_RX();
		return ERROR;
	}

	return SUCCESS;
}

void get_Status(void)
{
	PowerReport_TypeDef pwrReport;

	lastQueryTime = GetSystemTime();

	Power_GetReport(&pwrReport);

	status_st.battery = (uint8_t)pwrReport.RepSoc;
	status_st.load = Bridge_GetResistance();;
	status_st.temp = pwrReport.Temp;

	RADIO_TX();
}

void programm_Load(void)
{
	uint8_t period_num = spi_rx_st.packet.data.index;

	if(spi_rx_st.packet.data.periodParams.n_period == 0xffff)
	{
		status_st.status = TABLET_READY;
	}
	else
	{
		maxPeriod = period_num;
	}

	Program_InsertPeriod(period_num-1, &spi_rx_st.packet.data);
	RADIO_TX();
}

void CC2500_FLUSH_AND_RX(void)
{
	SPI_WR_CC2500_DMA(SPI_buffer_RX, flush_and_rx, sizeof_flush_and_rx);
}

// signal from CC2500: packet from radio received
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(GDO2_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(GDO2_EXTI_LINE);
		Debug_SetPin(DEBUG_PIN1,DISABLE);
		NVIC_DisableIRQ(GDO2_IRQn);
		CC2500_flags_st.receiving_flag = 1;
		// get packet with DMA
		SPI_WR_CC2500_DMA((uint8_t*)&spi_rx_st, &SPI_GET_PACKET[0], RX_SPI_BUFFER_LENGTH);
	}
}

// packet get to SPI buffer
void DMA1_Stream3_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
	{
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		CC2500_DESELECT;
		// далее забираем данные из буфера SPI в буфер радио
		if(CC2500_flags_st.receiving_flag == 1)
		{
			CC2500_flags_st.receiving_flag = 0;
			processing();
		}
		else
		{
			EXTI_ClearITPendingBit(GDO2_EXTI_LINE);
			Debug_SetPin(DEBUG_PIN1, ENABLE);
			NVIC_EnableIRQ(GDO2_IRQn);
		}
	}
}

//---------INIT-------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void spi_pin_setup(void)
{
	GPIO_InitTypeDef GPIO_Init_SPI;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_Init_SPI.GPIO_Pin = SCK_PIN | MOSI_PIN | MISO_PIN;
	GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_SPI.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_SPI.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(SPI_PORT, &GPIO_Init_SPI);

	GPIO_Init_SPI.GPIO_Pin = CS_PIN;
	GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_SPI.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_SPI.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(CS_PORT, &GPIO_Init_SPI);
}

void spi_setup(void)
{
	GPIO_InitTypeDef GPIO_Init_SPI;
	SPI_InitTypeDef SPI_Init_Setup;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_Init_SPI.GPIO_Pin = SCK_PIN | MOSI_PIN | MISO_PIN;
	GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_SPI.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_SPI.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(SPI_PORT, &GPIO_Init_SPI);

	GPIO_PinAFConfig(SPI_PORT, SCK_PIN_SOURSE, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_PORT, MOSI_PIN_SOURSE, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_PORT, MISO_PIN_SOURSE, GPIO_AF_SPI2);

	GPIO_Init_SPI.GPIO_Pin = CS_PIN;
	GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_SPI.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_SPI.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(CS_PORT, &GPIO_Init_SPI);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_Init_Setup.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_Init_Setup.SPI_Mode = SPI_Mode_Master;
	SPI_Init_Setup.SPI_DataSize = SPI_DataSize_8b;
	SPI_Init_Setup.SPI_CPOL = SPI_CPOL_Low;
	SPI_Init_Setup.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_Init_Setup.SPI_NSS = SPI_NSS_Soft;
	SPI_Init_Setup.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_Init_Setup.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init_Setup.SPI_CRCPolynomial = 7;

	SPI_Init(SPI2, &SPI_Init_Setup);
	SPI_Cmd(SPI2, ENABLE);

	SPI_DMA_setup();
}

void SPI_DMA_setup (void)
{
	DMA_InitTypeDef DMA_Init_SPI;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	//TX
	DMA_Init_SPI.DMA_Channel = DMA_Channel_0;
	DMA_Init_SPI.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);
	DMA_Init_SPI.DMA_Memory0BaseAddr = (uint32_t)SPI_buffer_TX;
	DMA_Init_SPI.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init_SPI.DMA_BufferSize = sizeof(SPI_buffer_TX);
	DMA_Init_SPI.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init_SPI.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Init_SPI.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_Init_SPI.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Init_SPI.DMA_Mode = DMA_Mode_Normal;
	DMA_Init_SPI.DMA_Priority = DMA_Priority_Medium;
	DMA_Init_SPI.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init_SPI.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_Init_SPI.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_Init_SPI.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream4, &DMA_Init_SPI);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	//RX
	DMA_Init_SPI.DMA_Channel = DMA_Channel_0;
	DMA_Init_SPI.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);
	DMA_Init_SPI.DMA_Memory0BaseAddr = (uint32_t)SPI_buffer_RX;
	DMA_Init_SPI.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init_SPI.DMA_BufferSize = sizeof(SPI_buffer_RX);
	DMA_Init_SPI.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init_SPI.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Init_SPI.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_Init_SPI.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Init_SPI.DMA_Mode = DMA_Mode_Normal;
	DMA_Init_SPI.DMA_Priority = DMA_Priority_Medium;
	DMA_Init_SPI.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init_SPI.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_Init_SPI.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_Init_SPI.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream3, &DMA_Init_SPI);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_SetPriority(DMA1_Stream3_IRQn, DMA1_Stream3_IRQ_PRIORITY);

    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
}

void Delay_init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	nCount = (RCC_Clocks.HCLK_Frequency / 10000000);
}

void Delay_us(uint32_t us)
{
	volatile uint32_t nCountTmp;
	nCountTmp = nCount*us;
    for (; nCountTmp!=0; nCountTmp--);
}

void EXTI_GDO2_Init (void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	GDO2_EXTI_RCC_EN;
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// GDO_2
	/* Configure GDO0_PIN as input floating, pulling to GND through the internal resistor */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GDO2_PIN;
	GPIO_Init(GDO2_PORT, &GPIO_InitStructure);

	/* Connect EXTI Line to GDO0_PIN */
	SYSCFG_EXTILineConfig(GDO2_EXTI_PORT, GDO2_EXTI_PIN);

	/* Configure EXTI Line */
	EXTI_InitStructure.EXTI_Line = GDO2_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*set EXTI priority */ // enable after all settings load to CC2500
	NVIC_SetPriority(GDO2_IRQn, EXTI2_IRQ_PRIORITY);
}


uint8_t spi_tx_rx(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI2);
}

void cc_write_reg(uint8_t addr, uint8_t value)
{
	cc_select_and_wait_for_rdy();
	spi_tx_rx(addr);
	spi_tx_rx(value);
	CC2500_DESELECT;       // /CS disable
}

void cc_select_and_wait_for_rdy(void)
{
	CC2500_SELECT;
	while (GPIO_ReadInputDataBit(MISO_PORT, MISO_PIN));  // Wait for CC2500 ready
}

// DMA SPI TX - not used
void DMA1_Stream4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) == SET)
	{
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
	}
}

uint32_t Radio_IsConnected()
{
	uint32_t now = GetSystemTime();
	FlagStatus ovr = Dispatcher_GetFlag(FLAG_RADIO_OVERRIDE);
	return ovr?(1):((now - lastQueryTime) < MAX_ALLOWED_PAUSE);
}
