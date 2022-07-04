
#include <string.h>
#include "stm32f4xx.h"

#ifndef _CC2500_H
#define _CC2500_H

// Packet parameters -------------------------------------------
#define FIFO_LENGTH 64
#define CRC_LENGTH 2
#define PACKET_LENGTH (FIFO_LENGTH - CRC_LENGTH)
#define RX_SPI_BUFFER_LENGTH (FIFO_LENGTH + 1)
#define TX_SPI_BUFFER_LENGTH (PACKET_LENGTH + sizeof(header_tx))
//----------------------------------------------------------------

#define	END_PACKET 0xFE // special address

// GPIO definitions
#define GDO2_PORT         (GPIOB)
#define GDO2_PIN          (GPIO_Pin_2)
#define GDO2_EXTI_PIN     (EXTI_PinSource2)   //GDO2 interrupt
#define GDO2_EXTI_PORT    (EXTI_PortSourceGPIOB)
#define GDO2_EXTI_LINE    (EXTI_Line2)
#define GDO2_IRQn         (EXTI2_IRQn)
#define GDO2_IRQ_PrePrior (0x0F)
#define GDO2_IRQ_SubPrior (0x0F)
#define GDO2_EXTI_FLAG    (EXTI_Line2) //flag of GDO2 interrupt
#define GDO2_EXTI_RCC_EN (RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE))

#define SPI_PORT (GPIOB)

#define CS_PORT (GPIOB)
#define CS_PIN (GPIO_Pin_12)
#define CS_PIN_SOURSE (GPIO_PinSource12)

#define SCK_PORT (GPIOB)
#define SCK_PIN (GPIO_Pin_10)
#define SCK_PIN_SOURSE (GPIO_PinSource10)
#define SCK_LOW (GPIO_ResetBits(SCK_PORT, SCK_PIN))
#define SCK_HI (GPIO_SetBits(SCK_PORT, SCK_PIN))

#define MOSI_PORT (GPIOB)
#define MOSI_PIN (GPIO_Pin_15)
#define MOSI_PIN_SOURSE (GPIO_PinSource15)
#define MOSI_LOW (GPIO_ResetBits(MOSI_PORT, MOSI_PIN))
#define MOSI_HI (GPIO_SetBits(MOSI_PORT, MOSI_PIN))

#define MISO_PORT (GPIOB)
#define MISO_PIN (GPIO_Pin_14)
#define MISO_PIN_SOURSE (GPIO_PinSource14)
#define MISO_LOW (GPIO_ResetBits(MISO_PORT, MISO_PIN))
#define MISO_HI (GPIO_SetBits(MISO_PORT, MISO_PIN))

#define CC2500_SELECT  (GPIO_ResetBits(CS_PORT, CS_PIN))
#define CC2500_DESELECT  (GPIO_SetBits(CS_PORT, CS_PIN))

// CC2500 Configuration Registers
#define TI_CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define TI_CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define TI_CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define TI_CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define TI_CCxxx0_SYNC1        0x04        // Sync word, high byte
#define TI_CCxxx0_SYNC0        0x05        // Sync word, low byte
#define TI_CCxxx0_PKTLEN       0x06        // Packet length
#define TI_CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define TI_CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define TI_CCxxx0_ADDR         0x09        // Device address
#define TI_CCxxx0_CHANNR       0x0A        // Channel number
#define TI_CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define TI_CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define TI_CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define TI_CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define TI_CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define TI_CCxxx0_MDMCFG4      0x10        // Modem configuration
#define TI_CCxxx0_MDMCFG3      0x11        // Modem configuration
#define TI_CCxxx0_MDMCFG2      0x12        // Modem configuration
#define TI_CCxxx0_MDMCFG1      0x13        // Modem configuration
#define TI_CCxxx0_MDMCFG0      0x14        // Modem configuration
#define TI_CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define TI_CCxxx0_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation config
#define TI_CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define TI_CCxxx0_AGCCTRL2     0x1B        // AGC control
#define TI_CCxxx0_AGCCTRL1     0x1C        // AGC control
#define TI_CCxxx0_AGCCTRL0     0x1D        // AGC control
#define TI_CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define TI_CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define TI_CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define TI_CCxxx0_FREND1       0x21        // Front end RX configuration
#define TI_CCxxx0_FREND0       0x22        // Front end TX configuration
#define TI_CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define TI_CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define TI_CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define TI_CCxxx0_FSTEST       0x29        // Frequency synthesizer cal control
#define TI_CCxxx0_PTEST        0x2A        // Production test
#define TI_CCxxx0_AGCTEST      0x2B        // AGC test
#define TI_CCxxx0_TEST2        0x2C        // Various test settings
#define TI_CCxxx0_TEST1        0x2D        // Various test settings
#define TI_CCxxx0_TEST0        0x2E        // Various test settings

// CC2500 Strobe commands
#define TI_CCxxx0_SRES         0x30        // Reset chip.
#define TI_CCxxx0_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define TI_CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define TI_CCxxx0_SCAL         0x33        // Calibrate freq synthesizer & disable
#define TI_CCxxx0_SRX          0x34        // Enable RX.
#define TI_CCxxx0_STX          0x35        // Enable TX.
#define TI_CCxxx0_SIDLE        0x36        // Exit RX / TX
#define TI_CCxxx0_SAFC         0x37        // AFC adjustment of freq synthesizer
#define TI_CCxxx0_SWOR         0x38        // Start automatic RX polling sequence
#define TI_CCxxx0_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define TI_CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define TI_CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define TI_CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define TI_CCxxx0_SNOP         0x3D        // No operation.

// CC2500 Status registers
#define TI_CCxxx0_PARTNUM      0x30        // Part number
#define TI_CCxxx0_VERSION      0x31        // Current version number
#define TI_CCxxx0_FREQEST      0x32        // Frequency offset estimate
#define TI_CCxxx0_LQI          0x33        // Demodulator estimate for link quality
#define TI_CCxxx0_RSSI         0x34        // Received signal strength indication
#define TI_CCxxx0_MARCSTATE    0x35        // Control state machine state
#define TI_CCxxx0_WORTIME1     0x36        // High byte of WOR timer
#define TI_CCxxx0_WORTIME0     0x37        // Low byte of WOR timer
#define TI_CCxxx0_PKTSTATUS    0x38        // Current GDOx status and packet status
#define TI_CCxxx0_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define TI_CCxxx0_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define TI_CCxxx0_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define TI_CCxxx0_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// CC2500 Other memory locations
#define TI_CCxxx0_PATABLE      0x3E
#define TI_CCxxx0_TXFIFO       0x3F
#define TI_CCxxx0_RXFIFO       0x3F

// CC2500 Masks for appended status bytes
#define TI_CCxxx0_LQI_RX       0x01        // Position of LQI byte
#define TI_CCxxx0_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// CC2500 Definitions to support burst/single access:
#define TI_CCxxx0_WRITE_BURST  0x40
#define TI_CCxxx0_READ_SINGLE  0x80
#define TI_CCxxx0_READ_BURST   0xC0

typedef enum {
	TABLET_NOT_CONNECTED = 0,
	TABLET_READY = 1,
	TABLET_IDLE = 2,
	TABLET_STIMUL = 3,
	TABLET_PAUSED = 4,
	TABLET_ERROR = 5,
	TABLET_LOADING = 6
} TabletStatus;

typedef enum {
	CC2500_READY = 0,
	CC2500_NOT_READY = 1
} CC2500_RDYn;

typedef enum {
	CC2500_IDLE = 0,
	CC2500_RX = 1,
	CC2500_TX = 2,
	CC2500_FSTX_ON = 3,
	CC2500_CALIBRATE = 4,
	CC2500_SETTLING = 5,
	CC2500_RXFIFO_OVERFLOW = 6,
	CC2500_TXFIFO_UNDERFLOW = 7
} CC2500_STATE;

typedef enum {
	COMMAND_NOP = 0x00,
	COMMAND_SPECIAL = 0x06,
	COMMAND_GET_STATUS = 0x0C,
	COMMAND_PROGRAMM_LOAD = 0x0D,
	COMMAND_PROGRAMM_START = 0x0E,
	COMMAND_PROGRAMM_STOP = 0x0F,
	COMMAND_PROGRAMM_PAUSE = 0x10,
	COMMAND_PROGRAMM_NEXT_PERIOD = 0x11,
	COMMAND_PROGRAMM_PREV_PERIOD = 0x12,
	COMMAND_TABLET_MCU_RESET = 0x13,
    COMMAND_SYNC = 0x14,
    COMMAND_CHANGE_CHANNEL = 0x15
	} command_list;

typedef struct __attribute__((packed)) {
	TabletStatus status;
	uint8_t signal;
	uint8_t battery;
	uint16_t load;
	uint8_t temp; // temperature in case
	uint8_t act_period; // actual period number
	uint8_t act_repeat; // actual repeat number
	uint16_t act_time; // actual time of period, sec
} dataFromTablet ;

ProgramPeriod_TypeDef program_st[100];

dataFromTablet status_st;

extern uint8_t Settings[];
extern const size_t sizeof_Settings;

typedef struct __attribute__((packed)){
  uint8_t receiving_flag;
  uint8_t transmitting_flag;
  uint8_t settings_flag;
} CC2500_flags;

CC2500_flags CC2500_flags_st;

typedef struct __attribute__((packed)){
 uint8_t destination;  // Packet destination
 uint8_t source;       // Packet source
 uint16_t number; // packet number
 uint8_t payload_size;
 command_list command; // payload part
 uint8_t param;
 ProgramPeriod_TypeDef data; // payload part
} packet_rx;

typedef struct __attribute__((packed)){
  uint8_t FIFO : 4;
  CC2500_STATE STATE : 3;
  CC2500_RDYn CHIP_RDYn : 1;
} spi_reply;

typedef struct __attribute__((packed)){
 spi_reply reply;    // status of cc2500
 packet_rx packet;
 uint8_t reserved_0;
 uint32_t reserved_1;
 uint64_t reserved_3;
 uint64_t reserved_4;
 uint8_t rssi; // Radio Signal Strength Indicator
 uint8_t lqi : 7; // Link Quality Indicator
 uint8_t crc_ok : 1; // flag, not used because may be not correct, see ERRATA
} spi_rx;
spi_rx spi_rx_st;

typedef struct __attribute__((packed)){
  uint8_t sidle; // IDLE command
  uint8_t sfrx; // clear RX FIFO
  uint8_t sftx; // clear TX FIFO
  uint8_t stx; // cc2500 is going to TX
  uint8_t txfifo; // burst write to TX FIFO command
} header_tx;

typedef struct __attribute__((packed)){
  uint8_t d_addr; // to
  uint8_t t_addr; // from
} addrs_tx;

typedef struct __attribute__((packed)){
  addrs_tx addrs;
  dataFromTablet status;
  packet_rx copy_rx;
} packet_tx;

typedef struct __attribute__((packed)){
 header_tx header;
 packet_tx packet;
 uint8_t reserved_0;
 uint64_t reserved_2;
} spi_tx;

spi_tx spi_tx_st;

ErrorStatus CC2500_Init(void);
ErrorStatus CC2500_struct_init (void);
ErrorStatus CC2500_powerup_reset(void);
void spi_pin_setup(void);
void spi_setup(void);
void SPI_DMA_setup (void);
void Delay_init (void);
void Delay_us (uint32_t);
void SPI_WR_CC2500_DMA (uint8_t*, uint8_t*, uint16_t);
void EXTI_GDO2_Init (void);
void processing(void);
void get_Status(void);
void programm_Load (void);
void RADIO_TX (void);
uint8_t spi_tx_rx(uint8_t);
void cc_write_reg(uint8_t, uint8_t);
void cc_select_and_wait_for_rdy(void);
void CC2500_FLUSH_AND_RX(void);
ErrorStatus packet_filter(void);
command_list Radio_GetCommand();
uint32_t Radio_IsConnected();
void set_Channel(uint8_t channel);

#endif /* _CC2500_H */
