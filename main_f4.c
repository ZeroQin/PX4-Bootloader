/*
 * STM32F4 board support for the bootloader.
 *
 */

#include "hw_config.h"


#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include "bl.h"
#include "uart.h"

/* flash parameters that we should not really know */
/* flash_sector 用于描述 STM32F4 芯片内部 flash 的结构,STM32F4 片内 flash 大小为 1M 或 2M 两种，具体参见芯片手册*/
static struct {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = {

	/* Physical FLASH sector 0 is reserved for bootloader and is not
	 * the table below.
	 * N sectors may aslo be reserved for the app fw in which case
	 * the zero based define BOARD_FIRST_FLASH_SECTOR_TO_ERASE must
	 * be defined to begin the erase above of the reserved sectors.
	 * The default value of BOARD_FIRST_FLASH_SECTOR_TO_ERASE is 0
	 * and begins flash erase operations at phsical sector 1 the 0th entry
	 * in the table below.
	 * A value of 1 for BOARD_FIRST_FLASH_SECTOR_TO_ERASE would reserve
	 * the 0th entry and begin erasing a index 1 the third physical sector
	 * on the device.
	 *
	 * When BOARD_FIRST_FLASH_SECTOR_TO_ERASE is defined APP_RESERVATION_SIZE
	 * must also be defined to remove that additonal reserved FLASH space
	 * from the BOARD_FLASH_SIZE. See APP_SIZE_MAX below.
	 */

	{0x01, 16 * 1024},
	{0x02, 16 * 1024},
	{0x03, 16 * 1024},
	{0x04, 64 * 1024},
	{0x05, 128 * 1024},
	{0x06, 128 * 1024},
	{0x07, 128 * 1024},
	{0x08, 128 * 1024},
	{0x09, 128 * 1024},
	{0x0a, 128 * 1024},
	{0x0b, 128 * 1024},
	/* flash sectors only in 2MiB devices */
	{0x10, 16 * 1024},
	{0x11, 16 * 1024},
	{0x12, 16 * 1024},
	{0x13, 16 * 1024},
	{0x14, 64 * 1024},
	{0x15, 128 * 1024},
	{0x16, 128 * 1024},
	{0x17, 128 * 1024},
	{0x18, 128 * 1024},
	{0x19, 128 * 1024},
	{0x1a, 128 * 1024},
	{0x1b, 128 * 1024},
};
#define BOOTLOADER_RESERVATION_SIZE	(16 * 1024)

#define OTP_BASE			0x1fff7800
#define OTP_SIZE			512
#define UDID_START		        0x1FFF7A10

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000
#define STM32_UNKNOWN	0
#define STM32F40x_41x	0x413
#define STM32F42x_43x	0x419
#define STM32F42x_446xx	0x421

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

// A board may disable VBUS sensing, but still provide a (non-standard) VBUS
// sensing pin (and use it for fast booting when USB is disconnected). If VBUS
// sensing is enabled, only PA9 can be used.

#ifndef BOARD_USB_VBUS_SENSE_DISABLED
# define BOARD_PORT_VBUS                GPIOA
# define BOARD_PIN_VBUS                 GPIO9
#endif

/* magic numbers from reference manual */

typedef enum mcu_rev_e {
	MCU_REV_STM32F4_REV_A = 0x1000,
	MCU_REV_STM32F4_REV_Z = 0x1001,
	MCU_REV_STM32F4_REV_Y = 0x1003,
	MCU_REV_STM32F4_REV_1 = 0x1007,
	MCU_REV_STM32F4_REV_3 = 0x2001
} mcu_rev_e;

/* 结构体 mcu_des_t 存储了 STM32F4 类 MCU 的所有型号信息，
*  并被实体化为变量数组 mcu_descriptions，此结构用于程序中自动识别当前 MCU 类型。
*  MCU 类型信息被存储在 DBGMCU_IDCODE 寄存器中，地址为 0xE0042000。
*  此结构体在 check_silicon 函数中使用。*/
typedef struct mcu_des_t {
	uint16_t mcuid;
	const char *desc;
	char  rev;
} mcu_des_t;

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
mcu_des_t mcu_descriptions[] = {
	{ STM32_UNKNOWN,	"STM32F???",    '?'},
	{ STM32F40x_41x, 	"STM32F40x",	'?'},
	{ STM32F42x_43x, 	"STM32F42x",	'?'},
	{ STM32F42x_446xx, 	"STM32F446XX",	'?'},
};

/* 结构体 mcu_rev_t 存储了 MCU 的版本信息，它被定义在 main_f4.c 文件中，
*  并被实体化为变量数组 silicon_revs，此结构除了可以识别处理器版本外，
*  还可以据此判断 MCU 的内部 flash 信息。*/
typedef struct mcu_rev_t {
	mcu_rev_e revid;
	char  rev;
} mcu_rev_t;

/*
 * This table is used in 2 ways. One to look look up the revision
 * of a given chip. Two to see it a revsion is in the group of "Bad"
 * silicon.
 *
 * Therefore when adding entries for good silicon rev, they must be inserted
 * at the beginning of the table. The value of FIRST_BAD_SILICON_OFFSET will
 * also need to be increased to that of the value of the first bad silicon offset.
 *
 */
const mcu_rev_t silicon_revs[] = {
	{MCU_REV_STM32F4_REV_3, '3'}, /* Revision 3 */

	{MCU_REV_STM32F4_REV_A, 'A'}, /* Revision A */  // FIRST_BAD_SILICON_OFFSET (place good ones above this line and update the FIRST_BAD_SILICON_OFFSET accordingly)
	{MCU_REV_STM32F4_REV_Z, 'Z'}, /* Revision Z */
	{MCU_REV_STM32F4_REV_Y, 'Y'}, /* Revision Y */
	{MCU_REV_STM32F4_REV_1, '1'}, /* Revision 1 */
};

#define FIRST_BAD_SILICON_OFFSET 1

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,/*BOARD_TYPE板载处理器编号在hw_config.h中定义，V2主控为9。 */
	.board_rev	= 0,/* board_rev，修订版本为0 */
	.fw_size	= 0,/* fw_size，飞控固件大小的最大值，这里的0没有意义，在board_init函数中被重新初始化 */

	.systick_mhz	= 168,/* systick_mhz，系统时钟输入，即CPU主频168MHz */
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                MMIO32(RTC_BASE + 0x50)

/* standard clocking for all F4 boards */
static const struct rcc_clock_scale clock_setup = {
	.pllm = OSC_FREQ,
	.plln = 336,
	.pllp = 2,
	.pllq = 7,
#if defined(STM32F446) || defined(STM32F469)
	.pllr = 2,
#endif
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_4,
	.ppre2 = RCC_CFGR_PPRE_DIV_2,
	.power_save = 0,
	.flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS,
	.apb1_frequency = 42000000,
	.apb2_frequency = 84000000,
};

/* State of an inserted USB cable */
static bool usb_connected = false;

static uint32_t
board_get_rtc_signature()
{
	/* enable the backup registers */
	PWR_CR |= PWR_CR_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	uint32_t result = BOOT_RTC_REG;

	/* disable the backup registers */
	PWR_CR &= ~PWR_CR_DBP;

	return result;
}

static void
board_set_rtc_signature(uint32_t sig)
{
	/* enable the backup registers */
	PWR_CR |= PWR_CR_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	BOOT_RTC_REG = sig;

	/* disable the backup registers */
	PWR_CR &= ~PWR_CR_DBP;
}

static bool
board_test_force_pin()
{
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* two pins strapped together */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
		gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);

		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) != 0) {
				vote++;
			}

			samples++;
		}

		gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);

		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) == 0) {
				vote++;
			}

			samples++;
		}
	}

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
#if defined(BOARD_FORCE_BL_PIN)
	/* single pin pulled up or down */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (samples = 0; samples < 200; samples++) {
		if ((gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN) ? 1 : 0) == BOARD_FORCE_BL_STATE) {
			vote++;
		}
	}

	/* reject a little noise */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
	return false;
}

#if INTERFACE_USART
static bool
board_test_usart_receiving_break()
{
#if !defined(SERIAL_BREAK_DETECT_DISABLED)
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* Set the timer period to be half the bit rate
	 *
	 * Baud rate = 115200, therefore bit period = 8.68us
	 * Half the bit rate = 4.34us
	 * Set period to 4.34 microseconds (timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729)
	 */
	systick_set_reload(((board_info.systick_mhz * 1000000) / USART_BAUDRATE) >> 1);
	systick_counter_enable(); // Start the timer

	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/* Loop for 3 transmission byte cycles and count the low and high bits. Sampled at a rate to be able to count each bit twice.
	 *
	 * One transmission byte is 10 bits (8 bytes of data + 1 start bit + 1 stop bit)
	 * We sample at every half bit time, therefore 20 samples per transmission byte,
	 * therefore 60 samples for 3 transmission bytes
	 */
	while (cnt < 60) {
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1) {
			if (gpio_get(BOARD_PORT_USART_RX, BOARD_PIN_RX) == 0) {
				cnt_consecutive_low++;	// Increment the consecutive low counter

			} else {
				cnt_consecutive_low = 0; // Reset the consecutive low counter
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if (cnt_consecutive_low >= 18) {
			break;
		}

	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */
	if (cnt_consecutive_low >= 18) {
		return true;
	}

#endif // !defined(SERIAL_BREAK_DETECT_DISABLED)

	return false;
}
#endif

uint32_t
board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;
#if defined(TARGET_HW_PX4_FMU_V2) || defined(TARGET_HW_PX4_FMU_V3) || defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)

	if (check_silicon() && board_info.fw_size == (2 * 1024 * 1024) - BOOTLOADER_RESERVATION_SIZE) {
		board_info.fw_size = (1024 * 1024) - BOOTLOADER_RESERVATION_SIZE;
	}

#endif

#if defined(BOARD_POWER_PIN_OUT)
	/* Configure the Power pins */
	rcc_peripheral_enable_clock(&BOARD_POWER_CLOCK_REGISTER, BOARD_POWER_CLOCK_BIT);
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN_OUT);
	gpio_set_output_options(BOARD_POWER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BOARD_POWER_PIN_OUT);
	BOARD_POWER_ON(BOARD_POWER_PORT, BOARD_POWER_PIN_OUT);
#endif

#if INTERFACE_USB
#if !defined(BOARD_USB_VBUS_SENSE_DISABLED)
	/* enable configured GPIO to sample VBUS */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
#endif
#endif

#if INTERFACE_USART
	/* configure USART pins */
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT_TX);
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT_RX);

	/* Setup GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART_TX, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_TX);
	gpio_mode_setup(BOARD_PORT_USART_RX, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_RX);
	/* Setup USART TX & RX pins as alternate function. */
	gpio_set_af(BOARD_PORT_USART_TX, BOARD_PORT_USART_AF_TX, BOARD_PIN_TX);
	gpio_set_af(BOARD_PORT_USART_RX, BOARD_PORT_USART_AF_RX, BOARD_PIN_RX);

	/* configure USART clock */
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN_IN);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_set_output_options(BOARD_FORCE_BL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, BOARD_FORCE_BL_PIN_OUT);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);
#endif

	/* initialise LEDs */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LEDS);
	gpio_mode_setup(
		BOARD_PORT_LEDS,
		GPIO_MODE_OUTPUT,
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	gpio_set_output_options(
		BOARD_PORT_LEDS,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_2MHZ,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* enable the power controller clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);
}

void
board_deinit(void)
{

#if INTERFACE_USART
	/* deinitialise GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART_TX, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX);
	gpio_mode_setup(BOARD_PORT_USART_RX, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_RX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_IN);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN);
#endif

	/* deinitialise LEDs */
	gpio_mode_setup(
		BOARD_PORT_LEDS,
		GPIO_MODE_INPUT,
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* disable the power controller clock */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

	/* disable the AHB peripheral clocks */
	RCC_AHB1ENR = 0x00100000; // XXX Magic reset number from STM32F4x reference manual
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
	rcc_clock_setup_hse_3v3(&clock_setup);
}

inline void arch_systic_init(void)
{
	/* (re)start the timer system */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);  /* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
}

inline void arch_systic_deinit(void)
{
	/* kill the systick interrupt */
	systick_interrupt_disable();
	systick_counter_disable();
}

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL and PLLI2S OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  */
void
clock_deinit(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(RCC_HSE);
	rcc_osc_off(RCC_PLL);
	rcc_css_disable();

	/* Reset the RCC_PLLCFGR register */
	RCC_PLLCFGR = 0x24003010; // XXX Magic reset number from STM32F4xx reference manual

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

inline void arch_flash_lock(void)
{
	flash_lock();
}

inline void arch_flash_unlock(void)
{
	flash_unlock();
}

inline void arch_setvtor(uint32_t address)
{
	SCB_VTOR = address;
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return flash_sectors[sector].size;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	/* Caculate the logical base address of the sector
	 * flash_func_read_word will add APP_LOAD_ADDRESS
	 */
	uint32_t address = 0;

	for (unsigned i = BOARD_FIRST_FLASH_SECTOR_TO_ERASE; i < sector; i++) {
		address += flash_func_sector_size(i);
	}

	/* blank-check the sector */
	unsigned size = flash_func_sector_size(sector);
	bool blank = true;

	for (unsigned i = 0; i < size; i += sizeof(uint32_t)) {
		if (flash_func_read_word(address + i) != 0xffffffff) {
			blank = false;
			break;
		}
	}

	/* erase the sector if it failed the blank check */
	if (!blank) {
		flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
	}
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word);
}

uint32_t
flash_func_read_word(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	if (address > OTP_SIZE) {
		return 0;
	}

	return *(uint32_t *)(address + OTP_BASE);
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	int32_t mcuid = idcode & DEVID_MASK;
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	mcu_des_t des = mcu_descriptions[STM32_UNKNOWN];

	for (int i = 0; i < arraySize(mcu_descriptions); i++) {
		if (mcuid == mcu_descriptions[i].mcuid) {
			des = mcu_descriptions[i];
			break;
		}
	}

	for (int i = 0; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			des.rev = silicon_revs[i].rev;
		}
	}

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *des.desc) {
		*strp++ = *des.desc++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	if (strp < endp) {
		*strp++ = des.rev;
	}

	return  strp - revstr;
}


int check_silicon(void)
{
#if defined(TARGET_HW_PX4_FMU_V2)  || defined(TARGET_HW_PX4_FMU_V3) || defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	for (int i = FIRST_BAD_SILICON_OFFSET; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			return -1;
		}
	}

#endif
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	// read a byte out from unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address + UDID_START);
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
		break;

	case LED_BOOTLOADER:
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
		break;
	}
}

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

int
main(void)
{
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	/* bootloader是否立即跳转到飞控固件入口的标志（不等待timeout） */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */
	/* bootloader初始化完毕后跳转到飞控固件入口地址时，所需等待的时间，以ms计算。
	*宏BOOTLOADER_DELAY在hw_config.h中被定义为5000 */
	
	/* Enable the FPU before we hit any FP instructions */
	/* 使能浮点运算，其功能与pre_main函数相同，实际上可以去掉。 */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

#if defined(BOARD_POWER_PIN_OUT)/* 宏BOARD_POWER_PIN_OUT未定义，下面函数不编译 */

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	 
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	/* do board-specific initialisation */
	/* 调用board_init函数，此函数在main_f4.c中被定义，功能如下： */
	/* 1. 赋值board_info.fw_size，确定飞控固件size允许的最大值 */
	/* 2. 初始化USB端口GPIOA9（VBUS） */
	/* 3. 初始化串口USART2 */
	/* 4. 初始化LED并点亮（B/E，LED701） */
	/* 5. 使能能耗控制器时钟 */

	board_init();

	/* configure the clock for bootloader activity */
	
	/* 调用clock_init函数，此函数在main_f4.c中被定义，功能如下： */
	/* 1. PLL时钟设置，并选择main PLL作为系统时钟sysclk，fsysclk=fPLL=168MHz，fUSBSDRNG=48MHz */
	/* 2. AHB、APB1、APB2时钟设置，fAHB=fsysclk=168MHz，fAPB1=42MHz，fAPB2=84MHz */
	/* 3. 选择高能耗模式（Scale 2 mode） */
	/* 4. flash访问控制设置，启动Icache和Dcache，并设置等待时间5周期 */
	/* 5. 更新库libopencm3中AHB，APB1，APB2对应的全局变量 */
	clock_init();

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	 	/* 检查寄存器BOOT_RTC_REG中的值是否与宏BOOT_RTC_SIGNATURE相同，若相同则停留在bootloader中，若不相同则可以跳转至飞控固件 */
	/* BOOT_RTC_REG：RTC_BKPxR的第一个32位寄存器，重启不改变存储值，定义在main_f4.c */
	/* BOOT_RTC_SIGNATURE：值0xb007b007，被定义在main_f4.c中 */
	/* board_get_rtc_signature：被定义在main_f4.c中，用于获取寄存器BOOT_RTC_REG的存储值 */
	/* board_set_rtc_signature：被定义在main_f4.c中，设置寄存器BOOT_RTC_REG的值 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

#ifdef BOOT_DELAY_ADDRESS/* 宏BOOT_DELAY_ADDRESS在hw_config.h中有定义，下列代码有效 *
	{
		/*
		  if a boot delay signature is present then delay the boot
		  by at least that amount of time in seconds. This allows
		  for an opportunity for a companion computer to load a
		  new firmware, while still booting fast by sending a BOOT
		  command
		 */
		 
		/* 这里给定一个机会，通过飞控固件自身的设置可以影响bootloader的行为。 */
		/* 设置地址在0x080041a0和0x080041a4的flash值满足一系列逻辑要求，可以防止bootloader自动跳转到飞控固件，给调试工作带来方便。 */
		/* BOOT_DELAY_ADDRESS：值0x000001a0，定义在hw_config.h */
		/* flash_func_read_word：定义在main_f4.c，用于读取飞控固件内某地址 */
		/* 这里，sig1为flash中地址在0x080041a0的32位值，sig2为flash中地址在0x080041a4的32位地址值，在飞控固件地址范围内 */
		uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
		uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);
		//sig1的后两位可由飞控固件设置

		/* BOOT_DELAY_SIGNATURE1：值0x92c2ecff，定义在bl.h */
		/* BOOT_DELAY_SIGNATURE2：值0xc5057d5d，定义在bl.h */
		/* BOOT_DELAY_MAX：值30，定义在bl.h */
		/* 在满足一系列逻辑下，设定try_boot为假，且重新设置timeout，保证跳转到飞控固件前等待timeout时间 */
		if (sig2 == BOOT_DELAY_SIGNATURE2 &&
		    (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) {
			unsigned boot_delay = sig1 & 0xFF;

			if (boot_delay <= BOOT_DELAY_MAX) {
				try_boot = false;

				if (timeout < boot_delay * 1000) {
					timeout = boot_delay * 1000;
				}
			}
		}
	}
#endif

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	 /* board_test_force_pin：定义在main_f4.c，若bootloader引脚为真，
	 * 则设置try_boot为假，程序不立即跳转至飞控固件，需等待timeout时间 
	*  由于函数中条件编译的3个宏（BOARD_FORCE_BL_PIN_OUT、BOARD_FORCE_BL_PIN_IN
	*  和BOARD_FORCE_BL_PIN）均未被定义，此函数返回恒为假 */
	if (board_test_force_pin()) {
		try_boot = false;
	}

	/* 检查USB是否连接，若连接则必须等待timeout再跳转到飞控固件，
	*否则立即跳转。这样的设置为飞控固件烧写以及调试带来极大方便。 */
#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
#if defined(BOARD_PORT_VBUS)

	/* GPIOA：地址为0x40020000的寄存器（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO9：值1<<9（libopencm3/include/libopencm3/stm32/common/gpio_common_all.h） */
	/* gpio_get：获取某GPIO组的值，定义在libopencm3/lib/stm32/common/gpio_common_all.c */
	/* 根据原理图，GPIOA9对应VBUS/3.1A，高电平表示USB已连接，低电平表示USB未接 *
	if (gpio_get(BOARD_PORT_VBUS, BOARD_PIN_VBUS) != 0) {
		usb_connected = true;
		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#else
	try_boot = false;

#endif
#endif

	/* 检测串口是否收到break信号（字节0），若收到置try_boot为假，
	*需等待timeout时间再跳转到飞控固件 */

#if INTERFACE_USART

	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes,
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if (board_test_usart_receiving_break()) {
		try_boot = false;
	}

#endif

	/* 若运行到此处try_boot依然为真，则立即跳转到飞控固件
	*（实际上jump_to_app函数代码依然不短，这里可以近似这样认为）； 
	*  若跳转未成功，则设置RTC备份寄存器BOOT_RTC_REG为预定值，
	*  确保下次重启若不更新飞控固件依然无法跳转。 */


	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		/* 宏BOARD_BOOT_FAIL_DETECT未定义，以下代码无效 */
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();/* jump_to_app：定义在bl.c，跳转到飞控固件 */

		// If it failed to boot, reset the boot signature and stay in bootloader/* BOOT_RTC_SIGNATURE： */
		/* board_set_rtc_signature： */
		/* 设置RTC备份寄存器BOOT_RTC_REG值为BOOT_RTC_SIGNATURE，
		*  下次重启检测到后若不更新飞控固件则不跳转，程序一直运行在bootloader中。 */
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		/* 置timeout为0，程序一直在bootloader中 */
		timeout = 0;
	}


	/* start the interface */
	/* 现在bootloader首次启动飞控固件失败，初始化上位机通信接口（USB和USART2） */
#if INTERFACE_USART/* 宏INTERFACE_USART定义在hw_config.h，下列代码有效 */
	/* BOARD_INTERFACE_CONFIG_USART：值USART2（值0x40004400，指向USART2寄存器首地址）。 */
	/* BOARD_INTERFACE_CONFIG_USART在main_f4.c中被定义为BOARD_USART，BOARD_USART在hw_config.h中被定义为USART2。*/
	/* USART2被定义为USART2_BASE，值0x40004400，指向USART2寄存器首地址（libopencm3/include/libopencm3/stm32/common/usart_common_all.h）。 */
	/* USART：值1，枚举型，定义在bl.h */
	/* cinit：定义在bl.c，用于初始化通信端口 */
	/* 初始化串口USART2为与上位机的通信接口 */

	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB/* 宏INTERFACE_USB定义在hw_config.h，下列代码有效 */
	/* BOARD_INTERFACE_CONFIG_USB：值NULL，定义在main_f4.c */
	/* USB：值2，枚举型，定义在bl.h */
	/* cinit：定义在bl.c，用于初始化通信端口 */
	/* 初始化USB为虚拟串口作为与上位机的通信接口 */
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif


#if 0/* 下列代码无效 */
	// MCO1/02
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF0, GPIO9);
#endif


	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		/* bootloader：bootloader与上位机的命令处理函数，
		*烧写新的固件或者timeout（不为0）时间到返回，定义在bl.c中 */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		/* board_test_force_pin：定义在main_f4.c，若bootloader引脚为真，则继续循环 
		*  由于函数中条件编译的3个宏（BOARD_FORCE_BL_PIN_OUT、BOARD_FORCE_BL_PIN_IN和
		*  BOARD_FORCE_BL_PIN）均未被定义，此函数返回恒为假 */
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART/* 宏INTERFACE_USART定义在hw_config.h，下列代码有效 */
		/* board_test_usart_receiving_break：连续接收3个字节的内容，
		*  如果收到一个0字节则返回真，否则返回假。定义在main_f4.c 
		*  若串口USART2收到break信号（0字节），继续循环 */
		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break()) {
			continue;
		}

#endif

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT/* 宏BOARD_BOOT_FAIL_DETECT未定义，下列代码无效 */
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();/* jump_to_app：跳转至飞控固件 */

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
