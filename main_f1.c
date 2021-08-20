/*
 * STM32F1 board support for the bootloader.
 *
 */
#include "hw_config.h"

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/systick.h>

#include "bl.h"

#define UDID_START      0x1FFFF7E8 //Unique ID address of F1

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000 //用于确定STM32型号的 DEGMCU_IDCODE address of F10x


#ifdef INTERFACE_USART
# define BOARD_INTERFACE_CONFIG		(void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG		NULL
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE, //BOARD_TYPE板载处理器编号在hw_config.h中定义，v2 IO协处理器为10。 */..
	.board_rev	= 0,
	.fw_size	= APP_SIZE_MAX,//固件大小的最大值，在hw_config.h中被定义为0xf000，60KB

	.systick_mhz	= OSC_FREQ, //系统时钟输入等于OSC_FREQ，在hw_config.h中被定义为24MHz
};
static void board_init(void);

uint32_t
board_get_devices(void)
{
	return BOOT_DEVICES_SELECTION;//USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
}

static void
board_init(void)
{
	/* initialise LEDs */
	rcc_peripheral_enable_clock(&BOARD_CLOCK_LEDS_REGISTER, BOARD_CLOCK_LEDS);
	gpio_set_mode(BOARD_PORT_LEDS,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);//LED_BOOTLOADER | LED_ACTIVITY
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* if we have one, enable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);

	gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
	gpio_set_mode(BOARD_FORCE_BL_PORT,
		      GPIO_MODE_INPUT,
		      BOARD_FORCE_BL_PULL,
		      BOARD_FORCE_BL_PIN);
#endif

	/* enable the backup registers */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

#ifdef INTERFACE_USART
	/* configure usart pins */
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT);
	gpio_set_mode(BOARD_PORT_USART,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      BOARD_PIN_TX);

	/* configure USART clock */
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#ifdef INTERFACE_I2C
# error I2C GPIO config not handled yet
#endif
}

void
board_deinit(void)
{
	/* deinitialise LEDs */
	gpio_set_mode(BOARD_PORT_LEDS,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

	/* if we have one, disable the force-bootloader pin */
#ifdef BOARD_FORCE_BL_PIN
	gpio_set_mode(BOARD_FORCE_BL_PORT,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_FORCE_BL_PIN);
	gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
#endif

	/* disable the backup registers */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

#ifdef INTERFACE_USART
	/* configure usart pins */
	gpio_set_mode(BOARD_PORT_USART,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      BOARD_PIN_TX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif
#ifdef INTERFACE_I2C
# error I2C GPIO config not handled yet
#endif

	/* reset the APB2 peripheral clocks */
	RCC_APB2ENR = 0x00000000; // XXX Magic reset number from STM32F1x reference manual
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
#if defined(INTERFACE_USB)
	rcc_clock_setup_in_hsi_out_48mhz();
#else
	rcc_clock_setup_in_hsi_out_24mhz();
#endif
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

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return FLASH_SECTOR_SIZE;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		flash_erase_page(APP_LOAD_ADDRESS + (sector * FLASH_SECTOR_SIZE));
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
	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

// See F4 version for future enhancement for full decoding

int get_mcu_desc(int max, uint8_t *revstr)
{
	const char none[] = "STM32F1xxx,?";
	int i;

	for (i = 0; none[i] && i < max - 1; i++) {
		revstr[i] = none[i];
	}

	return i;
}

int check_silicon(void)
{
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
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);//gpio_clear(uint32_t gpioport, uint8_t gpios)
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

static bool
should_wait(void)
{
	bool result = false;

	PWR_CR |= PWR_CR_DBP;//Disable backup domain write protection 

	if (BKP_DR1 == BL_WAIT_MAGIC) {
		result = true;
		BKP_DR1 = 0;
	}

	PWR_CR &= ~PWR_CR_DBP;

	return result;
}

int
main(void)
{
	unsigned timeout = 0;
	/* bootloader初始化完毕后跳转到飞控固件入口地址时，所需等待的时间，以ms计算。 */

	/* do board-specific initialisation */
	
	/* 调用board_init函数，此函数在main_f1.c中被定义，功能如下 */
	/* 1. 初始化LED控制并点亮LED灯 */
	/* 2. 设置强制Bootloader引脚GPIOB5为浮动输入模式，方便采集安全开关的状态 */
	/* 3. 使能能源接口时钟和备份接口时钟，准备RTC备份寄存器 */
	/* 4. 设置USART2-TX引脚GPIOA2，对应原理图SERIAL_IO_TO_FMU，用于IO协处理器发送信息到主控FMU */
	board_init();

#if defined(INTERFACE_USART) || defined (INTERFACE_USB) /*宏INTERFACE_USART和INTERFACE_USB分别被定义为1和0，代码有效，hw_config.h */
	/* XXX sniff for a USART connection to decide whether to wait in the bootloader? */
	timeout = BOOTLOADER_DELAY;/* BOOTLOADER_DELAY：值200，表示200ms，hw_config.h */
#endif

#ifdef INTERFACE_I2C/* 宏INTERFACE_I2C未定义，代码无效 */
# error I2C bootloader detection logic not implemented
#endif

	/* if the app left a cookie saying we should wait, then wait */
	/* 若RTC备份寄存器BKP_DR1中存储了预定值，则timeout赋值为200ms */
	/* should_wait：判定RTC备份寄存器BKP_DR1中是否存储了预定值。若存储了返回真，未存储则返回假 */
	/* BOOTLOADER_DELAY：值200，表示200ms，hw_config.h */

	if (should_wait()) {
		timeout = BOOTLOADER_DELAY;
	}

#ifdef BOARD_FORCE_BL_PIN/* 宏BOARD_FORCE_BL_PIN定义为GPIO5，代码有效，hw_config.h */
	/* 若GPIO5为高电平，则timeout赋值为0xffffffff，永久停留在bootloader中 */
	/* BOARD_FORCE_BL_VALUE：被定义为GPIO5（1<<5），hw_config.h */
	/* BOARD_FORCE_BL_PORT：被定义为GPIOB（GPIOB的收寄存器），hw_config.h */
	/* BOARD_FORCE_BL_PIN：被定义为GPIO5（1<<5），hw_config.h */
	/* gpio_get：获取某GPIO组的值，定义在libopencm3/lib/stm32/common/gpio_common_all.c */

	/* if the force-BL pin state matches the state of the pin, wait in the bootloader forever */
	if (BOARD_FORCE_BL_VALUE == gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN)) {
		timeout = 0xffffffff;
	}

#endif

	/* look for the magic wait-in-bootloader value in backup register zero */


	/* if we aren't expected to wait in the bootloader, try to boot immediately */
	/* 若timeout为0，则调用jump_to_app立即启动飞控固件。 */
	/* 若jump_to_app函数返回，则timeout赋值为0 */

	if (timeout == 0) {
		/* try to boot immediately */
		jump_to_app();

		/* if we returned, there is no app; go to the bootloader and stay there */
		timeout = 0;
	}

	/* configure the clock for bootloader activity */
	/* 初始化系统时钟为PLL（使用HSI） */
	/* clock_init：使用HSI将MCU的系统时钟为PLL，频率24MHz，定义在main_f1.c */
	clock_init();

	/* start the interface */
	/* 函数初始化串口USART2 */
	/* BOARD_INTERFACE_CONFIG：定义为BOARD_USART（USART2），main_f1.c */
	/* USART：值1，枚举型，定义在bl.h中 */
	cinit(BOARD_INTERFACE_CONFIG, USART);

	while (1) {
		/* run the bootloader, possibly coming back after the timeout */
		/* bootloader：bootloader与上位机的命令处理函数，
		*烧写新的固件或者timeout（不为0）时间到返回，定义在bl.c中 */
		bootloader(timeout);

		/* look to see if we can boot the app */
		jump_to_app();

		/* boot failed; stay in the bootloader forever next time */
		timeout = 0;
	}
}
