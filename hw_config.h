/*
 * hw_config.h
 *
 *  Created on: May 17, 2015
 *      Author: david_s5
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/****************************************************************************
 * 10-8--2016:
 *  To simplify the ripple effect on the tools, we will be using
 *  /dev/serial/by-id/<asterisk>PX4<asterisk> to locate PX4 devices. Therefore
 *  moving forward all Bootloaders must contain the prefix "PX4 BL "
 *  in the USBDEVICESTRING
 *  This Change will be made in an upcoming BL release
 ****************************************************************************/
/*
 * Define usage to configure a bootloader
 *
 *
 * Constant                example          Usage
 * APP_LOAD_ADDRESS     0x08004000            - The address in Linker Script, where the app fw is org-ed
 * BOOTLOADER_DELAY     5000                  - Ms to wait while under USB pwr or bootloader request
 * BOARD_FMUV2
 * INTERFACE_USB        1                     - (Optional) Scan and use the USB interface for bootloading
 * INTERFACE_USART      1                     - (Optional) Scan and use the Serial interface for bootloading
 * USBDEVICESTRING      "PX4 BL FMU v2.x"     - USB id string
 * USBPRODUCTID         0x0011                - PID Should match defconfig
 * BOOT_DELAY_ADDRESS   0x000001a0            - (Optional) From the linker script from Linker Script to get a custom
 *                                               delay provided by an APP FW
 * BOARD_TYPE           9                     - Must match .prototype boad_id
 * _FLASH_KBYTES        (*(uint16_t *)0x1fff7a22) - Run time flash size detection
 * BOARD_FLASH_SECTORS  ((_FLASH_KBYTES == 0x400) ? 11 : 23) - Run time determine the physical last sector
 * BOARD_FLASH_SECTORS   11                   - Hard coded zero based last sector
 * BOARD_FLASH_SIZE     (_FLASH_KBYTES*1024)-   Total Flash size of device, determined at run time.
 *                         (1024 * 1024)      - Hard coded Total Flash of device - The bootloader and app reserved will be deducted
 *                                              programmatically
 *
 * BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2        - Optional sectors index in the flash_sectors table  (F4 only), to begin erasing.
 *                                               This is to allow sectors to be reserved for app fw usage. That will NOT be erased
 *                                               during a FW upgrade.
 *                                               The default is 0, and selects the first sector to be erased, as the 0th entry in the
 *                                               flash_sectors table. Which is the second physical sector of FLASH in the device.
 *                                               The first physical sector of FLASH is used by the bootloader, and is not defined
 *                                               in the table.
 *
 * APP_RESERVATION_SIZE (BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 16 * 1024) - Number of bytes reserved by the APP FW. This number plus
 *                                                                        BOOTLOADER_RESERVATION_SIZE  will be deducted from
 *                                                                        BOARD_FLASH_SIZE to determine the size of the App FW
 *                                                                        and hence the address space of FLASH to erase and program.
 * USBMFGSTRING            "PX4 AP"            - Optional USB MFG string (default is '3D Robotics' if not defined.)
 * SERIAL_BREAK_DETECT_DISABLED                -  Optional prevent break selection on Serial port from entering or staying in BL
 *
 * * Other defines are somewhat self explanatory.
 */

/* Boot device selection list*/
#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

#if  defined(TARGET_HW_PX4_FMU_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMU
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v1.x"
# define USBPRODUCTID                   0x0010
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     5
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO15
# define BOARD_PIN_LED_BOOTLOADER       GPIO14
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART1
# define BOARD_USART_CLOCK_REGISTER     RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB2ENR_USART1EN

# define BOARD_PORT_USART               GPIOB
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO6
# define BOARD_PIN_RX                   GPIO7
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPBEN
# define  SERIAL_BREAK_DETECT_DISABLED  1

/*
 * Uncommenting this allows to force the bootloader through
 * the PPM-in pin. Some receivers pull their PPM output low
 * when the transmitter is off, resulting in a stuck bootup,
 * so this feature is best disabled.
 *
 * # define BOARD_FORCE_BL_PIN             GPIO10
 * # define BOARD_FORCE_BL_PORT            GPIOA
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPAEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 * # define BOARD_FORCE_BL_STATE           0
 */

/****************************************************************************
 * TARGET_HW_PX4_FMU_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V2)

# define APP_LOAD_ADDRESS               0x08004000
/* APP_LOAD_ADDRESS，为Bootloader初始化完成后的飞控固件跳转地址
*（实际入口地址在0x08004004，这一点在bl.c文件的源代码中可以看到,
* 0x08004000被用于标识后面的代码是否有效） */
# define BOOTLOADER_DELAY               5000
/* BOOTLOADER_DELAY，为Bootloader初始化完毕到跳转到飞控固件需等待的时间 */
# define BOARD_FMUV2/* BOARD_FMUV2，在全部代码中没有用到 */
# define INTERFACE_USB                  1/* INTERFACE_USB，表示Bootloader可采用USB口与上位机通信 */
# define INTERFACE_USART                1/* INTERFACE_USART，表示Bootloader可采用串口与上位机通信 */
# define USBDEVICESTRING                "PX4 BL FMU v2.x"
/* USBDEVICESTRING，表示对应USB ID的字符串 */
# define USBPRODUCTID                   0x0011/* USBPRODUCTID，USB接口的PID */
# define BOOT_DELAY_ADDRESS             0x000001a0
/* BOOT_DELAY_ADDRESS，flash上对应存放Bootloader等待时间的内存地址 */

# define BOARD_TYPE                     9
/* BOARD_TYPE，对应不同硬件配置板子上处理器的编号，每个不同的处理器对应不同的值*/
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22) /* _FLASH_KBYTES，程序运行期间flash以KB计的大小的存储地址（为何位于Reserved地址？） */
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
/* BOARD_FLASH_SECTORS，STM32F4芯片若片内flash为1M，则Sector数量为11；若不为1M（2M），则Sector数量为23*/

# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024) /* BOARD_FLASH_SIZE，flash总大小 */

# define OSC_FREQ                       24/* OSC_FREQ，外部晶振频率24kHz */

/* 主控FMU指示灯在板上一共有2个，FMU的PWR和B/E；
*  PWR（LED704）为绿色常亮，上电即亮，B/E（LED701）由FMU的GPIOE12控制，为闪烁红色 */
# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
/* BOARD_PIN_LED_BOOTLOADER，主控Bootloader的LED指示灯为GPIOE12 */
# define BOARD_PORT_LEDS                GPIOE
/* BOARD_PORT_LEDS，主控FMU的LED指示灯控制引脚在GPIO Port E */
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
/* BOARD_CLOCK_LEDS，RCC_AHB1ENR_IOPEEN被libopencm3/include/libopencm3/stm32/f4/rcc.h
*  定义为(1<<4)，对应RCC_AHB1ENR寄存器的第4位，GPIOE的时钟使能位 */

# define BOARD_LED_ON                   gpio_clear
/* BOARD_LED_ON，gpio_clear函数定义在libopencm3/lib/stm32/common/gpio_common_all.c中，
*  实际为GPIO pin清零功能 */
# define BOARD_LED_OFF                  gpio_set
/* BOARD_LED_OFF，gpio_set函数定义在libopencm3/lib/stm32/common/gpio_common_all.c中，
*  实际为GPIO pin置1功能 */


# define BOARD_USART                    USART2
/* BOARD_USART，通信串口，USART2被libopencm3/include/libopencm3/stm32/common/usart_common_all.h
*  定义为SART2_BASE（0x40004400） */

# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
/* BOARD_USART_CLOCK_REGISTER，被定义为RCC_APB1ENR寄存器，用于使能USART2的时钟 */
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN
/* BOARD_USART_CLOCK_BIT，RCC_APB1ENR_USART2EN被libopencm3/include/libopencm3/stm32/f4/rcc.h
*  定义为(1<<17)，对应RCC_APB1ENR的17位，用于使能USART2的时钟 */


# define BOARD_PORT_USART               GPIOD
/* BOARD_PORT_USART，USART2的引脚位于GPIO Port D */
# define BOARD_PORT_USART_AF            GPIO_AF7
/* BOARD_PORT_USART_AF，GPIO_AF7被libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h
*  定义为0x7，对应AF7的alternative function功能为USART1～3 */

# define BOARD_PIN_TX                   GPIO5/* BOARD_PIN_TX，USART2的发送引脚为GPIOD5 */
# define BOARD_PIN_RX                   GPIO6/* BOARD_PIN_RX，USART2的接收引脚为GPIOD6 */
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
/* BOARD_USART_PIN_CLOCK_REGISTER，被定义为RCC_AHB1ENR寄存器，用于使能USART2引脚的时钟 */
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
/* BOARD_USART_PIN_CLOCK_BIT，RCC_AHB1ENR_IOPDEN被libopencm3/include/libopencm3/stm32/f4/rcc.h
*定义为(1<<3)，对应RCC_AHB1ENR寄存器的第3位，用于使能GPIOD的时钟 */

# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 */

/****************************************************************************
 * TARGET_HW_PX4_FMU_V3
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V3)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v3.x"
# define USBPRODUCTID                   0x0011
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     9
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 */

/****************************************************************************
 * TARGET_HW_PX4_FMU_V4
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V4)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v4.x"
# define USBPRODUCTID                   0x0012
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     11
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO11|GPIO1
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART1
# define BOARD_USART_CLOCK_REGISTER     RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB2ENR_USART1EN

# define BOARD_PORT_USART               GPIOB
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO6
# define BOARD_PIN_RX                   GPIO7
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPBEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/
/****************************************************************************
 * TARGET_HW_PX4_FMU_V4_PRO
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V4_PRO)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v4.x PRO"
# define USBPRODUCTID                   0x0013
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     13
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO11|GPIO1
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/

/****************************************************************************
 * TARGET_HW_PX4_FMU_V5
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V5)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v5.x"
# define USBPRODUCTID                   0x0032
# define BOOT_DELAY_ADDRESS             0x00000200

# define BOARD_TYPE                     50
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO7 // BLUE
# define BOARD_PIN_LED_BOOTLOADER       GPIO6 // GREEN
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/
/****************************************************************************
 * TARGET_HW_PX4_FMU_V5X
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FMU_V5X)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL FMU v5X.x"
# define USBPRODUCTID                   0x0033
# define USBVENDORID                    0x3185
# define USBMFGSTRING                   "Auterion"
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     51
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO5 // BLUE
# define BOARD_PIN_LED_BOOTLOADER       GPIO4 // GREEN
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    UART5
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_UART5EN

# define BOARD_PORT_USART_AF_TX         GPIO_AF7
# define BOARD_PORT_USART_TX            GPIOB
# define BOARD_PIN_TX                   GPIO9
# define BOARD_PORT_USART_AF_RX         GPIO_AF8
# define BOARD_PORT_USART_RX            GPIOD
# define BOARD_PIN_RX                   GPIO2
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT_TX   RCC_AHB1ENR_GPIOBEN
# define BOARD_USART_PIN_CLOCK_BIT_RX   RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1
# define OVERRIDE_USART_BAUDRATE        1500000

/****************************************************************************
 * TARGET_HW_MINDPX_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_MINDPX_V2)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "MindPX BL FMU v2.x"
# define USBPRODUCTID                   0x0030
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     88
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO8
# define BOARD_PORT_LEDS                GPIOA
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPAEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 */

/****************************************************************************
 * TARGET_HW_PX4_FLOW_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_FLOW_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FLOW
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 BL FLOW v1.3"
# define USBPRODUCTID                   0x0015

# define BOARD_TYPE                     6
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

/****************************************************************************
 * TARGET_HW_PX4_DISCOVERY_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_DISCOVERY_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_DISCOVERY
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 BL DISCOVERY"
# define USBPRODUCTID                   0x0001

# define BOARD_TYPE                     99
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO12
# define BOARD_PIN_LED_BOOTLOADER       GPIO13
# define BOARD_PORT_LEDS                GPIOD
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPDEN
# define BOARD_LED_ON                   gpio_set
# define BOARD_LED_OFF                  gpio_clear

/****************************************************************************
 * TARGET_HW_PX4_PIO_V1 or TARGET_HW_PX4_PIO_V2
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_PIO_V1) ||  defined(TARGET_HW_PX4_PIO_V2)

# define APP_LOAD_ADDRESS               0x08001000
/* APP_LOAD_ADDRESS，为bootloader初始化完成后的飞控固件跳转地址
*（实际入口地址在0x08001004，这一点在bl.c文件的源代码中可以看到,
*  0x08001000被用于标识后面的代码是否有效） */
# define APP_SIZE_MAX                   0xf000/* APP_SIZE_MAX，板载系统程序（Bootloader）大小上限 */
# define BOOTLOADER_DELAY               200
/* BOOTLOADER_DELAY，为Bootloader初始化完毕到跳转到飞控固件需等待的时间 */
# define BOARD_PIO
# define INTERFACE_USB                  0/*不能使用USB与上位机通信*/
# define INTERFACE_USART                1/*可以使用USART和上位机通信*/
# define USBDEVICESTRING                ""
# define USBPRODUCTID                   -1/*因为不用了，所以设为非法值*/

# define OSC_FREQ                       24/*外部晶振频率*/

/* IO协处理器指示灯在板上一共有3个，IO的PWR、B/E和ACT；PWR（LED702）为绿色常亮，上电即亮，
*  B/E（LED703）由IO协处理器的GPIOB15控制，为闪烁红色，
*  ACT（LED705）由IO协处理器的GPIOB14控制，为闪烁蓝色 */

# define BOARD_PIN_LED_ACTIVITY         GPIO14/*Bootloader 有命令要处理时亮起*/
# define BOARD_PIN_LED_BOOTLOADER       GPIO15/*IO BootLoader的指示灯为GPIO15*/
# define BOARD_PORT_LEDS                GPIOB/*IO的LED所在引脚都在GPIO Port B*/
# define BOARD_CLOCK_LEDS_REGISTER      RCC_APB2ENR
/*被定义为RCC_APB2ENR寄存器，用于使能LED时钟 */
# define BOARD_CLOCK_LEDS               RCC_APB2ENR_IOPBEN
/*被libopencm3/include/libopencm3/stm32/f1/rcc.h定义为(1<<3)，
* 对应RCC_APB2ENR寄存器的第3，GPIOB的时钟使能位 */
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA/*USART2的引脚位于GPIO Port A */
# define BOARD_PIN_TX                   GPIO_USART2_TX
# define BOARD_PIN_RX                   GPIO_USART2_RX
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_APB2ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_APB2ENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO5
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_APB2ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_APB2ENR_IOPBEN
/*对应RCC_APB2ENR寄存器的第3，GPIOB的时钟使能位 */
# define BOARD_FORCE_BL_PULL            GPIO_CNF_INPUT_FLOAT /* depend on external pull  ,
*GPIO_CNF_INPUT_FLOAT被libopencm3/include/libopencm3/stm32/f1/gpio.h定义为0x01，
*根据芯片手册表示GPIOB5为浮动输入状态，其值取决于外部硬件输入，即安全开关的状态*/
# define BOARD_FORCE_BL_VALUE           BOARD_FORCE_BL_PIN/*BOARD_FORCE_BL_PIN为GPIO5的值，用于判定IO协处理器是否永远进入Bootloader的循环状态 */

# define BOARD_FLASH_SECTORS            60/*STM32F1片内flash页数，Bootloader认为片内flash的前60页有效 */
# define BOARD_TYPE                     10/*对应不同硬件配置板子上处理器的编号，每个不同的处理器对应不同的值 */
# define FLASH_SECTOR_SIZE              0x400/*FLASH_SECTOR_SIZE，STM32F1片内flash每页大小为1KB */
# define NO_OTP_SN_CHIP                 1

/****************************************************************************
 * TARGET_HW_PX4_PIO_V3
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_PIO_V3)

# define APP_LOAD_ADDRESS               0x08001000
# define APP_SIZE_MAX                   0x3f000
# define BOOTLOADER_DELAY               200
# define BOARD_PIO
# define INTERFACE_USB                  0
# define INTERFACE_USART                1
# define USBDEVICESTRING                ""
# define USBPRODUCTID                   -1

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0
# define BOARD_PIN_LED_BOOTLOADER       GPIO13
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS_REGISTER      RCC_AHBENR
# define BOARD_CLOCK_LEDS               RCC_AHBENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO2
# define BOARD_PIN_RX                   GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHBENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHBENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO5
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHBENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHBENR_IOPBEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_NONE // depend on external pull
# define BOARD_FORCE_BL_VALUE           BOARD_FORCE_BL_PIN

# define BOARD_FLASH_SECTORS            (128-4) /* application #sec - bootloader - sec */
# define BOARD_TYPE                     13
# define FLASH_SECTOR_SIZE              0x800

/****************************************************************************
 * TARGET_HW_PX4_AEROCORE_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_PX4_AEROCORE_V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_AEROCORE
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "Gumstix BL AEROCORE"
# define USBPRODUCTID                   0x1001

# define BOARD_TYPE                     98
# define BOARD_FLASH_SECTORS            23
# define BOARD_FLASH_SIZE               (2048 * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO10  // Yellow
# define BOARD_PIN_LED_BOOTLOADER       GPIO9   // Blue
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_FORCE_BL_PIN_OUT         GPIO0   // J11 header, pin 1
# define BOARD_FORCE_BL_PIN_IN          GPIO1   // J11 header, pin 3
# define BOARD_FORCE_BL_PORT            GPIOB
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPBEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP

/****************************************************************************
 * TARGET_HW_TAP_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_TAP_V1)

# define APP_LOAD_ADDRESS               0x0800C000
# define BOOTLOADER_DELAY               5000
# define BOARD_TAP
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL TAP v1.x"
# define USBPRODUCTID                   0x0040
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     64
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2
# define APP_RESERVATION_SIZE            (2 * 16 * 1024) /* 2 16 Kib Sectors */
# define OSC_FREQ                       16

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO2
# define BOARD_PIN_RX                   GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPAEN

# define BOARD_PIN_LED_ACTIVITY         GPIO4
# define BOARD_PIN_LED_BOOTLOADER       GPIO5
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_POWER_PIN_OUT            GPIO4
# define BOARD_POWER_PORT               GPIOA
# define BOARD_POWER_CLOCK_REGISTER     RCC_AHB1ENR
# define BOARD_POWER_CLOCK_BIT          RCC_AHB1ENR_IOPAEN
# define BOARD_POWER_ON                 gpio_set
# define BOARD_POWER_OFF                gpio_clear
# undef  BOARD_POWER_PIN_RELEASE        /* Leave pin enabling Power - un comment to release (disable power)*/
# define USBMFGSTRING                   "The Autopilot"
# define USB_FORCE_DISCONNECT           1
#define  SERIAL_BREAK_DETECT_DISABLED   1

/****************************************************************************
 * TARGET_HW_CRAZYFLIE
 ****************************************************************************/

#elif  defined(TARGET_HW_CRAZYFLIE)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_CRAZYFLIE
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "Crazyflie BL"
# define USBPRODUCTID                   0x0016

# define BOARD_TYPE                     12
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO0
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USB_VBUS_SENSE_DISABLED

# define USBMFGSTRING                   "Bitcraze AB"

#elif  defined(TARGET_HW_CRAZYFLIE21)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_CRAZYFLIE21
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "Crazyflie21 BL"
# define USBPRODUCTID                   0x0016

# define BOARD_TYPE                     14
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO0
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USB_VBUS_SENSE_DISABLED

# define USBMFGSTRING                   "Bitcraze AB"

/****************************************************************************
 * TARGET_HW_OMNIBUS_NXT
 ****************************************************************************/

#elif  defined(TARGET_HW_OMNIBUSF4SD)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 OmnibusF4SD"
# define USBPRODUCTID                   0x0016

# define BOARD_TYPE                     42
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE    1
# define APP_RESERVATION_SIZE           (1 * 16 * 1024) /* 1 16 Kib Sectors */

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO5
# define BOARD_PIN_LED_BOOTLOADER       GPIO4
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USB_VBUS_SENSE_DISABLED

# define USBMFGSTRING                   "Vertile"

/****************************************************************************
 * TARGET_HW_KAKUTEF7
 ****************************************************************************/

#elif  defined(TARGET_HW_KAKUTEF7)

# define APP_LOAD_ADDRESS               0x08018000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 KakuteF7"
# define USBPRODUCTID                   0x0016
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     123
# define BOARD_FLASH_SECTORS            7
# define BOARD_FLASH_SIZE               (1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE    2
# define APP_RESERVATION_SIZE            (2 * 32 * 1024) /* 2 32 Kib Sectors */

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         0
# define BOARD_PIN_LED_BOOTLOADER       GPIO2 // BLUE
# define BOARD_PORT_LEDS                GPIOA
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOAEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define USBMFGSTRING                   "Holybro"

// VBUS sense is connected to PA8 instead of PA9
# define BOARD_USB_VBUS_SENSE_DISABLED
# define BOARD_PORT_VBUS                GPIOA
# define BOARD_PIN_VBUS                 GPIO8

# define BOARD_USART                    USART3 // labeled as UART3
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART3EN
# define BOARD_PORT_USART               GPIOB
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO10
# define BOARD_PIN_RX                   GPIO11
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_GPIOBEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/****************************************************************************
 * TARGET_HW_AUAV_X2V1
 ****************************************************************************/

#elif  defined(TARGET_HW_AUAV_X2V1)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL AUAV X2.1"
# define USBPRODUCTID                   0x0021
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     33
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 */

# define USBMFGSTRING                   "AUAV"

/****************************************************************************
 * TARGET_HW_PX4_AEROFC_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_AEROFC_V1)

# define APP_LOAD_ADDRESS               0x0800C000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  0
# define USBDEVICESTRING                ""
# define USBPRODUCTID                   0

# define INTERFACE_USART                1
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     65
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE    2
# define APP_RESERVATION_SIZE           (2 * 16 * 1024) /* 2 16 Kib Sectors */

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO12
# define BOARD_PIN_LED_BOOTLOADER       GPIO9 | GPIO10 | GPIO11 | GPIO13 | GPIO14 | GPIO15
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOA
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO2
# define BOARD_PIN_RX                   GPIO3
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPAEN

# define BOARD_FORCE_BL_PIN             GPIO11
# define BOARD_FORCE_BL_PORT            GPIOA
# define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
# define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPAEN
# define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLDOWN
# define BOARD_FORCE_BL_STATE           1

/****************************************************************************
 * TARGET_HW_FMUK66_V3
 ****************************************************************************/

#elif  defined(TARGET_HW_FMUK66_V3)
# define APP_LOAD_ADDRESS               0x00006000  // Reserve 24K for BL
# define BOOTLOADER_DELAY               5000
# define BOARD_NXPHLITEV3
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                // See kinetis/usb_device_descriptor.c
# define USBPRODUCTID                   0x001c
# define USBVENDORID                    0x1fc9
# define BOARD_TYPE                     28
# define _FLASH_KBYTES                  2048
# define FLASH_SECTOR_SIZE              4096
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)
# define BOARD_FLASH_SECTORS            ((BOARD_FLASH_SIZE / FLASH_SECTOR_SIZE)- (BOOTLOADER_RESERVATION_SIZE/FLASH_SECTOR_SIZE))

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         14                   // green LED
# define BOARD_PIN_LED_BOOTLOADER       13                   // Amber LED
# define BOARD_PORT_LEDS                D
# define BOARD_LED_ON                   GPIO_ClearPinsOutput
# define BOARD_LED_OFF                  GPIO_SetPinsOutput

# define BOARD_UART                     4
# define BOARD_PORT_UART                C
# define BOARD_PORT_UART_AF             kPORT_MuxAlt3
# define BOARD_PIN_TX                   15
# define BOARD_PIN_RX                   14



# define BOARD_PORT_VBUS                E
# define BOARD_PIN_VBUS                 8

# define BOARD_PORT_UART_RTS            E
# define BOARD_UART_RTS_PIN             27

# define SERIAL_BREAK_DETECT_DISABLED   1
# define USB_DATA_ALIGN __attribute__((aligned(2)))
# define ARCH_SN_MAX_LENGTH             16

/****************************************************************************
 * TARGET_HW_FMUK66_E
 ****************************************************************************/

#elif  defined(TARGET_HW_FMUK66_E)
# define APP_LOAD_ADDRESS               0x00006000  // Reserve 24K for BL
# define BOOTLOADER_DELAY               5000
# define BOARD_NXPHLITEV3
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                // See kinetis/usb_device_descriptor.c
# define USBPRODUCTID                   0x001c
# define USBVENDORID                    0x1fc9
# define BOARD_TYPE                     30
# define _FLASH_KBYTES                  2048
# define FLASH_SECTOR_SIZE              4096
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)
# define BOARD_FLASH_SECTORS            ((BOARD_FLASH_SIZE / FLASH_SECTOR_SIZE)- (BOOTLOADER_RESERVATION_SIZE/FLASH_SECTOR_SIZE))

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         14                   // green LED
# define BOARD_PIN_LED_BOOTLOADER       13                   // Amber LED
# define BOARD_PORT_LEDS                D
# define BOARD_LED_ON                   GPIO_ClearPinsOutput
# define BOARD_LED_OFF                  GPIO_SetPinsOutput

# define BOARD_UART                     4
# define BOARD_PORT_UART                C
# define BOARD_PORT_UART_AF             kPORT_MuxAlt3
# define BOARD_PIN_TX                   15
# define BOARD_PIN_RX                   14



# define BOARD_PORT_VBUS                E
# define BOARD_PIN_VBUS                 8

# define BOARD_PORT_UART_RTS            E
# define BOARD_UART_RTS_PIN             27

# define SERIAL_BREAK_DETECT_DISABLED   1
# define USB_DATA_ALIGN __attribute__((aligned(2)))
# define ARCH_SN_MAX_LENGTH             16

/****************************************************************************
 * TARGET_HW_CUBE_F4
 ****************************************************************************/

#elif  defined(TARGET_HW_CUBE_F4)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMUV2
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "ProfiCNC CUBE F4 BL"
# define USBPRODUCTID                   0x0001
# define USBMFGSTRING                   "Hex Technology Limited"
# define USBVENDORID                    0x2DAE
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     9
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         0               // no activity LED
# define BOARD_PIN_LED_BOOTLOADER       GPIO12
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPDEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
 */

/****************************************************************************
 * TARGET_HW_AV_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_AV_X_V1)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  0
# define INTERFACE_USART                1
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     29
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_USART                    UART8
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_UART8EN

# define BOARD_PORT_USART               GPIOE
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO1
# define BOARD_PIN_RX                   GPIO0
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/
/****************************************************************************
 * TARGET_HW_SMARTAP_AIRLINK
 ****************************************************************************/

#elif  defined(TARGET_HW_SMARTAP_AIRLINK)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL SmartAP AIRLink"
# define USBPRODUCTID                   0xa32f
# define USBVENDORID                    0x0483
# define USBMFGSTRING                   "Sky-Drones Technologies Ltd"
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     55
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO5 // BLUE
# define BOARD_PIN_LED_BOOTLOADER       GPIO4 // GREEN
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOEEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    UART5
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_UART5EN

# define BOARD_PORT_USART_AF_TX         GPIO_AF7
# define BOARD_PORT_USART_TX            GPIOB
# define BOARD_PIN_TX                   GPIO9
# define BOARD_PORT_USART_AF_RX         GPIO_AF8
# define BOARD_PORT_USART_RX            GPIOD
# define BOARD_PIN_RX                   GPIO2
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT_TX   RCC_AHB1ENR_GPIOBEN
# define BOARD_USART_PIN_CLOCK_BIT_RX   RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1
# define OVERRIDE_USART_BAUDRATE        1500000

/****************************************************************************
 * TARGET_HW_SMARTAP_PRO
 ****************************************************************************/

#elif  defined(TARGET_HW_SMARTAP_PRO)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                0
# define USBDEVICESTRING                "PX4 BL SmartAP PRO"
# define USBPRODUCTID                   0x0020
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     32
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO2
# define BOARD_PORT_LEDS                GPIOH
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOHEN
# define BOARD_LED_ON                   gpio_set
# define BOARD_LED_OFF                  gpio_clear

/****************************************************************************
 * TARGET_HW_MODALAI_FC_V1
 ****************************************************************************/

#elif  defined(TARGET_HW_MODALAI_FC_V1)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL ModalAI FCv1"
# define USBMFGSTRING                   "ModalAI"
# define USBPRODUCTID                   0xa32f
# define USBVENDORID                    0x0483
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     41775
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO0 // RED
# define BOARD_PIN_LED_BOOTLOADER       GPIO1 // GREEN
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART3
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART3EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO8
# define BOARD_PIN_RX                   GPIO9
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/****************************************************************************
 * TARGET_HW_UVIFY_CORE
 ****************************************************************************/

#elif  defined(TARGET_HW_UVIFY_CORE)

# define APP_LOAD_ADDRESS               0x08004000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL UVify Core"
# define USBMFGSTRING                   "UVify"
# define USBPRODUCTID                   0x0001
# define USBVENDORID                    0x1008
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     20
# define _FLASH_KBYTES                  (*(uint16_t *)0x1fff7a22)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 11 : 23)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       24

# define BOARD_PIN_LED_ACTIVITY         GPIO3
# define BOARD_PIN_LED_BOOTLOADER       GPIO11|GPIO1
# define BOARD_PORT_LEDS                GPIOB
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART1
# define BOARD_USART_CLOCK_REGISTER     RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB2ENR_USART1EN

# define BOARD_PORT_USART               GPIOB
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO6
# define BOARD_PIN_RX                   GPIO7
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPBEN
# define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 */

/****************************************************************************
 * TARGET_HW_HOLYBRO_PIX32_V5
 ****************************************************************************/

#elif  defined(TARGET_HW_HOLYBRO_PIX32_V5)

# define APP_LOAD_ADDRESS               0x08008000
# define BOOTLOADER_DELAY               5000
# define INTERFACE_USB                  1
# define INTERFACE_USART                1
# define USBDEVICESTRING                "PX4 BL PIX32V5"
# define USBPRODUCTID                   0x004E
# define USBVENDORID                    0x3162
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     78
# define _FLASH_KBYTES                  (*(uint16_t *)0x1ff0f442)
# define BOARD_FLASH_SECTORS            ((_FLASH_KBYTES == 0x400) ? 7 : 11)
# define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

# define OSC_FREQ                       16

# define BOARD_PIN_LED_ACTIVITY         GPIO7 // BLUE
# define BOARD_PIN_LED_BOOTLOADER       GPIO6 // GREEN
# define BOARD_PORT_LEDS                GPIOC
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_GPIOCEN
# define BOARD_LED_ON                   gpio_clear
# define BOARD_LED_OFF                  gpio_set

# define BOARD_USART                    USART2
# define BOARD_USART_CLOCK_REGISTER     RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT          RCC_APB1ENR_USART2EN

# define BOARD_PORT_USART               GPIOD
# define BOARD_PORT_USART_AF            GPIO_AF7
# define BOARD_PIN_TX                   GPIO5
# define BOARD_PIN_RX                   GPIO6
# define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
# define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_GPIODEN
# define SERIAL_BREAK_DETECT_DISABLED   1
# define USBMFGSTRING                   "Holybro"

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         GPIO14
 * # define BOARD_FORCE_BL_PIN_IN          GPIO11
 * # define BOARD_FORCE_BL_PORT            GPIOE
 * # define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
 * # define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPEEN
 * # define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP
*/

#else
# error Undefined Target Hardware
#endif

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(USBMFGSTRING)
# define USBMFGSTRING "3D Robotics"
#endif

#if !defined(USBVENDORID)
#  define USBVENDORID 0x26AC
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 0
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 0
#endif

#if defined(OVERRIDE_USART_BAUDRATE)
#  define USART_BAUDRATE OVERRIDE_USART_BAUDRATE
#else
#  define USART_BAUDRATE 115200
#endif

#if INTERFACE_USART
#  if !defined(BOARD_PORT_USART_TX)
#    define BOARD_PORT_USART_TX BOARD_PORT_USART
#    define BOARD_PORT_USART_RX BOARD_PORT_USART
#  endif
#  if !defined(BOARD_USART_PIN_CLOCK_BIT_TX)
#    define BOARD_USART_PIN_CLOCK_BIT_TX BOARD_USART_PIN_CLOCK_BIT
#    define BOARD_USART_PIN_CLOCK_BIT_RX BOARD_USART_PIN_CLOCK_BIT
#  endif
#  if !defined(BOARD_PORT_USART_AF_TX)
#    define BOARD_PORT_USART_AF_TX BOARD_PORT_USART_AF
#  endif
#  if !defined(BOARD_PORT_USART_AF_RX)
#    define BOARD_PORT_USART_AF_RX BOARD_PORT_USART_AF
#  endif
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#endif /* HW_CONFIG_H_ */
