/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bl.c
 *
 * Common bootloader logic.
 *
 * Aside from the header includes below, this file should have no board-specific logic.
 */
#include "hw_config.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "bl.h"
#include "cdcacm.h"

#ifdef SECURE_BTL_ENABLED
#include "crypto_hal/crypto.h"
#endif

#include "uart.h"


// bootloader flash update protocol.
//
// Command format:
//
//      <opcode>[<command_data>]<EOC>
//
// Reply format:
//
//      [<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow (protocol 3) is:
//
// GET_SYNC		verify that the board is present 验证是否存在 baord
// GET_DEVICE		determine which board (select firmware to upload) 根据 board 选择固件上传
// CHIP_ERASE		erase the program area and reset address counter 擦除程序区域，重置地址计数器
// loop://写入程序
//      PROG_MULTI      program bytes
// GET_CRC		verify CRC of entire flashable area 验证整个 flashable 区域的 CRC
// RESET		finalise flash programming,reset chip and starts application 完成flash 编程，重置chip，开启应用程序
//

#define BL_PROTOCOL_VERSION 		5		// 协议版本信息 The revision of the bootloader protocol //bl协议版本
//* Next revision needs to update


// <opcode> and <status> values

//  protocol bytes 协议字
#define PROTO_INSYNC				0x12    // 'in sync' byte sent before stat 同步字
#define PROTO_EOC					0x20    // end of command 命令终止字

// Reply bytes 回告字
#define PROTO_OK					0x10    // INSYNC/OK      - 'ok' response 指令运行成功
#define PROTO_FAILED				0x11    // INSYNC/FAILED  - 'fail' response 指令运行错误
#define PROTO_INVALID				0x13	// INSYNC/INVALID - 非法指令 'invalid' response for bad commands
#define PROTO_BAD_SILICON_REV 		0x14 	// 主控MCU版本错误 On the F4 series there is an issue with < Rev 3 silicon
#define PROTO_RESERVED_0X15     0x15  // Reserved

// see https://pixhawk.org/help/errata
// Command bytes 命令字
#define PROTO_GET_SYNC				0x21    // NOP for re-establishing sync 获取同步命令
#define PROTO_GET_DEVICE			0x22    // get device ID bytes 获取设备ID
#define PROTO_CHIP_ERASE			0x23    //擦除flash与准备烧写飞控固件 erase program area and reset program address
#define PROTO_PROG_MULTI			0x27    //烧写flash write bytes at program address and increment
#define PROTO_GET_CRC				0x29	//计算CRC校验 compute & return a CRC
#define PROTO_GET_OTP				0x2a	//读取OTP区域某地址的1字节 read a byte from OTP at the given address
#define PROTO_GET_SN				0x2b    //读取MCU的UDID read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP				0x2c    //读取芯片ID和版本 read chip version (MCU IDCODE)
#define PROTO_SET_DELAY				0x2d    //设置飞控固件启动等待时间 set minimum boot delay
#define PROTO_GET_CHIP_DES			0x2e    //获取芯片描述信息 read chip version In ASCII
#define PROTO_BOOT					0x30    //完成烧写并启动飞控固件 boot the application
#define PROTO_DEBUG					0x31    //输出调试信息 emit debug information - format not defined
#define PROTO_SET_BAUD				0x33    //设置串口波特率 set baud rate on uart

#define PROTO_RESERVED_0X36     0x36  // Reserved
#define PROTO_RESERVED_0X37     0x37  // Reserved
#define PROTO_RESERVED_0X38     0x38  // Reserved
#define PROTO_RESERVED_0X39     0x39  // Reserved

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE 命令PROTO_GET_DEVICE参数*/
#define PROTO_DEVICE_BL_REV	1	// bootloader revision 协议版本
#define PROTO_DEVICE_BOARD_ID	2	// board ID 板载处理器编号
#define PROTO_DEVICE_BOARD_REV	3	// board revision 修订版本
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area 最大固件空间
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10 飞控固件向量表7-10项的函数地址（debug_monitor、sv_call、pend_sv、systick）

// State
#define STATE_PROTO_GET_SYNC      0x1     // Have Seen NOP for re-establishing sync
#define STATE_PROTO_GET_DEVICE    0x2     // Have Seen get device ID bytes
#define STATE_PROTO_CHIP_ERASE    0x4     // Have Seen erase program area and reset program address
#define STATE_PROTO_PROG_MULTI    0x8     // Have Seen write bytes at program address and increment
#define STATE_PROTO_GET_CRC       0x10    // Have Seen compute & return a CRC
#define STATE_PROTO_GET_OTP       0x20    // Have Seen read a byte from OTP at the given address
#define STATE_PROTO_GET_SN        0x40    // Have Seen read a word from UDID area ( Serial)  at the given address
#define STATE_PROTO_GET_CHIP      0x80    // Have Seen read chip version (MCU IDCODE)
#define STATE_PROTO_GET_CHIP_DES  0x100   // Have Seen read chip version In ASCII
#define STATE_PROTO_BOOT          0x200   // Have Seen boot the application

#if defined(TARGET_HW_PX4_PIO_V1)
#define STATE_ALLOWS_ERASE        (STATE_PROTO_GET_SYNC)
#define STATE_ALLOWS_REBOOT       (STATE_PROTO_GET_SYNC)
#  define SET_BL_STATE(s)
#  define SET_BL_FIRST_STATE(s)   bl_state |= (s)
#else
#define STATE_ALLOWS_ERASE        (STATE_PROTO_GET_SYNC|STATE_PROTO_GET_DEVICE)
#define STATE_ALLOWS_REBOOT       (STATE_ALLOWS_ERASE|STATE_PROTO_PROG_MULTI|STATE_PROTO_GET_CRC)
#  define SET_BL_STATE(s)         bl_state |= (s)
#  define SET_BL_FIRST_STATE(s)   bl_state |= (s)
#endif

static uint8_t bl_type;/* bl_type：接口类型静态全局变量，被bootloader函数定义 */
static uint8_t last_input;/* last_input：静态全局变量，上次获取串口数据的方式 */

/* cinit 函数是串口和 USB 虚拟串口初始化的主要函数，初始化后 Bootloader 可与上位机的通信接口，
*  根据输入命令对 Bootloader 进行调试。*/

inline void cinit(void *config, uint8_t interface)
{
#if INTERFACE_USB/* 主控FMU：宏INTERFACE_USB值为1，代码有效，定义在hw_config.h */
				/* IO协处理器：宏INTERFACE_USB值为0，代码无效，定义在hw_config.h */

	if (interface == USB) {
		return usb_cinit(config);
	}

#endif
#if INTERFACE_USART

	if (interface == USART) {
		return uart_cinit(config);
	}

#endif
}
/* 使 Bootloader 主动放弃通信串口的控制权。cfini 函数根据输入 interface 的值决定具体调用的
*  初始化函数；uart_cfini 用于反向初始化串口，usb_cfini 用于反向初始化 USB 接口。*/
inline void cfini(void)
{
#if INTERFACE_USB
	usb_cfini();
#endif
#if INTERFACE_USART
	uart_cfini();
#endif
}


/* 获取串口数据函数，可为USART和USB虚拟串口 */
/* USART：值为1，枚举类，定义在bl.h */
/* USB：值为2，枚举类，定义在bl.h */
inline int cin(uint32_t devices)
{
#if INTERFACE_USB	/* 对主控FMU，宏INTERFACE_USART定义为1，代码有效，hw_config.h */
					/* 对IO协处理器，宏INTERFACE_USART定义为0，代码无效，hw_config.h */
		/* 通过串口获取调试数据，若返回的数据值有效（>=0），则返回获取的数据值，并更新全局变量last_input */
		/* NONE：值0，枚举类，定义在bl.h */
		/* USB：值2，枚举类，定义在bl.h */
		/* bl_type：输入接口类型全局静态变量，被bootloader函数赋值 */
		/* last_input：全局变量上次获取串口数据的方式 */
		/* usb_cin：从USB模拟串口读入1个字，定义在cdcacm.c */

	if ((bl_type == NONE || bl_type == USB) && (devices & USB0_DEV) != 0) {
		int usb_in = usb_cin();

		if (usb_in >= 0) {
			last_input = USB;
			return usb_in;
		}
	}

#endif

#if INTERFACE_USART	/* 宏INTERFACE_USART定义为1，代码有效，hw_config.h */
		/* 通过串口获取调试数据，若返回的数据值有效（>=0），则返回获取的数据值，并更新全局变量last_input */
		/* NONE：值0，枚举类，定义在bl.h */
		/* USART：值1，枚举类，定义在bl.h */
		/* bl_type：输入接口类型全局静态变量，被bootloader函数赋值 */
		/* last_input：全局变量上次获取串口数据的方式 */
		/* uart_cin：从串口读入1个字，定义在usart.c */

	if ((bl_type == NONE || bl_type == USART) && (devices & SERIAL0_DEV) != 0) {
		int	uart_in = uart_cin();

		if (uart_in >= 0) {
			last_input = USART;
			return uart_in;
		}
	}

#endif

	return -1;
}


/* 串口或模拟串口输出特定内容的函数 */
inline void cout(uint8_t *buf, unsigned len)
{
#if INTERFACE_USB/* 对主控FMU，宏INTERFACE_USART定义为1，代码有效，hw_config.h */
				 /* 对IO协处理器，宏INTERFACE_USART定义为0，代码无效，hw_config.h */
	/* USB：值2，枚举类，定义在bl.h */
	/* bl_type：启动接口类型全局静态变量，被bootloader函数赋值 */
	/* usb_cout：USB模拟串口输出特定内容的函数，定义在usart.c */

	if (bl_type == USB) {
		usb_cout(buf, len);
	}

#endif
#if INTERFACE_USART

	if (bl_type == USART) {
		uart_cout(buf, len);
	}

#endif
}

/* The PX4IO is so low on FLASH that this abstaction is not possible as
 * a called API. Therefore these macros are needed.
 */
#if defined(TARGET_HW_PX4_PIO_V1)
# include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#define arch_systic_init(d) \
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); \
	systick_set_reload(board_info.systick_mhz * 1000); \
	systick_interrupt_enable(); \
	systick_counter_enable();

#define arch_systic_deinit() \
	systick_interrupt_disable(); \
	systick_counter_disable();

#define arch_flash_lock flash_lock
#define arch_flash_unlock flash_unlock

#define arch_setvtor(address) SCB_VTOR = address;

#endif

static const uint32_t	bl_proto_rev = BL_PROTOCOL_VERSION;	// 5，表示Bootloader协议版本 value returned by PROTO_DEVICE_BL_REV


static unsigned head, tail;		/* 头、尾指针 */
static uint8_t rx_buf[256];		/* 缓冲区长度 */


static enum led_state {LED_BLINK, LED_ON, LED_OFF} _led_state;/* _led_state：LED状态枚举型变量 */

void sys_tick_handler(void);

void
buf_put(uint8_t b)
{
	unsigned next = (head + 1) % sizeof(rx_buf);

	if (next != tail) {
		rx_buf[head] = b;
		head = next;
	}
}

int
buf_get(void)
{
	int	ret = -1;

	if (tail != head) {
		ret = rx_buf[tail];
		tail = (tail + 1) % sizeof(rx_buf);
	}

	return ret;
}


/* do_jump函数，重置堆栈指针并跳转至飞控固件入口的函数，jump_to_app函数的终极目标 */

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	/* 汇编嵌入C语言代码 */
	asm volatile(/* 汇编代码声明 */
		"msr msp, %0	\n"/* msr：无条件赋值指令，msp：主堆栈寄存器，%0：0号输入（stacktop）。将stacktop赋值给msp */
		"bx	%1	\n"			/* bx：寄存器寻址跳转指令，%1：1号输入（entrypoint）。程序跳转至寄存器entrypoint */
		: : "r"(stacktop), "r"(entrypoint) :);/* 定义汇编代码中%0代表stacktop，%1代表entrypoint */

	// just to keep noreturn happy
	/* 程序应该不会运行到这里，如果不幸到了，进入死循环 */
	for (;;) ;
}

void
jump_to_app()
{
	/* APP_LOAD_ADDRESS，值0x08004000（主控FMU）0x08001000（IO协处理器），飞控固件起始地址（hw_config.h） */
	const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;
	const uint32_t *vec_base = (const uint32_t *)app_base;

	/*
	 * We refuse to program the first word of the app until the upload is marked
	 * complete by the host.  So if it's not 0xffffffff, we should try booting it.
	 */
	 /* 1. 根据飞控固件的烧写约定，检测固件是否有效 */
	/* 飞控固件烧写时，我们特意约定最后烧写固件的首地址（飞控固件使用的堆栈首地址）。若固件首地址为0xffffffff，表明固件烧写未完成（或固件无效）无法跳转，函数直接返回 */
	if (app_base[0] == 0xffffffff) {
		return;
	}

#ifdef SECURE_BTL_ENABLED
	const image_toc_entry_t *toc_entries;
	uint8_t len;
	uint8_t i = 0;

	/* When secure btl is used, the address comes from the TOC */
	app_base = (const uint32_t *)0;
	vec_base = (const uint32_t *)0;

	/* TOC not found or empty, stay in btl */
	if (!find_toc(&toc_entries, &len)) {
		return;
	}

	/* Verify the first entry, containing the TOC itself */
	if (!verify_app(0, toc_entries)) {
		/* Image verification failed, stay in btl */
		return;
	}

	/* TOC is verified, loop through all the apps and perform crypto ops */
	for (i = 0; i < len; i++) {
		/* Verify app, if needed. i == 0 is already verified */
		if (i != 0 &&
		    toc_entries[i].flags1 & TOC_FLAG1_CHECK_SIGNATURE &&
		    !verify_app(i, toc_entries)) {
			/* Signature check failed, don't process this app */
			continue;
		}

		/* Check if this app needs decryption */
		if (toc_entries[i].flags1 & TOC_FLAG1_DECRYPT &&
		    !decrypt_app(i, toc_entries)) {
			/* Decryption / authenticated decryption failed, skip this app */
			continue;
		}

		/* Check if this app needs to be copied to RAM */
		if (toc_entries[i].flags1 & TOC_FLAG1_COPY) {
			/* TOC is verified, so we assume that the addresses are good */
			memcpy(toc_entries[i].target, toc_entries[i].start,
			       (uintptr_t)toc_entries[i].end - (uintptr_t)toc_entries[i].start);
		}

		/* Check if this app is bootable, if so set the app_base */
		if (toc_entries[i].flags1 & TOC_FLAG1_BOOT) {
			app_base = get_base_addr(&toc_entries[i]);
		}

		/* Check if this app has vectors, if so set the vec_base */
		if (toc_entries[i].flags1 & TOC_FLAG1_VTORS) {
			vec_base = get_base_addr(&toc_entries[i]);
		}
	}

	if (app_base == 0) {
		/* No bootable app found in TOC, bail out */
		return;
	}

	if (vec_base == 0) {
		/* No separate vectors block, vectors come along with the app */
		vec_base = app_base;
	}

#endif

	/*
	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */
	 /*地址范围检测*/
	if (app_base[1] < APP_LOAD_ADDRESS) {
		return;
	}

	if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size)) {
		return;
	}

	
	/* 2. 现在飞控固件有效，反向初始化以便把外设控制权交给飞控固件 */
	/* flash_lock：主控FMU，锁定flash，操作寄存器FLASH_CR的域LOCK（bit31），定义libopencm3/lib/stm32/common/flash_common_f234.c */
	/* flash_lock：IO协处理器，锁定flash，操作寄存器FLASH_CR的域LOCK（bit7），定义libopencm3/lib/stm32/common/flash_common_f01.c */
	/* bootloader程序中可能解锁flash进行固件烧写功能，这里确保flash上锁 */

	/* just for paranoia's sake */
	arch_flash_lock();

	/* kill the systick interrupt */
	/* 关闭systick中断（libopencm3/lib/cm3/systick.c） */
	/* systick倒计时关闭（libopencm3/lib/cm3/systick.c） */
	arch_systic_deinit();

	/* deinitialise the interface */
	/* 关闭串口USART2和USB虚拟串口，配置不变 */
	/* cfini：串口反向初始化函数，即关闭串口和USB虚拟串口，定义在main_f?.c */
	cfini();

	/* reset the clock */
	/* 关闭时钟源，将所有内部时钟设置为重启后的初始状态 */
	/* clock_deinit：时钟反向初始化函数，定义在main_f?.c */
	clock_deinit();

	/* deinitialise the board */
	/* 反向初始化板载外设（包括USART2、LED控制） */
	/* board_deinit：板载外设反向初始化函数，定义在main_f?.c */
	board_deinit();

	/* switch exception handlers to the application */
	
	/* 3. 更改向量表地址至飞控固件的向量表 */
	/* SCB_VTOR：寄存器，向量表偏移地址，地址0xE000ED08 */
	/* APP_LOAD_ADDRESS：飞控固件起始地址。0x08004000（主控FMU），0x08001000（IO协处理器） */
	arch_setvtor((uint32_t)vec_base);

	/* extract the stack and entrypoint from the app vector table and go */
	/* 4. 重置堆栈并跳转至飞控固件 */
	do_jump(app_base[0], app_base[1]);//栈顶 和向量表进入点
}

volatile unsigned timer[NTIMERS];/* NTIMERS：值4，通用倒计时器数量，定义在bl.h；timer：通用计时器数组变量 */

void
sys_tick_handler(void)
{
	unsigned i;

	for (i = 0; i < NTIMERS; i++)
		if (timer[i] > 0) {
			timer[i]--;
		}
	/* 处理B/E LED（主控FMU为LED701，IO协处理器为LED703）闪烁功能 */
	/* 时钟设为50ms，每个周期就是100ms。 */
	/* _led_state：全局LED状态枚举（led_state枚举型）变量，存储B/E LED的状态，定义在bl.c */
	/* TIMER_LED：值2，定义在bl.h */
	/* LED_BLINK：值0，led_state枚举型，定义在bl.c */
	/* LED_BOOTLOADER：值2，定义在bl.h */
	/* led_toggle：使输入的LED反向，定义在main_f?.c */
	if ((_led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
		led_toggle(LED_BOOTLOADER);
		timer[TIMER_LED] = 50;
	}
}


/* 延时函数，被bootloader函数调用 */

void
delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;

	while (timer[TIMER_DELAY] > 0)
		;
}


/* 根据输入设置LED状态的函数，被bootloader函数调用，功能如下： */
/* 若输入为LED_OFF，则关闭B/E LED灯 */
/* 若输入为LED_ON，则关闭B/E LED灯 */
/* 若输入为LED_BLINK，则2号计时器（TIMER_LED）清零，systick中断函数立即，准备闪烁B/E LED灯 */

static void
led_set(enum led_state state)
{
	_led_state = state;

	switch (state) {
	case LED_OFF:
		led_off(LED_BOOTLOADER);
		break;

	case LED_ON:
		led_on(LED_BOOTLOADER);
		break;

	case LED_BLINK:
		/* restart the blink state machine ASAP */
		timer[TIMER_LED] = 0;
		break;
	}
}


/* 发送同步命令函数，是bootloader函数中命令运行成功后调用的函数 */
static void
sync_response(void)
{
	/* PROTO_INSYNC：值0x12，同步指令字，定义在bl.c */
	/* PROTO_OK：值0x10，指令运行成功回告字，定义在bl.c */
	/* cout：串口或模拟串口输出特定内容的函数，定义在bl.c */
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_OK	// "OK"
	};

	cout(data, sizeof(data));
}

/* 当MCU型号错误处理函数，串口或USB模拟串口输出非法响应（PROTO_INSYNC+PROTO_BAD_SILICON_REV），
*  是bootloader函数中bad_silicon标号对应的函数 */

#if defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)/* 对主控FMU，TARGET_HW_PX4_FMU_V4定义为1，代码有效，Makefile.f4 */
static void														/* 对IO协处理器，TARGET_HW_PX4_FMU_V4未定义，代码无效 */
bad_silicon_response(void)
{
	/* PROTO_INSYNC：值0x12，同步指令字，定义在bl.c */
	/* PROTO_BAD_SILICON_REV：值0x14，主控MCU型号错误命令字，定义在bl.c */
	/* cout：串口或模拟串口输出特定内容的函数，定义在bl.c */
	uint8_t data[] = {
		PROTO_INSYNC,			// "in sync"
		PROTO_BAD_SILICON_REV	// "issue with < Rev 3 silicon"
	};

	cout(data, sizeof(data));
}
#endif


/* 当指令非法时的处理函数，串口或USB模拟串口输出非法响应（PROTO_INSYNC+PROTO_INVALID），
*  是bootloader函数中cmd_bad标号对应的函数 */

static void
invalid_response(void)
{
	/* PROTO_INSYNC：值0x12，同步指令字，定义在bl.c */
	/* PROTO_INVALID：值0x13，非法指令字，定义在bl.c */
	/* cout：串口或模拟串口输出特定内容的函数，定义在bl.c */
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_INVALID	// "invalid command"

	};

	cout(data, sizeof(data));
}

/* 当指令运行失败时的处理函数，串口或USB模拟串口输出运行失败响应（PROTO_INSYNC+PROTO_FAILED），
*  是bootloader函数中cmd_fail标号对应的函数 */
static void
failure_response(void)
{
	/* PROTO_INSYNC：值0x12，同步指令字，定义在bl.c */
	/* PROTO_FAILED：值0x11，运行失败指令字，定义在bl.c */
	/* cout：串口或模拟串口输出特定内容的函数，定义在bl.c */
	uint8_t data[] = {
		PROTO_INSYNC,	// "in sync"
		PROTO_FAILED	// "command failed"
	};

	cout(data, sizeof(data));
}


/* 在给定的时间timeout内读到串口数据，则返回数据；若超时返回-1 */
static int
cin_wait(unsigned timeout)
{
	int c = -1;

	
	/* 设置1号定时器（TIMER_CIN）周期为输入值（此处为0） */
	/* TIMER_CIN：值1，定义在bl.h */
	/* timer：通用计时器数组变量，定义在bl.c */
	/* start the timeout */
	timer[TIMER_CIN] = timeout;//当 timeout = 0,以下循环会只执行一次

	
	/* 一直读取串口获得数据。若要返回，或者读取到数据，或者若定时器（TIMER_CIN）到时。 */
	/* cin_count：串口和USB虚拟串口接收到的数据总量，定义在bl.c */
	/* cin：获取串口数据函数，定义在bl.c */
	do {
		c = cin(board_get_devices());

		if (c >= 0) {
			break;
		}

	} while (timer[TIMER_CIN] > 0);

	return c;
}

/**
 * Function to wait for EOC
 *
 * @param timeout length of time in ms to wait for the EOC to be received
 * @return true if the EOC is returned within the timeout perio, else false
 */

/* 在给定的等待时间内，下一条获取到的命令是否为终止字PROTO_EOC；若是返回真，不是返回假 */
inline static bool
wait_for_eoc(unsigned timeout)
{
	return cin_wait(timeout) == PROTO_EOC;
}


/* 串口或USB模拟串口输出1个字 */
static void
cout_word(uint32_t val)
{
	/* cout：串口或模拟串口输出特定内容的函数，定义在bl.c */
	cout((uint8_t *)&val, 4);
}


/* 串口或USB模拟串口读入1个字，具有超时检测功能 */
#ifndef NO_OTP_SN_CHIP//only for IO1/2
static int
cin_word(uint32_t *wp, unsigned timeout)
{
	/* 由于USB串口每次只能读入1字节，而结果需要的是1个字，
	*故定义联合体u长度4字节适用于两种操作 */
	union {
		uint32_t w;
		uint8_t b[4];
	} u;

	for (unsigned i = 0; i < 4; i++) {
		int c = cin_wait(timeout);

		if (c < 0) {
			return c;
		}

		u.b[i] = c & 0xff;
	}

	*wp = u.w;
	return 0;
}
#endif


/* 给出数据src的crc32校验结果 */

static uint32_t
crc32(const uint8_t *src, unsigned len, unsigned state)
{
	static uint32_t crctab[256];

	/* check whether we have generated the CRC table yet */
	/* this is much smaller than a static table */
	/* 检查是否已生成crc32校验表（按照定义肯定是未生成的），若未生成则立刻生成 */
	if (crctab[1] == 0) {
		for (unsigned i = 0; i < 256; i++) {
			uint32_t c = i;

			for (unsigned j = 0; j < 8; j++) {
				if (c & 1) {
					c = 0xedb88320U ^ (c >> 1);

				} else {
					c = c >> 1;
				}
			}

			crctab[i] = c;
		}
	}
	/* 对于从src开始的len长度字节进行crc32校验 */
	for (unsigned i = 0; i < len; i++) {
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	}

	return state;
}

void
bootloader(unsigned timeout)
{
	/* bl_type：全局接口类型变量。NONE：值0，枚举类，定义在bl.h */
	/* 若使bootloader实际上能够起作用，必须设置bl_type的类型，不能按照默认值NONE */
	bl_type = NONE; // The type of the bootloader, whether loading from USB or USART, will be determined by on what port the bootloader recevies its first valid command.
	volatile uint32_t  bl_state = 0; // Must see correct command sequence to erase and reboot (commit first word)
	uint32_t	address = board_info.fw_size;	/* force erase before upload will work */
	uint32_t	first_word = 0xffffffff;

	/* (re)start the timer system */
	
	/* 1.（重新）启动systick定时器和中断，周期为1ms */
	/* STK_CSR_CLKSOURCE_AHB：值1<<2，选取AHB作为systick的时钟源，libopencm3/include/libopencm3/cm3/systick.h */
	/* board_info.systick_mhz：值168（主控FMU，main_f4.c），24（IO协处理器，main_f1.c） */
	/* systick_set_clocksource：选取systick的时钟源，libopencm3/lib/cm3/systick.c */
	/* systick_set_reload：设置systick时钟倒计时（寄存器STK_VAL），libopencm3/lib/cm3/systick.c */
	/* systick_interrupt_enable：开启systick中断，libopencm3/lib/cm3/systick.c */
	/* systick_counter_enable：systick倒计时开启，libopencm3/lib/cm3/systick.c */
	arch_systic_init();

	/* if we are working with a timeout, start it running */
	/* 2. 若timeout不为0，启动0号时钟（TIMER_BL_WAIT），周期为timeout，单位ms */
	/* timeout为0则不启动时钟，永远停留在本函数中 */
	/* TIMER_BL_WAIT：值0，定义在bl.h */
	/* timer：unsigned型全局数组，成员有4个，定义在bl.c */
	if (timeout) {
		timer[TIMER_BL_WAIT] = timeout;//timeout 仅仅在这儿初始化了 TIMER_BL_WAIT ；第一次以后的while 里都为0
	}

	/* make the LED blink while we are idle */
	/* 3. 设置闪烁B/E LED灯（主控FMU为LED701，IO协处理器为LED703） */
	/* B/E LED灯闪烁表示程序处于闲置状态，通过systick时钟的中断处理函数来实现 */
	/* LED_BLINK：值0，led_state枚举成员，定义在bl.c */
	/* led_set：LED灯状态设置函数，定义在bl.c */
	led_set(LED_BLINK);

	while (true) {
		volatile int c;
		int arg;
		static union {
			uint8_t		c[256];
			uint32_t	w[64];
		} flash_buffer;

		// Wait for a command byte
		
		/* 4.1 关闭ACT LED灯（主控FMU没有，IO协处理器为LED705） */
		/* LED_ACTIVITY：值1，led_state枚举成员，定义在bl.c */
		led_off(LED_ACTIVITY);
		/* 4.2 死循环读取串口数据（1字节），只有读到数据方可跳出循环 */
		do {
			/* if we have a timeout and the timer has expired, return now */
			
			/* 若timeout非零，0号时钟（TIMER_BL_WAIT）倒计时至0，则timeout时间到返回。 */
			/* TIMER_BL_WAIT：值0，定义在bl.h */
			/* timeout：bootloader跳转到飞控固件所需等待的时间，bootloader函数输入 */
			/* timer：通用计时器数组变量，定义在bl.c */
			if (timeout && !timer[TIMER_BL_WAIT]) {//第一次等待到时间耗尽都没有读到一个 byte 就跳出 主while
				return;
			}

			/* try to get a byte from the host */
			/* 不等待，立即读取串口一个数据（cin_wait输入为0） */
			/* cin_wait：在给定的输入时间timeout内读到串口数据，则返回数据；若超时返回-1，定义在bl.c */
			c = cin_wait(0);

		} while (c < 0);
		/* 4.3 读到串口数据（命令），开启ACT LED灯 */
		/* LED_ACTIVITY：值1，led_state枚举成员，定义在bl.c */
		/* led_off：开启LED灯，定义在main_f?.c */
		led_on(LED_ACTIVITY);

		// handle the command byte
		switch (c) {

		// sync
		//
		// command:		GET_SYNC/EOC
		// reply:		INSYNC/OK
		//
		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* (1) 同步命令PROTO_GET_SYNC（0x21），命令结构PROTO_GET_SYNC+PROTO_EOC */
		/*	 * 收到PROTO_GET_SYNC的2ms内未接收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*	 * 运行成功，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* 接收到同步命令PROTO_GET_SYNC的2ms内位需收到终止字PROTO_EOC（0x20），否则进行错误指令处理 */
		/* PROTO_GET_SYNC：值0x21，同步命令字，定义在bl.c */
		/* PROTO_EOC：值0x20，命令终止字，定义在bl.c */
		/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
		case PROTO_GET_SYNC:

			/* expect EOC */
			if (!wait_for_eoc(2)) {
				goto cmd_bad; //bl_state =0
			}

			SET_BL_FIRST_STATE(STATE_PROTO_GET_SYNC);//bl_state |= 0x01
			break;

		// get device info
		//
		// command:		GET_DEVICE/<arg:1>/EOC
		// BL_REV reply:	<revision:4>/INSYNC/EOC
		// BOARD_ID reply:	<board type:4>/INSYNC/EOC
		// BOARD_REV reply:	<board rev:4>/INSYNC/EOC
		// FW_SIZE reply:	<firmware size:4>/INSYNC/EOC
		// VEC_AREA reply	<vectors 7-10:16>/INSYNC/EOC
		// bad arg reply:	INSYNC/INVALID
		//
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （2）获取设备ID命令PROTO_GET_DEVICE（0x22），它后面通常还会跟随一个参数（用于指示具体获取设备的哪个内容），命令结构：PROTO_GET_DEVICE+arg(1byte)+PROTO_EOC */
		/*   * 若为无效参数或参数后2ms内未接收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 若为PROTO_DEVICE_BL_REV（值1），则返回bootloader协议版本号BL_PROTOCOL_VERSION（值5），回告结构：rev(4byte)+(PROTO_INSYNC+PROTO_OK) */
		/*   * 若为PROTO_DEVICE_BOARD_ID（值2），则返回板载处理器编号board_info.board_type（BOARD_TYPE，主控FMU值9，IO协处理器值10），回告结构：board_type(4byte)+(PROTO_INSYNC+PROTO_OK) */
		/*   * 若为PROTO_DEVICE_BOARD_REV（值3），则返回修订版本board_info.board_rev（值0），回告结构：board_rev(4byte)+(PROTO_INSYNC+PROTO_OK) */
		/*   * 若为PROTO_DEVICE_FW_SIZE（值4），则返回飞控固件的最大值（单位KB）board_info.fw_size（主控值2000，IO协处理器APP_SIZE_MAX=0xf000），回告结构：fw_size(4byte)+(PROTO_INSYNC+PROTO_OK) */
		/*   * 若为PROTO_DEVICE_VEC_AREA（值5），则返回飞控固件向量表7-10项的函数地址（debug_monitor、sv_call、pend_sv、systick），回告结构：vectors(4*4byte)+(PROTO_INSYNC+PROTO_OK) */
		/*   * 若为其他参数，则进行非法指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 运行成功，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_GET_DEVICE:
			/* expect arg then EOC */
			arg = cin_wait(1000);

			if (arg < 0) {
				goto cmd_bad;
			}

			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}
			/* 判定命令PROTO_GET_DEVICE的参数 */
			/* PROTO_DEVICE_BL_REV：值1，命令PROTO_GET_DEVICE的参数，表示获取Bootloader协议版本，定义在bl.c */
			/* PROTO_DEVICE_BOARD_ID：值2，命令PROTO_GET_DEVICE的参数，表示获取板载处理器编号（对应board_info.board_type=BOARD_TYPE），定义在bl.c */
			/* PROTO_DEVICE_BOARD_REV：值3，命令PROTO_GET_DEVICE的参数，表示获取修订版本（对应board_info.board_rev=0），定义在bl.c */
			/* PROTO_DEVICE_FW_SIZE：值4，命令PROTO_GET_DEVICE的参数，表示飞控固件的最大值（对应board_info.fw_size），定义在bl.c */
			/* PROTO_DEVICE_VEC_AREA：值5，命令PROTO_GET_DEVICE的参数，表示获取飞控固件向量表7-10项的函数地址（debug_monitor、sv_call、pend_sv、systick），定义在bl.c */
			/* bl_proto_rev：全局变量，存储Bootloader的版本号（BL_PROTOCOL_VERSION值5），是获取设备ID命令参数PROTO_DEVICE_BL_REV的回告值，定义在bl.c */
			/* cout：串口（包括USB虚拟串口）输出特定内容的函数，定义在bl.c */
			/* flash_func_read_word：读取flash特定地址1个字的函数，定义在main_f?.c */
			/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
			switch (arg) {
			case PROTO_DEVICE_BL_REV:
				cout((uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
				break;

			case PROTO_DEVICE_BOARD_ID:
				cout((uint8_t *)&board_info.board_type, sizeof(board_info.board_type));
				break;

			case PROTO_DEVICE_BOARD_REV:
				cout((uint8_t *)&board_info.board_rev, sizeof(board_info.board_rev));
				break;

			case PROTO_DEVICE_FW_SIZE:
				cout((uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
				break;

			case PROTO_DEVICE_VEC_AREA:
				for (unsigned p = 7; p <= 10; p++) {
					uint32_t bytes = flash_func_read_word(p * 4);

					cout((uint8_t *)&bytes, sizeof(bytes));
				}

				break;

			default:
				goto cmd_bad;
			}

			SET_BL_STATE(STATE_PROTO_GET_DEVICE); //bl_state |= 0x02
			break;

		// erase and prepare for programming
		//
		// command:		ERASE/EOC
		// success reply:	INSYNC/OK
		// erase failure:	INSYNC/FAILURE
		//
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ 
		* （3）擦除flash与准备烧写飞控固件指令PROTO_CHIP_ERASE（0x23），命令结构：PROTO_CHIP_ERASE+PROTO_EOC 
		*   * 收到PROTO_CHIP_ERASE的2ms内未收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) 
		*   * 若为主控FMU芯片（F4），调用check_silicon函数判断MCU版本是否正确（是否为revision 3） 
		*       * 若主控FMU版本不正确（revision 3以下），跳转进行bad_silicon处理，回告结构：(PROTO_INSYNC+PROTO_BAD_SILICON_REV) 
		*   * 点亮B/E灯，指示正在擦除flash，逐段、页（sector）擦除flash 
		*   * 关闭B/E灯，指示擦除flash已完成，取每KB的首字检查flash擦除操作是否成功 
		*       * 若flash擦除操作失败，进行指令运行失败处理，回告结构：(PROTO_INSYNC+PROTO_FAILED) 
		*   * 重置flash编写逻辑地址变量address为0，恢复B/E灯闪烁 
		*   * 运行成功，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) 
		* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_CHIP_ERASE:

			/* expect EOC */
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

#if defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)/* 对主控FMU，宏TARGET_HW_PX4_FMU_V4定义为1，代码有效，Makefile.F4 */
										/* 对UI协处理器，宏TARGET_HW_PX4_FMU_V4未定义，代码无效 */
			/* check_silicon：函数自动辨识MCU的版本，默认STM32F427的revision 3返回0，其余返回-1 */
			/* bad_silicon：MCU版本错误处理函数的标号，在bootloader函数末尾 */

			if (check_silicon()) { //f1 alwayes return 0
				goto bad_silicon;
			}

#endif

			if ((bl_state & STATE_ALLOWS_ERASE) != STATE_ALLOWS_ERASE) { 
				goto cmd_bad;
			}

			// clear the bootloader LED while erasing - it stops blinking at random
			// and that's confusing
			/* 点亮B/E灯 */
			/* LED_ON： 值1，led_state枚举成员，定义在bl.c*/
			/* led_set：LED灯状态设置函数，定义在bl.c */
			led_set(LED_ON);//LED_BOOTLOADER ON

			// erase all sectors
			/* flash_unlock：解锁flash的库函数，定义在libopencm3/lib/stm32/common/flash_common_f234.c（主控FMU），
			*  libopencm3/lib/stm32/common/flash_common_f01.c（IO协处理器） */
			arch_flash_unlock();//清楚 unlock 标志；授权 FPEC 访问
			/* 逐段擦除flash */
			/* flash_func_sector_size：返回MCU的flash在某sector的大小值，若无此sector则返回0，定义在main_f?.c */
			/* flash_func_erase_sector： */
			for (int i = 0; flash_func_sector_size(i) != 0; i++) {
				flash_func_erase_sector(i);
			}
			
			
			/* 关闭B/E灯 */
			/* LED_OFF： 值2，led_state枚举成员，定义在bl.c*/
			/* led_set：LED灯状态设置函数，定义在bl.c */
			// disable the LED while verifying the erase
			led_set(LED_OFF);

			// verify the erase 
			/* 取每KB的首字检查flash擦除操作是否正确，即是否有擦除成功 */
			/* board_info.fw_size：飞控固件的最大值（单位KB） */
			/* flash_func_read_word：读取某flash地址的数据，定义在main_f?.c */
			/* cmd_fail：指令运行错误处理的标号，在bootloader函数末尾 */
			for (address = 0; address < board_info.fw_size; address += 4)
				if (flash_func_read_word(address) != 0xffffffff) {
					goto cmd_fail;
				}

			address = 0;
			SET_BL_STATE(STATE_PROTO_CHIP_ERASE); //bl_state |= 0x4

			// resume blinking
			/* B/E灯闪烁 */
			/* LED_BLINK： 值0，led_state枚举成员，定义在bl.c*/
			/* led_set：LED灯状态设置函数，定义在bl.c */
			led_set(LED_BLINK);
			break;

		// program bytes at current address
		//
		// command:		PROG_MULTI/<len:1>/<data:len>/EOC
		// success reply:	INSYNC/OK
		// invalid reply:	INSYNC/INVALID
		// readback failure:	INSYNC/FAILURE
		//
		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
				/* （4）flash烧写命令PROTO_PROG_MULTI（0x27），命令结构：PROTO_PROG_MULTI+arg(1byte)+c(len bytes)+PROTO_EOC */
				/*	 * 收到PROTO_PROG_MULTI的50ms内未收到第一个参数（长度arg），按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 若收到的第一个参数（长度arg）不是4字节对齐，按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 若当前编写地址（address）+第一个参数（长度arg）超出飞控固件最大范围board_info.fw_size，按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 若收到的第一个参数（长度arg）超出缓冲区长度，按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 根据长度arg逐字节接收数据，并存放在缓冲区flash_buffer中；若其中任何1个字节超时1s，按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 接收完有效数据后，若200ms内未接收到命令终止字（PROTO_EOC），按照错误命令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
				/*	 * 若当前编写地址address为0，则将烧写缓冲区flash_buffer首个32位保存在变量first_word中，并将缓冲区首改为0xFFFFFFFF */
				/*		 * 对于主控FMU，若MCU版本检查失败，按照MCU版本错误处理，回告结构：(PROTO_INSYNC+PROTO_BAD_SILICON_REV) */
				/*	 * 逐字烧写flash地址，若写入与读出不一致，进行命令失败处理，回告结构：（PROTO_INSYNC+PROTO_FAILED） */
				/*	 * 运行成功，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
				/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_PROG_MULTI:		// program bytes
			// expect count
			/* 50ms内获取第一个参数 */
			/* cin_wait：在给定的时间内读到串口数据，则返回数据；若超时返回-1，定义在bl.c */
			/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
			/* board_info.fw_size：飞控固件的最大值 */
			/* flash_buffer：共同体，代表接收缓冲区（256字节或32字） */
			arg = cin_wait(50);

			if (arg < 0) {
				goto cmd_bad;
			}

			// sanity-check arguments 合理性检查，4 字节对齐
			if (arg % 4) {
				goto cmd_bad;
			}

			if ((address + arg) > board_info.fw_size) { //写入程序大于程序flash内存
				goto cmd_bad;
			}

			if ((unsigned int)arg > sizeof(flash_buffer.c)) {//每次传入字节数小于 buffer容量
				goto cmd_bad;
			}
			
			/* 逐字节接收串口数据并存放在缓冲区内，数据延时不超过1s */
			/* cin_wait：在给定的时间内读到串口数据，则返回数据；若超时返回-1，定义在bl.c */
			/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
			/* flash_buffer：共同体，代表接收缓冲区（256字节或32字） */
			for (int i = 0; i < arg; i++) {
				c = cin_wait(1000);

				if (c < 0) {
					goto cmd_bad;
				}

				flash_buffer.c[i] = c;
			}
			/* 接收完有效数据后，200ms内必须接收到命令终止字 */
			/* wait_for_eoc：在给定的时间内，下一条获取到的命令是否为命令终止字PROTO_EOC；若是返回真，不是返回假，定义在bl.c */
			/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
			if (!wait_for_eoc(200)) {
				goto cmd_bad;
			}

			/* 若当前编写逻辑地址为0，（对主控FMU，若MCU版本检查失败，跳转到bad_silicon，进行错误MCU版本处理），则将烧写缓冲区flash_buffer首个32位保存在变量first_word中，并将缓冲区首改为0xFFFFFFFF */
			/* check_silicon：函数自动辨识MCU的版本，默认STM32F427的revision 3返回0，其余返回-1 */
			/* bad_silicon：MCU版本错误处理函数的标号，在bootloader函数末尾 */
			/* first_word：首字保存变量，定义在bootloader函数中 */
			/* flash_buffer：共同体，代表接收缓冲区（256字节或32字） */
			if (address == 0) {

#if defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)

				if (check_silicon()) {
					goto bad_silicon;
				}

#endif

				// save the first word and don't program it until everything else is done
				first_word = flash_buffer.w[0];
				// replace first word with bits we can overwrite later
				flash_buffer.w[0] = 0xffffffff;
			}

			
			/* 逐字烧写flash地址，若写入与读出不一致，跳转到cmd_fail，进行命令失败处理，返回PROTO_INSYNC+PROTO_FAILED */
			/* flash_func_write_word：烧写flash某地址起始为特定内容，4位烧写，定义在main_f?.c */
			/* flash_func_read_word：读取flash某特定地址内容，4位读取，定义在main_f?.c */
			/* cmd_fail：指令运行错误处理的标号，在bootloader函数末尾 */
			arg /= 4;

			for (int i = 0; i < arg; i++) {

				// program the word
				flash_func_write_word(address, flash_buffer.w[i]);

				// do immediate read-back verify
				if (flash_func_read_word(address) != flash_buffer.w[i]) {
					goto cmd_fail;
				}

				address += 4;
			}

			SET_BL_STATE(STATE_PROTO_PROG_MULTI);//bl_state |= 0x08

			break;

		// fetch CRC of the entire flash area
		//
		// command:			GET_CRC/EOC
		// reply:			<crc:4>/INSYNC/OK
		//
		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ 
		* （5）CRC32校验命令PROTO_GET_CRC（0x29），命令结构：PROTO_GET_CRC+PROTO_EOC 
		*	  收到PROTO_GET_CRC的2ms内未接收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) 
		*	  计算CRC32结果，输出crcsum(4byte)，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) 
		* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_GET_CRC:

			// expect EOC
			if (!wait_for_eoc(2)) {
				goto cmd_bad;
			}

			// compute CRC of the programmed area
			/* 计算烧写区域的CRC32值，注意飞控固件首地址的处理 */
			/* board_info.fw_size：飞控固件的最大值 */
			/* flash_func_read_word：读取flash某特定地址内容，4位读取，定义在main_f?.c */
			/* crc32：计算给定数据的crc32校验结果 */
			uint32_t sum = 0;

			for (unsigned p = 0; p < board_info.fw_size; p += 4) {
				uint32_t bytes;

				if ((p == 0) && (first_word != 0xffffffff)) {
					bytes = first_word;

				} else {
					bytes = flash_func_read_word(p);
				}

				sum = crc32((uint8_t *)&bytes, sizeof(bytes), sum);
			}

			cout_word(sum);
			SET_BL_STATE(STATE_PROTO_GET_CRC); // bl_state |= 0x10
			break;

#ifndef NO_OTP_SN_CHIP

		// read a word from the OTP
		//
		// command:			GET_OTP/<addr:4>/EOC
		// reply:			<value:4>/INSYNC/OK
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （6）读取OTP区域命令PROTO_GET_OTP（0x2a），命令结构：PROTO_GET_OTP+index(4byte)+PROTO_EOC */
		/*   * 收到PROTO_GET_OTP后未在允许延时（100ms）内收到有效的地址信息，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 收到地址后2ms内未收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 运行成功，输出OTP地址数值val(4byte)，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_GET_OTP: //从flash读取OTP区域某地址的1字节指令 返回来验证
			// expect argument
			{
				uint32_t index = 0;
				

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_otp(index));
			}
			break;

		// read the SN from the UDID
		//
		// command:			GET_SN/<addr:4>/EOC
		// reply:			<value:4>/INSYNC/OK
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （7）获取MCU的UDID（Unique Device ID，或称为序列号）PROTO_GET_SN（0x2b），命令结构PROTO_GET_SN+index(4byte)+PROTO_EOC */
		/*   * 收到PROTO_GET_SN后未在允许延时（100ms）内收到有效的地址信息，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 收到地址后2ms内未收到命令终止字，则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 运行成功，输出UDID数值val(4byte)，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_GET_SN:
			// expect argument
			{
				uint32_t index = 0;

				if (cin_word(&index, 100)) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				// expect valid indices 0, 4 ...ARCH_SN_MAX_LENGTH-4

				if (index % sizeof(uint32_t) != 0 || index > ARCH_SN_MAX_LENGTH - sizeof(uint32_t)) {
					goto cmd_bad;
				}

				cout_word(flash_func_read_sn(index));
			}

			SET_BL_STATE(STATE_PROTO_GET_SN); //bl_state |= 0x 40
			break;

		// read the chip ID code
		//
		// command:			GET_CHIP/EOC
		// reply:			<value:4>/INSYNC/OK
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （8）获取芯片ID和版本信息PROTO_GET_CHIP（0x2c），命令结构PROTO_GET_CHIP+PROTO_EOC */
		/*   * 收到命令PROTO_GET_CHIP后2ms内未收到命令终止字，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*   * 运行成功，输出芯片ID和版本信息数值val(4byte)，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_GET_CHIP: {
				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				
			/* get_mcu_id：读取寄存器DBGMCU_IDCODE值，表示芯片ID和版本 */
				cout_word(get_mcu_id());//IDCODE
				SET_BL_STATE(STATE_PROTO_GET_CHIP); //bl_state |= 0x 80
			}
			break;

		// read the chip  description
		//
		// command:			GET_CHIP_DES/EOC
		// reply:			<value:4>/INSYNC/OK
		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （9）获取芯片描述信息PROTO_GET_CHIP_DES（0x2e），命令结构PROTO_GET_CHIP_DES+PROTO_EOC */
		/*	 * 收到命令PROTO_GET_CHIP后2ms内未收到命令终止字，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*	 * 运行成功，输出芯片描述信息len（4byte）+buffer(len byte)，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		case PROTO_GET_CHIP_DES: {
				uint8_t buffer[MAX_DES_LENGTH];
				unsigned len = MAX_DES_LENGTH;

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}

				len = get_mcu_desc(len, buffer);//MCU 描述  STM32F1xxx,?
				cout_word(len);
				cout(buffer, len);
				SET_BL_STATE(STATE_PROTO_GET_CHIP_DES);//bl_state |= 0x 100
			}
			break;
#endif

#ifdef BOOT_DELAY_ADDRESS/* 对主控FMU，BOOT_DELAY_ADDRESS定义为0x1a0，代码有效，hw_config.h */
									/* 对IO协处理器，BOOT_DELAY_ADDRESS未定义，代码无效 */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （10）设置飞控固件启动延时PROTO_SET_DELAY（0x2d），命令结构：PROTO_SET_DELAY+v(4byte)+PROTO_EOC */
		/*    * 收到PROTO_SET_DELAY后100ms内未收到延时v，或延时时间为负数，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*    * 取延时v的最低字节为时延boot_delay有效信息（单位s），若此值大于允许最大值BOOT_DELAY_MAX，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*    * 收到延时v的2ms内未收到命令终止字，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*    * 读取BOOT_DELAY_ADDRESS起始的显示延时信息有效性的2个关键字，通过与预定的关键字对比，若不一致则进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*    * 根据延时boot_delay和关键字BOOT_DELAY_SIGNATURE1确定需存放在地址BOOT_DELAY_ADDRESS的数据，并写入以更新延时参数 */
		/*    * 再次读取地址BOOT_DELAY_ADDRESS的数据与需写入的数据进行对比，若不一致（写入失败），则进行命令失败处理，回告结构：（PROTO_INSYNC+PROTO_FAILED） */
		/*    * 运行成功，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */

		case PROTO_SET_DELAY: {
				/*
				  Allow for the bootloader to setup a
				  boot delay signature which tells the
				  board to delay for at least a
				  specified number of seconds on boot.
				 */
				int v = cin_wait(100);//suppose 30

				if (v < 0) {
					goto cmd_bad;
				}

				
				/* 取v的低字节作为时延boot_delay的有效信息（时延最大255s） */
				/* BOOT_DELAY_MAX：值30，飞控固件启动最大时延，定义在bl.c */
				/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
				uint8_t boot_delay = v & 0xFF;

				if (boot_delay > BOOT_DELAY_MAX) {
					goto cmd_bad;
				}

				// expect EOC
				if (!wait_for_eoc(2)) {
					goto cmd_bad;
				}


				
				/* 读取BOOT_DELAY_ADDRESS起始的2个字，提取显示延时信息有效性的2个关键字，通过对比来判断信息是否有效 */
				/* BUG?sig1的最低字节是延时参数，不应作为判断依据，这里有问题 */
				/* BOOT_DELAY_ADDRESS：值0x000001a0，飞控固件启动延时信息在flash中的存储地址，仅对主控FMU有效，定义在hw_config.h */
				/* BOOT_DELAY_SIGNATURE1：值0x92c2ecff，飞控固件启动延时信息有效的关键字1，定义在bl.h */
				/* BOOT_DELAY_SIGNATURE2：值0xc5057d5d，飞控固件启动延时信息有效的关键字2，定义在bl.h */
				/* cmd_bad：错误命令处理函数的标号，在bootloader函数末尾 */
				uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
				uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

				if (sig1 != BOOT_DELAY_SIGNATURE1 ||
				    sig2 != BOOT_DELAY_SIGNATURE2) {
					goto cmd_bad;
				}

				/* 根据延时boot_delay和关键字BOOT_DELAY_SIGNATURE1确定需存放在地址BOOT_DELAY_ADDRESS的数据，并写入 */
				/* BOOT_DELAY_SIGNATURE1：值0x92c2ecff，飞控固件启动延时信息有效的关键字1，定义在bl.h */
				/* BOOT_DELAY_ADDRESS：值0x000001a0，飞控固件启动延时信息在flash中的存储地址，仅对主控FMU有效，定义在hw_config.h */
				/* flash_func_write_word：烧写flash某地址起始为特定内容，4位烧写，定义在main_f?.c */
				uint32_t value = (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00) | boot_delay;//BOOT_DELAY_SIGNATURE1 的低两位设为 boot_delay
				flash_func_write_word(BOOT_DELAY_ADDRESS, value);


				
				/* 再次读取地址BOOT_DELAY_ADDRESS的数据与需写入的数据进行对比，若不一致则跳转到cmd_fail，运行失败 */
				/* BOOT_DELAY_ADDRESS：值0x000001a0，飞控固件启动延时信息在flash中的存储地址，仅对主控FMU有效，定义在hw_config.h */
				/* flash_func_read_word：读取flash某特定地址内容，4位读取，定义在main_f?.c */
				/* cmd_fail：指令运行错误处理的标号，在bootloader函数末尾 */
				if (flash_func_read_word(BOOT_DELAY_ADDRESS) != value) {
					goto cmd_fail;
				}
			}
			break;
#endif

		// finalise programming and boot the system
		//
		// command:			BOOT/EOC
		// reply:			INSYNC/OK
		//

		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （11）完成烧写并启动飞控固件PROTO_BOOT（0x30），命令结构：PROTO_BOOT+PROTO_EOC */
		/*	  * 收到启动命令PROTO_BOOT的1s内未收到命令终止字，进行错误指令处理，回告结构：(PROTO_INSYNC+PROTO_INVALID) */
		/*	  * 若变量first_word（飞控固件首字保存变量）中有内容 */
		/*		   * 烧写飞控固件首字first_word */
		/*		   * 读取飞控固件首字并与first_word对比，若不同则进行命令失败处理，回告结构：（PROTO_INSYNC+PROTO_FAILED） */
		/*		   * 烧写首字成功，将first_word变量置为无效（0xffffffff） */
		/*	  * 运行成功，发送PROTO_INSYNC+PROTO_OK，延时100ms，bootloader函数返回 */
		case PROTO_BOOT:

			// expect EOC
			if (!wait_for_eoc(1000)) {
				goto cmd_bad;
			}
			
			/* 若变量first_word内容有效，烧写飞控固件首字并判断是否烧写成功 */
			/* first_word：飞控固件首字保存变量，定义在bootloader函数中 */
			/* flash_func_write_word：烧写flash某地址起始为特定内容，4位烧写，定义在main_f?.c */
			/* flash_func_read_word：读取flash某特定地址内容，4位读取，定义在main_f?.c */
			/* cmd_fail：指令运行错误处理的标号，在bootloader函数末尾 */
			if (first_word != 0xffffffff && (bl_state & STATE_ALLOWS_REBOOT) != STATE_ALLOWS_REBOOT) { //要求完成过 GET_SYNC且目前未知未出过错
				goto cmd_bad;
			}

			// program the deferred first word
			if (first_word != 0xffffffff) {
				flash_func_write_word(0, first_word);

				if (flash_func_read_word(0) != first_word) {
					goto cmd_fail;
				}

				// revert in case the flash was bad...
				first_word = 0xffffffff;
			}

			// send a sync and wait for it to be collected
			sync_response();
			delay(100);

			// quiesce and jump to the app
			return;
		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* （12）输出调试信息（命令暂未完成）PROTO_DEBUG（0x31），直接按运行成功处理，(发送PROTO_INSYNC+PROTO_OK，变量timeout和bl_type重新赋值，继续新的命令处理) */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		case PROTO_DEBUG:
			// XXX reserved for ad-hoc debugging as required
			break;

		
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		/* 其他命令，直接进入接收新的命令循环 */
		/* ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
		default:
			continue;
		}

		// we got a command worth syncing, so kill the timeout because
		// we are probably talking to the uploader
		
		/* 4.5 命令处理完毕，运行成功后的处理（变量timeout初始化和bl_type重新赋值，调用sync_response函数发送PROTO_INSYNC+PROTO_OK表示运行成功），继续接收下一条指令。 */
		/* timeout：bootloader跳转到飞控固件所需等待的时间，bootloader函数输入 */
		/* bl_type：静态全局接口类型变量， */
		/* NONE：值0，枚举类，定义在bl.h */
		/* last_input：静态全局变量，上次获取串口数据的方式 */
		/* sync_response：发送同步命令函数，表示运行成功，定义在bl.c */
		timeout = 0;

		// Set the bootloader port based on the port from which we received the first valid command
		if (bl_type == NONE) {
			bl_type = last_input;//输入设备 USB or USART
		}

		// send the sync response for this command
		sync_response();
		continue;
cmd_bad:
		// send an 'invalid' response but don't kill the timeout - could be garbage
		invalid_response();
		bl_state = 0;
		continue;

cmd_fail:
		// send a 'command failed' response but don't kill the timeout - could be garbage
		failure_response();
		continue;

#if defined(TARGET_HW_PX4_FMU_V4) || defined(TARGET_HW_UVIFY_CORE)
bad_silicon:
		// send the bad silicon response but don't kill the timeout - could be garbage
		bad_silicon_response();
		continue;
#endif
	}
}
