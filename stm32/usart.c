/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * LICENSE NOTE FOR EXTERNAL LIBOPENCM3 LIBRARY:
 *
 *   The PX4 development team considers libopencm3 to be
 *   still GPL, not LGPL licensed, as it is unclear if
 *   each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/*
 * USART interface for the bootloader.
 */

#include "hw_config.h"

# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/usart.h>

#if !defined(USART_SR)
#define USART_SR USART_ISR
#endif
#include "bl.h"
#include "uart.h"

uint32_t usart;/* 全局变量，uart_cinit函数中被赋值为USART2 */

/* usart 初始化函数 uart_cinit，输入参数 config 为 void * 型指针，值为 USART 寄存器基地址。
*  此函数使用 SOC 上的串口寄存器进行设置，功能为 115200@8N1，收发通信，无硬件流控制。
*  在 pixahwk V2 硬件和 PX4 代码使用 USART 作为上位机通信串口，
*  config 值 USART2 的寄存器基地址（0x40004400）。*/

void
uart_cinit(void *config)
{
	usart = (uint32_t)config;/* 全局变量usart指向串口寄存器基地址 */

	/* board is expected to do pin and clock setup */

	/* do usart setup */
	//USART_CR1(usart) |= (1 << 15);	/* because libopencm3 doesn't know the OVER8 bit */
	/* 引脚和时钟初始化 */
	/* USART_CR1：根据给定的USART寄存器首地址移位0x0c来获取寄存器USART_CR1。 */
	/* 寄存器USART_CR1的bit15为OVER8域，控制串口的过采样模式。默认值0：16次过采样，值1：8次过采样。 */
	//USART_CR1(usart) |= (1 << 15);	/* libopencm3不支持USART_CR1寄存器的OVER8位操作 */
	/* USART_BAUDRATE：值115200，定义在hw_config.h */
	/* usart_set_baudrate：用于设置串口波特率，操作寄存器USART_BRR，定义在libopencm3/lib/stm32/common/usart_common_all.c */
	/* 库函数usart_set_baudrate用到了库全局变量rcc_apb1_frequency作为时钟输入。 */
	usart_set_baudrate(usart, USART_BAUDRATE);
	/* usart_set_databits：用于设置串口数据位数（8或9），操作寄存器USART_CR1域M（bit12），定
	*  义在libopencm3/lib/stm32/common/usart_common_all.c */
	usart_set_databits(usart, 8);
	/* USART_STOPBITS_1：值0x00<<12，表示停止位1（libopencm3/include/libopencm3/stm32/common/usart_common_all.h） */
	/* usart_set_stopbits：用于设置串口停止位数，操作寄存器USART_CR2域STOP（bit13:12），定义在libopencm3/lib/stm32/common/usart_common_all.c */
	usart_set_stopbits(usart, USART_STOPBITS_1);
	/* USART_MODE_TX_RX：值USART_CR1_RE|USART_CR1_TE，表示收+发（libopencm3/include/libopencm3/stm32/common/usart_common_all.h） */
	/* USART_CR1_RE：值1<<2（libopencm3/include/libopencm3/stm32/common/usart_common_f124.h） */
	/* USART_CR1_TE：值1<<3（libopencm3/include/libopencm3/stm32/common/usart_common_f124.h） */
	/* usart_set_mode：用于设置串口传输模式（单收，单发，收+发），操作寄存器USART_CR1域TE（bit3）和域RE（bit2），定义在libopencm3/lib/stm32/common/usart_common_all.c */
	usart_set_mode(usart, USART_MODE_TX_RX);
	
	/* USART_PARITY_NONE：值0x00，表示无校验（libopencm3/include/libopencm3/stm32/common/usart_common_all.h） */
	/* usart_set_parity：用于设置串口校验模式（无、奇、偶），操作寄存器USART_CR1域PCE（bit10）和域PS（bit9），定义在libopencm3/lib/stm32/common/usart_common_all.c */
	usart_set_parity(usart, USART_PARITY_NONE);
	/* USART_FLOWCONTROL_NONE：值0x00，表示无硬件流控制（libopencm3/include/libopencm3/stm32/common/usart_common_all.h） */
	/* usart_set_flow_control：用于设置硬件流控制（无、RTS、CTS、RTS+CTS），操作寄存器USART_CR3的CTSE（bit9）和RTSE（bit8） */
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

	/* and enable */
	/* usart_enable：用于使能串口，操作寄存器USART_CR1的域UE（bit13），定义在libopencm3/lib/stm32/common/usart_common_all.c */
	usart_enable(usart);


#if 0
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'B');

	while (true) {
		int c;
		c = usart_recv_blocking(usart);
		usart_send_blocking(usart, c);
	}

#endif
}
/* uart_cfini 函数的草组很简单，直接调用库函数 usart_disable 关闭串口，
*  USART2 串口的配置并不改变*/
void
uart_cfini(void)
{
	usart_disable(usart);
}

int 
uart_cin(void)
{
	int c = -1;		/* 默认返回值-1 */
 
	/* 若串口数据寄存器不为空，调用库函数读取接收到的数据 */
	/* USART_SR：USART状态寄存器，定义在libopencm3/include/libopencm3/stm32/common/usart_common_f124.h */
	/* USART_SR_RXNE：值1<<5，定义在libopencm3/include/libopencm3/stm32/common/usart_common_f124.h */
	/* usart：值USART2（USART2基地址），定义在usart.c */
	/* usart_recv：接收串口数据的库函数，定义在libopencm3/lib/stm32/common/usart_common_f124.c */
	if (USART_SR(usart) & USART_SR_RXNE) {
		c = usart_recv(usart);
	}
 
	return c;
}

void
uart_cout(uint8_t *buf, unsigned len)
{
	/* usart_send_blocking：USART串口发送库函数，仅当串口数据缓存为空时再发送下一个数据*/
	while (len--) {
		usart_send_blocking(usart, *buf++);
	}
}
