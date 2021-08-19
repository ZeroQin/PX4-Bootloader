/************************************************************************
 *
 *	 Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *	 Copyright (c) 2010 libopencm3 project (Uwe Hermann, Stephen Caudle)
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
 *	 The PX4 development team considers libopencm3 to be
 *	 still GPL, not LGPL licensed, as it is unclear if
 *	 each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/**
 * @file stm32f4.ld
 *
 * Linker script for ST STM32F4 bootloader (use first 16K of flash, all 128K RAM).
 *
 * @author Uwe Hermann <uwe@hermann-uwe.de>
 * @author Stephen Caudle <scaudle@doceme.com>
 */

/* Define memory regions. */
/* 声明存储区域，分为rom和ram两部分 */
MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 16K /*定义rom区，即片内flash的部分区域，权限为可读可执行，起始于flash起始地址*/
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K/*定义ram区，即片内sram的部分区域，权限为可读可写可执行，起始于sram1起始地址*/
}

/* Enforce emmission of the vector table. */
/*声明外部定义的 vector_table 变量，此变量被定义再libopencm3/lib/cm3/vector.c 中*/
EXTERN (vector_table)

/* Define sections. */
SECTIONS				/* 定义段 */
{
	. = ORIGIN(rom);	/* 链接器指针定位于rom区起始地址 */

	.text : {			/* 定义代码段text 中断向量表*/
		*(.vectors)		/* Vector table 定义于libopencm3/lib/cm3/vector.c文件中，仅包含vector_table变量*/
		*(.text*)		/* Program code 程序代码段*/
		. = ALIGN(4);	/*当前地址4字节对齐*/
		*(.rodata*)		/* Read-only data程序只读数据段*/
		. = ALIGN(4);	/*当前地址4节z对齐*/
	} >rom				/*代码段text的程序地址位于rom中*/

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {				/*定义C++构造函数段preinit_array*/
		. = ALIGN(4);				/*当前地址4字节对齐*/
		__preinit_array_start = .;	/*定义变量__preinit_array_start为当前地址*/
		KEEP (*(.preinit_array))	/*强制链接器保留preinit_array段*/
		__preinit_array_end = .;	/*定义变量__preinit_aarray_end 为当前地址*/
	} >rom							/*C++构造函数段preinit_array位于rom中*/
	.init_array : {					/*定义C++构造函数段init_array */
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))	/*强制链接器保留init_array.*段，并对满足字符串模式的内容进行升序排列*/
		KEEP (*(.init_array))			/*强制链接器保留init_array段*/
		__init_array_end = .;
	} >rom
	.fini_array : {						/*定义C++析构函数段fini_array*/
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	. = ORIGIN(ram);					/*链接器指针定位于ram起始地址*/

	.data : {							/*定义数据段data*/
		_data = .;						/*定义变量_data为当前地址*/
		*(.data*)						/* Read-write initialized data ，程序数据段*/
		. = ALIGN(4);
		_edata = .;						/*定义变量_edata 为当前地址*/
	} >ram AT >rom						/*数据段data的程序地址位于ram中，加载地址位于rom中*/

	_data_loadaddr = LOADADDR(.data);	/*定义变量_data_*/

	.bss : {							/*定义bss段*/
	_sbss = .;
		*(.bss*)		/* Read-write zero initialized data,程序bss段 */
		*(COMMON)						/*COMMON段*/
		. = ALIGN(4);
		_ebss = .;
	} >ram								/*bss段的程序地址位于ram中*/

	/*
	 * The .eh_frame section appears to be used for C++ exception handling.
	 * You may need to fix this if you're using C++.
	 */
	/DISCARD/ : { *(.eh_frame) }			/* 被丢弃段eh_frame不会出现在输出文件中 */

	. = ALIGN(4);
	end = .;
}

PROVIDE(_stack = 0x20020000);				/* /* 定义变量_stack，仅在被使用时生效，堆栈指针指向ram区最高地址 */