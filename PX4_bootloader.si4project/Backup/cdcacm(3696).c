/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project (Gareth McMullin)
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

/**
 * @file cdcacm.c
 * @author Gareth McMullin <gareth@blacksphere.co.nz>
 * @author David Sidrane <david_s5@nscdg.com>
 */
#include "hw_config.h"

#include <stdlib.h>


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "bl.h"
#if INTERFACE_USB != 0
#define USB_CDC_REQ_GET_LINE_CODING			0x21 // Not defined in libopencm3

#ifndef OTG_GOTGCTL_BVALOVAL
#define OTG_GOTGCTL_BVALOVAL   (1 << 7)
#endif
#ifndef OTG_GOTGCTL_BVALOEN
#define OTG_GOTGCTL_BVALOEN    (1 << 6)
#endif

/*
 * ST changed the meaning and sense of a few critical bits
 * in the USB IP block identified as 0x00002000
 * libopencm3 has failed to merge my PR to fix this
 * So the the following are defined to fix the issue
 * herein.
 */
#define OTG_CID_HAS_VBDEN 0x00002000
#define OTG_GCCFG_VBDEN   (1 << 21)
/*
 * The B-peripheral session valid override (BVALOVAL and BVALOEN) bits got added
 * in OTG FS CID 0x00002000
 */
#define OTG_CID_HAS_BVALOVAL 0x00002000

/* Provide the stings for the Index 1-n as a requested index of 0 is used for the supported langages
 *  and is hard coded in the usb lib. The array below is indexed by requested index-1, therefore
 *  element[0] maps to requested index 1
 */
 
/* USB设备字符串列表，下标从1开始 */
static const char *usb_strings[] = {
	USBMFGSTRING, /* Maps to Index 1 Index *//* USBMFGSTRING：值"3D Robotics"，生产厂家名称，INDEX 1。（hw_config.h） */
	USBDEVICESTRING,/* USBDEVICESTRING："PX4 BL FMU v2.x"，USB设备名称，INDEX2。（hw_config.h） */
	"0",	/* 设备序号，INDEX 3 */
};
	
/* 上面定义的字符串数组usb_strings的成员个数 */
#define NUM_USB_STRINGS (sizeof(usb_strings)/sizeof(usb_strings[0]))

static usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

/* 标准的USB设备描述符结构体变量dev，属性packed，
*  定义在libopencm3/include/libopencm3/usb/usbstd.h */
static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,		/*值18，本结构体的大小（libopencm3/include/libopencm3/usb/usbstd.h） */
	.bDescriptorType = USB_DT_DEVICE,	/**< Specifies the descriptor type  值1，本结构体为USB设备描述符*/
	.bcdUSB = 0x0200,					/**USB接口版本，0x0200表示符合USB2.0规范< The USB interface version, binary coded (2.0) */
	.bDeviceClass = USB_CLASS_CDC,		/**值0x2，设备群组码< USB device class, CDC in this case */
	.bDeviceSubClass = 0,				/*设备次群组码*/
	.bDeviceProtocol = 0,				/*通信设备协议*/
	.bMaxPacketSize0 = 64,				//设备最大信息包大小为64字节（只能为8，16，32，64）
	.idVendor = USBVENDORID,			/**值0x26AC,USB设备厂家ID< Vendor ID (VID) */
	.idProduct = USBPRODUCTID,			/**值0x0011,USB产品ID< Product ID (PID) */
	.bcdDevice = 0x0101,				/**USB设备发行序号，0x0101表示1.01版，俄日了兼容NuttX< Product version. Set to 1.01 (0x0101) to agree with NuttX */
	.iManufacturer = 1,					/**USB设备厂家字符串下标，1表示USBFGSTRING（”3D Robotics“），数组usb_strings< Use string with index 1 for the manufacturer string ("3D Robotics") */
	.iProduct = 2,						/**USB产品字符串下标，2表示USBDEVICESTRING("PX4 BL FMU v2.x"),数组usb_strings< Use string with index 2 for the product string (USBDEVICESTRING define) */
	.iSerialNumber = 3,					/**USB设备序号下标，3代表空（"0"）< Use string with index 3 for the serial number string (empty) */
	.bNumConfigurations = 1,			/**USB设备配置的数目为1个< Number of configurations (one) */
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
/* USB通信端点描述符结构体数组变量comm_endp，属性packed，
*  定义在libopencm3/include/libopencm3/usb/usbstd.h 
*  这里仅对端点描述符的内容进行了赋值，结构体规定的额外内容在UNIX环境中被默认为0 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
		.bLength = USB_DT_ENDPOINT_SIZE,					/* USB_DT_ENDPOINT_SIZE：值7，端点描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bDescriptorType = USB_DT_ENDPOINT,				/* USB_DT_ENDPOINT：值5，表示本结构体为USB端点描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bEndpointAddress = 0x83,						/* 通信端点地址为0x83 */
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,		/* USB_ENDPOINT_ATTR_INTERRUPT：值0x03，本通信端点为中断传输（libopencm3/include/libopencm3/usb/usbstd.h） */
		.wMaxPacketSize = 16,							/* 数据包最大16字节 */
		.bInterval = 255,								/* 轮询间隔255ms */
	}
};
/* USB数据端点描述符结构体数组变量data_endp（2个，一般称为DATA0和DATA1），属性packed，
*  定义在libopencm3/include/libopencm3/usb/usbstd.h 
*  这里仅对端点描述符的内容进行了赋值，结构体规定的额外内容在UNIX环境中被默认为0 */

static const struct usb_endpoint_descriptor data_endp[] = {{
		.bLength = USB_DT_ENDPOINT_SIZE,					/* USB_DT_ENDPOINT_SIZE：值7，端点描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bDescriptorType = USB_DT_ENDPOINT, 			/* USB_DT_ENDPOINT：值5，表示本结构体为USB端点描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bEndpointAddress = 0x01,						/* 此数据端点的地址为0x01 */
		.bmAttributes = USB_ENDPOINT_ATTR_BULK, 		/* USB_ENDPOINT_ATTR_BULK：值0x2，本数据端点为批量传输（libopencm3/include/libopencm3/usb/usbstd.h） */
		.wMaxPacketSize = 64,							/* 数据包最大64字节 */
		.bInterval = 1, 								/* 轮询间隔1ms */
	}, {
		.bLength = USB_DT_ENDPOINT_SIZE,					/* USB_DT_ENDPOINT_SIZE：值7，端点描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bDescriptorType = USB_DT_ENDPOINT, 			/* USB_DT_ENDPOINT：值5，表示本结构体为USB端点描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bEndpointAddress = 0x82,						/* 此数据端点的地址为0x82 */
		.bmAttributes = USB_ENDPOINT_ATTR_BULK, 		/* USB_ENDPOINT_ATTR_BULK：值0x2，本数据端点为批量传输（libopencm3/include/libopencm3/usb/usbstd.h） */
		.wMaxPacketSize = 64,							/* 数据包最大64字节 */
		.bInterval = 1, 								/* 轮询间隔1ms */

	}
};
/* 对USB CDC通信需要额外提供的信息进行汇总，用于到关联通信接口描述符comm_iface */
static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
		sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	}
};
/* USB通信接口描述符结构体变量comm_iface，属性packed，
*  定义在libopencm3/include/libopencm3/usb/usbstd.h */
static const struct usb_interface_descriptor comm_iface[] = {{
		.bLength = USB_DT_INTERFACE_SIZE,			/* USB_DT_INTERFACE_SIZE：值9，接口描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bDescriptorType = USB_DT_INTERFACE,			/* USB_DT_INTERFACE：值4，表示本结构体为USB接口描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bInterfaceNumber = 0,						/* 接口数目1个，以0为基准开始计数 */
		.bAlternateSetting = 0,						/* 交互设置值为0 */
		.bNumEndpoints = 1,							/* 端点数目1个（comm_endp） */
		.bInterfaceClass = USB_CLASS_CDC,			/* USB_CLASS_CDC：值0x02，接口群组（libopencm3/include/libopencm3/usb/cdc.h） */
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,	/* USB_CDC_SUBCLASS_ACM：值0x02，接口次群组（libopencm3/include/libopencm3/usb/cdc.h） */
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,	/* USB_CDC_PROTOCOL_AT：值0x01，接口协议（libopencm3/include/libopencm3/usb/cdc.h） */
		.iInterface = 0,								/* 接口的字符串描述符索引为0 */
 
		/* 以下内容不再属于USB接口描述符信息，为程序内部使用的变量 */
		.endpoint = comm_endp,						/* 此接口对应的端点为comm_endp */
		.extra = &cdcacm_functional_descriptors,		/* 通信接口额外信息域为cdcacm_functional_descriptors */
		.extralen = sizeof(cdcacm_functional_descriptors)		/* 额外信息域长度 */

	}
};

/* USB数据接口描述符结构体变量data_iface，属性packed，定义在libopencm3/include/libopencm3/usb/usbstd.h */
/* 这里仅进行了部分赋值，结构体规定的额外内容在UNIX环境中被默认为0 */
static const struct usb_interface_descriptor data_iface[] = {{
		.bLength = USB_DT_INTERFACE_SIZE,			/* USB_DT_INTERFACE_SIZE：值9，接口描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bDescriptorType = USB_DT_INTERFACE,			/* USB_DT_INTERFACE：值4，表示本结构体为USB接口描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
		.bInterfaceNumber = 1,						/* 接口数目2个，以0为基准开始计数 */
		.bAlternateSetting = 0, 					/* 交互设置值为0 */
		.bNumEndpoints = 2, 						/* 端点数目2个（data_endp） */
		.bInterfaceClass = USB_CLASS_DATA,			/* USB_CLASS_DATA：值0x0a，接口群组（libopencm3/include/libopencm3/usb/cdc.h） */
		.bInterfaceSubClass = 0,						/* 接口次群组为0 */
		.bInterfaceProtocol = 0,						/* 接口协议，0表示无特定协议 */
		.iInterface = 0,								/* 接口的字符串描述符索引为0，无效 */
 
		/* 以下内容不再属于USB接口描述符信息，为程序内部使用的变量 */
		.endpoint = data_endp,						/* 此接口对应的端点为data_endp */

	}
};
/* 变量ifaces对接口描述符进行了统计（共2个），用于关联到配置描述符中 */
static const struct usb_interface ifaces[] = {{
		.num_altsetting = 1,
		.altsetting = comm_iface,
	}, {
		.num_altsetting = 1,
		.altsetting = data_iface,
	}
};

/* USB配置描述符结构体变量config，属性packed，定义在libopencm3/include/libopencm3/usb/usbstd.h */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,		/* USB_DT_CONFIGURATION_SIZE：值9，配置描述符大小（libopencm3/include/libopencm3/usb/usbstd.h） */
	.bDescriptorType = USB_DT_CONFIGURATION,		/* USB_DT_CONFIGURATION：值2，表示本结构体为USB配置描述符（libopencm3/include/libopencm3/usb/usbstd.h） */
	.wTotalLength = 0,							/* 描述符的总长度为0（是否应该为32字节？接口描述符9+配置描述符9+端点描述符7*2） */
	.bNumInterfaces = 2,							/* 该配置支持的接口数目2个（comm_iface和data_iface） */
	.bConfigurationValue = 1,					/* 配置值为1，作为set configuration请求的配置值 */
	.iConfiguration = 0,							/* 配置字符串描述符索引为0，无效 */
	.bmAttributes = 0x80,						/* 配置的属性为0x80 */
	.bMaxPower = 0xFA,							/* 当USB设备操作时，它从总线上获得的最大电源（单位2mA，0xFA=250，500mA） */
 
	/* 以下内容不再属于USB配置描述符信息，为程序内部使用的变量 */
	.interface = ifaces,							/* 此配置对应的接口 */
};


/* USB虚拟串口属性结构体变量line_coding，属性packed，定义在libopencm3/include/libopencm3/usb/cdc.h */
static const struct usb_cdc_line_coding line_coding = {
	.dwDTERate = 115200,						/* 波特率115200 */
	.bCharFormat = USB_CDC_1_STOP_BITS,		/* USB_CDC_1_STOP_BITS：值0，1位停止位（libopencm3/include/libopencm3/usb/cdc.h） */
	.bParityType = USB_CDC_NO_PARITY,		/* USB_CDC_NO_PARITY：值0，无校验（libopencm3/include/libopencm3/usb/cdc.h） */
	.bDataBits = 0x08						/* 数据8位 */
};


 
/* USB虚拟串口请求控制函数，被配置函数cdcacm_set_config函数使用 */
static enum usbd_control_buffer cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
								usbd_control_complete_callback *complete)
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;
 
	switch (req->bRequest) {		/* 请求类型 */
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {	/* USB_CDC_REQ_SET_CONTROL_LINE_STATE：值0x22，请求设置命令行状态，定义在libopencm3/include/libopencm3/usb/cdc.h */
			return 1;			/* 不操作，直接返回1 */
		}
	case USB_CDC_REQ_SET_LINE_CODING:			/* USB_CDC_REQ_SET_LINE_CODING：值0x20，请求设置虚拟串口属性，定义在libopencm3/include/libopencm3/usb/cdc.h */
		if (*len < sizeof(struct usb_cdc_line_coding)) {		/* 若配置的参数长度小于usb_cdc_line_coding的大小，返回0 */
			return 0;
		}
		return 1;				/* 不操作，返回1 */
	case USB_CDC_REQ_GET_LINE_CODING:			/* USB_CDC_REQ_GET_LINE_CODING：值0x21，请求读取虚拟串口属性，定义在cdcacm.c中 */
		*buf = (uint8_t *)&line_coding;			/* buf指向虚拟串口属性变量line_coding（115200@8N1），定义在cdcacm.c中 */
		return 1;
	}
	return 0;
}


/* USB虚拟串口数据接收函数，被配置函数cdcacm_set_config函数使用 */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
 
	char buf[64];
	unsigned i;
	/* usbd_ep_read_packet：去读USB设备usbd_dev在端点地址0x01的数据到缓冲区buf中，并返回读取到数据的个数。定义在libopencm3/lib/usb/usb.c */
	unsigned len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));
 
	/* 将数据赋值给接收数据缓冲区rx_buf */
	/* buf_put：将数据写入到缓冲区的函数，定义在bl.c */
	for (i = 0; i < len; i++) {
		buf_put(buf[i]);	
	}
}


/* USB虚拟串口配置函数，被usb_cinit中的库函数usbd_register_set_config_callback引用 */
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
 
	/* USB_ENDPOINT_ATTR_BULK：值0x2，批量传输（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* USB_ENDPOINT_ATTR_INTERRUPT：值0x03，中断传输（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* cdcacm_data_rx_cb：USB虚拟串口数据接收函数，将数据赋值给接收数据缓冲区rx_buf（bl.c），定义在cdcacm.c */
	/* usbd_ep_setup：设置USB端点地址、类型、FIFO存储空间和回调函数 */
	/* 第一行：地址为0x01的USB端点类型为批量传输模式（USB_ENDPOINT_ATTR_BULK），缓冲区长度64字节，回调函数为cdcacm_data_rx_cb，用于接收USB数据 */
	/* 第二行：地址为0x82的USB端点类型为批量传输模式（USB_ENDPOINT_ATTR_BULK），缓冲区长度64字节，无回调函数，用于发送USB数据 */
	/* 第三行：地址为0x83的USB端点类型为中断传输模式（USB_ENDPOINT_ATTR_INTERRUPT），缓冲区长度16字节，无回调函数，用于USB传输控制 */
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
 
	/* USB_REQ_TYPE_CLASS：值0x20（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* USB_REQ_TYPE_INTERFACE：值0x01（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* USB_REQ_TYPE_TYPE：值0x60（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* USB_REQ_TYPE_RECIPIENT：值0x1F（libopencm3/include/libopencm3/usb/usbstd.h） */
	/* cdcacm_control_request：USB虚拟串口请求控制函数，功能为处理各种USB请求信息，定义在cdcacm.c */
	/* usbd_register_control_callback：USB寄存器控制请求回调函数，定义在libopencm3/lib/usb/usb_control.c */
	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS|USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE|USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);
}




/* USB OTG FS中断处理函数 */
void otg_fs_isr(void)
{
	if (usbd_dev) {
		/* usbd_poll：USB设备的POLL操作；定义在libopencm3/lib/usb/usb.c */
		usbd_poll(usbd_dev);
	}
}


/* 根据 CDC ACM 虚拟串口协议对板载 USB OTG FS 进行设备初始化，115200@8N1，收发通信。
*  本函数使用了 USB 的设备描述符（usb_device_descriptor）、配置描述符（usb_config_descriptor）、
*  接口描述符（usb_interface_descriptor）、端点描述符（usb_endpoint_descriptor），
*  这些描述符的定义都在 libopencm3/include/libopencm3/usb/usbstd.h 中。这些描述符支撑了一个
*  全局使用的_usbd_device 型结构体指针 usbd_dev，它包含了 USB 的所有配置信息和操作函数，
*  对它成员的配置就是本函数的核心。配置完成后 USB 接口的操作就非常方便了。*/
void 
usb_cinit(void)
{
#if defined(STM32F4)			/* 宏STM32F4值为1，定义在Makefile.f4中，下列代码有效 */
	/* 1. 启动USB接口时钟，对应原理图中GPIOA9(VBUS/3.1A)，GPIOA11(OTG_FS_DM/3.1B)，GPIOA12(OTG_FS_DP/3.1A) */
	/* RCC_AHB1ENR：寄存器（地址0x40023830），定义在libopencm3/include/libopencm3/stm32/f4/rcc.h */
	/* RCC_AHB2ENR：寄存器（地址0x40023834），定义在libopencm3/include/libopencm3/stm32/f4/rcc.h */
	/* RCC_AHB1ENR_IOPAEN：值1<<0（libopencm3/include/libopencm3/stm32/f4/rcc.h） */
	/* RCC_AHB2ENR_OTGFSEN：值1<<7（libopencm3/include/libopencm3/stm32/f4/rcc.h） */
	/* rcc_peripheral_enable_clock：使能特定的外部时钟，被定义在libopencm3/lib/stm32/common/rcc_common_all.c */
	/* 第一行代码使能GPIOA的时钟，操作寄存器RCC_AHB1ENR的域GPIOAEN(bit0) */
	/* 第二行代码使能USB OTG FS时钟，操作寄存器RCC_AHB2ENR的域OTGFSEN(bit7) */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);
 
#if defined(USB_FORCE_DISCONNECT)		/* 宏USB_FORCE_DISCONNECT未定义，下列代码无效 */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_OTYPE_OD, GPIO12);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
	/* Spec is 2-2.5 uS */
	delay(1);
	systick_interrupt_disable();
	systick_counter_disable(); // Stop the timer
#endif
 
	/* 2. 配置GPIOA11和GPIOA12分别USB数据通信功能 */
	/* GPIOA：地址为0x40020000的寄存器（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO_MODE_AF：值0x2（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO_PUPD_NONE：值0x0（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO11：值1<<11（libopencm3/include/libopencm3/stm32/common/gpio_common_all.h） */
	/* GPIO12：值1<<12（libopencm3/include/libopencm3/stm32/common/gpio_common_all.h） */
	/* GPIO_AF10：值0xa（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* gpio_mode_setup：设置引脚工作模式，定义在libopencm3/lib/stm32/common/gpio_common_f0234.c */
	/* gpio_set_af：设置引脚具体的AF功能，定义在libopencm3/lib/stm32/common/gpio_common_f0234.c */
	/* 第一行代码设置GPIOA11（OTG_FS_DM/3.1B）和GPIOA12（OTG_FS_DP/3.1A）为AF功能（MODER=GPIO_MODE_AF=10），push-pull模式（PUPDR=GPIO_PUPD_NONE=0x0） */
	/* 第二行代码设置GPIOA11和GPIOA12的具体AF功能为OTGFS/OTGHS（AFRH10=AFRH9=GPIO_AF10=0xa） */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
 
#if defined(BOARD_USB_VBUS_SENSE_DISABLED)	/* 宏BOARD_USB_VBUS_SENSE_DISABLED未定义，下列代码无效 */
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
#endif
 
	/* 3. USB设备的初始化配置 */
	/* usbd_dev：_usbd_device型结构体全局指针变量，包含一系列USB信息与操作函数（libopencm3/lib/usb/usb_private.h） */
	/* otgfs_usb_driver：_usbd_driver型结构体变量stm32f107_usb_driver，包含一系列USBFS驱动函数（libopencm3/lib/usb/usb_f107.c） */
	/* dev：USB设备描述符usb_device_descriptor，包含USB设备的基本信息（libopencm3/include/libopencm3/usb/usbstd.h），初始化在cdcacm.c */
	/* config：USB配置描述符usb_config_descriptor，包含USB设备的配置信息（libopencm3/include/libopencm3/usb/usbstd.h），初始化在cdcacm.c */
	/* usb_strings：USB设备字符串列表，定义并被初始化在cdcacm.c */
	/* NUM_USB_STRINGS：值3，usb_strings的数量，宏定义在cdcacm.c */
	/* usbd_control_buffer：控制操作请求缓冲区，128字节，定义在cdcacm.c */
	/* usbd_init：usb接口初始化函数，目的是填写结构体变量usbd_dev，定义在libopencm3/lib/usb/usb.c */
	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, NUM_USB_STRINGS, usbd_control_buffer, sizeof(usbd_control_buffer));
 
#elif defined(STM32F1)		/* 宏STM32F1未定义（IO协处理器不执行usb_cinit），下列代码无效 */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	gpio_set(GPIOA, GPIO8);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, NUM_USB_STRINGS, usbd_control_buffer, sizeof(usbd_control_buffer));
#endif
 
	/* 4. USB传输回调函数设置 */
	/* usbd_dev：_usbd_device型结构体全局指针变量，包含一系列USB信息与操作函数（libopencm3/lib/usb/usb_private.h） */
	/* cdcacm_set_config：USB虚拟串口配置函数，配置USB端点的操作函数和USB寄存器的请求操作函数，定义在cdcmca.c */
	/* usbd_register_set_config_callback：追加回调函数cdcacm_set_config到设备描述符usbd_dev中 */
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
 
#if defined(STM32F4)			/* 宏STM32F4值为1，定义在Makefile.f4中，下列代码有效 */
 
	/* 5. 若USB产品的ID号为0x00002000，启动USB连接功能 */
	/* OTG_FS_CID：寄存器，用于存储USB产品ID，软件可更改（libopencm3/include/libopencm3/stm32/otg_fs.h） */
	/* OTG_CID_HAS_VBDEN：值0x00002000，定义在cdcacm.c */
	/* OTG_FS_GCCFG：寄存器（libopencm3/include/libopencm3/stm32/otg_fs.h） */
	/* OTG_GCCFG_VBDEN：值1<<21，置该位认为VBUS一直有效，不再受外围电路影响（cdcacm.c） */
	/* OTG_GCCFG_PWRDWN：值1<<16，置该位开启收发功能（libopencm3/include/libopencm3/stm32/otg_common.h） */
	/* OTG_FS_DCTL：寄存器（libopencm3/include/libopencm3/stm32/otg_fs.h） */
	/* OTG_DCTL_SDIS：值1<<1，该位由软件控制USB核心开启软件连接功能（libopencm3/include/libopencm3/stm32/otg_common.h） */
	if (OTG_FS_CID == OTG_CID_HAS_VBDEN) {
		OTG_FS_GCCFG |= OTG_GCCFG_VBDEN | OTG_GCCFG_PWRDWN;
		/* 对于STM32F446/469，启动后需要进行软件连接 */
		OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
	}
 
	/* 6. 使能USB OTG FS中断 */
	/* NVIC_OTG_FS_IRQ：值67，对应USB OTG FS的全局中断向量号，libopencm3/include/libopencm3/stm32/f4/nvic.h（编译时自动生成文件） */
	/* nvic_enable_irq：使能中断向量，定义在libopencm3/lib/cm3/nvic.c */
	nvic_enable_irq(NVIC_OTG_FS_IRQ);
#endif
}

/* USB 虚拟串口反向初始化函数 usb_cfini，并未改变 USB 的接口配置，主要操作如下：
*  关闭 USB OTG FS 中断
*  关闭 USB 软连接，并清空 usbd_dev 指针
*  配置 USB 数据引脚 GPIOA11 和 GPIOA12 的工作模式
*  关闭 USB OTG FS 时钟*/
void
usb_cfini(void)
{
#if defined(STM32F4)			/* 宏STM32F4值为1，定义在Makefile.f4中，下列代码有效 */
	/* 1. 关闭USB OTG FS中断 */
	/* NVIC_OTG_FS_IRQ：值67，对应USB OTG FS的全局中断向量号，libopencm3/include/libopencm3/stm32/f4/nvic.h（编译时自动生成文件） */
	/* nvic_disable_irq：使能中断向量，定义在libopencm3/lib/cm3/nvic.c */
	nvic_disable_irq(NVIC_OTG_FS_IRQ);
#endif
 
	/* 2.关闭USB软连接，并清空usbd_dev指针 */
	/* usbd_dev：usbd_device型全局变量指针，指向USB的配置信息和操作函数，被usbd_init库函数初始化，cdcacm.c */
	/* usbd_disconnect：断开USB软连接（不是所有的MCU都支持）。定义在libopencm3/lib/usb/usb.c */
	if (usbd_dev) {
		usbd_disconnect(usbd_dev, true);
		usbd_dev = NULL;
	}
 
#if defined(STM32F4)		/* 宏STM32F4值为1，定义在Makefile.f4中，下列代码有效 */
	/* 3. 配置USB数据引脚GPIOA11和GPIOA12为输入 */
	/* GPIOA：地址为0x40020000的寄存器（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO_MODE_INPUT：值0x0（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO_PUPD_NONE：值0x0（libopencm3/include/libopencm3/stm32/common/gpio_common_f234.h） */
	/* GPIO11：值1<<11（libopencm3/include/libopencm3/stm32/common/gpio_common_all.h） */
	/* GPIO12：值1<<12（libopencm3/include/libopencm3/stm32/common/gpio_common_all.h） */
	/* gpio_mode_setup：设置引脚工作模式，定义在libopencm3/lib/stm32/common/gpio_common_f0234.c */
	/* 设置GPIOA11（OTG_FS_DM/3.1B）和GPIOA12（OTG_FS_DP/3.1A）为输入功能（MODER=GPIO_MODE_AF=00），浮空模式（PUPDR=GPIO_PUPD_NONE=0x0） */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
 
	/* 4. 关闭USB OTG FS时钟 */
	/* RCC_AHB2ENR：寄存器（地址0x40023834），定义在libopencm3/include/libopencm3/stm32/f4/rcc.h */
	/* RCC_AHB2ENR_OTGFSEN：值1<<7（libopencm3/include/libopencm3/stm32/f4/rcc.h） */
	/* rcc_peripheral_disable_clock：关闭特定的外部时钟，被定义在libopencm3/lib/stm32/common/rcc_common_all.c */
	/* 关闭USB OTG FS时钟，操作寄存器RCC_AHB2ENR的域OTGFSEN(bit7) */
	rcc_peripheral_disable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);
 
#elif defined(STM32F1)		/* 宏STM32F1未定义（IO协处理器不执行usb_cfini），下列代码无效 */
	/* Reset the USB pins to being floating inputs */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);
	gpio_clear(GPIOA, GPIO8);
#endif
}



int 
usb_cin(void)
{
	/* 若usbd_dev为空指针，则USB设备未被初始化，返回错误 */
	/* usbd_dev：_usbd_device型结构体全局指针变量，包含一系列USB信息与操作函数，定义在cdcacm.c */
	if (usbd_dev == NULL) { return -1; }
#if defined(STM32F1)			/* 主控FMU未定义，代码无效 */
	usbd_poll(usbd_dev);
#endif
	/* buf_get：从串口缓冲区内读取1个字的内容，定义在bl.c */
	return buf_get();
}

void
usb_cout(uint8_t *buf, unsigned count)
{
	if (usbd_dev) {	/* 若USB设备有效 */
				/* 将需要写入的数据分成64字节1包，逐包写入到地址为0x82的端点中 */
		while (count) {
			unsigned len = (count > 64) ? 64 : count;
			unsigned sent;

			sent = usbd_ep_write_packet(usbd_dev, 0x82, buf, len);

			count -= sent;
			buf += sent;
		}
	}
}
#endif
