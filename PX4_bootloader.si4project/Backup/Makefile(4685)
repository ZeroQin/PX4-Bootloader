#
# Common Makefile for the PX4 bootloaders
#

#
# Paths to common dependencies
#
export BUILD_DIR_ROOT ?= build
export BL_BASE		?= $(wildcard .)#本地工程所在目录
export LIBOPENCM3	?= $(wildcard libopencm3)#libopencm3 所在目录
export LIBKINETIS  	?= $(wildcard lib/kinetis/NXP_Kinetis_Bootloader_2_0_0)#飞思卡尔的 bootloader
MKFLAGS=--no-print-directory

SRC_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

COLOR_BLUE = \033[0;94m
NO_COLOR   = \033[m

define colorecho
+@echo -e '${COLOR_BLUE}${1} ${NO_COLOR}'
endef

#
# Tools
#
export CC	 	 = arm-none-eabi-gcc#定义交叉编译工具
export OBJCOPY		 = arm-none-eabi-objcopy#定义二进制生成工具链变量

#
# Common configuration
#-std=gnu99 \#使用GNU99的优化C语言标准
#-Os \#专门针对生成目标文件大小进行
#-g \#生成调试信息
#-Wundef \#当没有定义的符号出现在#if中时警告
#-Wall \#打开一些有用的警告选项
#-fno-builtin \#不接收没有 __buildin__ 前缀的函数作为内建函数
#-I$(BL_BASE)/$(LIBOPENCM3)/include \#包含开源库 libopencm3 的头文件
#-I$(BL_BASE)/. \#包含当前目录
#-ffunction-sections \#要求编译器未每个function分配独立的section
#-nostartfiles \#链接时不使用标准的启动文件 ?
#-lnosys \#链接 libnosys.a 文件,其中所有函数都是空的；程序并不实际使用系统函数，但是某些代码引用了系统函数，引入 libnosys，以便通过编译
#-Wl,-gc-sections \#传递-gc-section 给链接器，删除没有使用的section
#-Wl,-g \#传递-g 选项给链接器，兼容其它工具
#-Werror#把警告当作错误，出现警告就放弃编译
#
export FLAGS		 = -std=gnu99\
			   -Os \
			   -g \
			   -Wundef \
			   -Wall \
			   -fno-builtin \
			   -I$(BL_BASE)/$(LIBOPENCM3)/include \
			   -I$(BL_BASE)/. \
			   -ffunction-sections \
			   -nostartfiles \
			   -lnosys \
			   -Wl,-gc-sections \
			   -Wl,-g \
			   -Werror

ifneq ($(CRYPTO_HAL),)
include crypto_hal/$(CRYPTO_HAL)/Makefile.include
endif

export COMMON_SRCS	 = bl.c $(CRYPTO_SRCS)#定义通用源文件变量

export ARCH_SRCS	 = cdcacm.c  usart.c


#
# Bootloaders to build
# Note: px4fmuv3_bl is the same as px4fmuv2_bl except for a different USB device
# string
##定义编译目标
TARGETS	= \
	aerofcv1_bl \
	auavx2v1_bl \
	avx_v1_bl \
	crazyflie21_bl \
	crazyflie_bl \
	cube_f4_bl \
	fmuk66e_bl \
	fmuk66v3_bl \
	kakutef7_bl \
	mindpxv2_bl \
	modalai_fc_v1_bl \
	omnibusf4sd_bl \
	pix32v5_bl \
	px4aerocore_bl \
	px4discovery_bl \
	px4flow_bl \
	px4fmu_bl \
	px4fmuv2_bl \
	px4fmuv3_bl \
	px4fmuv4_bl \
	px4fmuv4pro_bl \
	px4fmuv5_bl \
	px4fmuv5x_bl \
	px4io_bl \
	px4iov3_bl \
	smartap_airlink_bl \
	smartap_pro_bl \
	tapv1_bl \
	uvify_core_bl
#编译目标all=$(TARGETS) sizes
all:	$(TARGETS) sizes
#定义清理工程的操作
clean:
	cd libopencm3 && make --no-print-directory clean && cd ..
	rm -f *.elf *.bin *.map # Remove any elf or bin files contained directly in the Bootloader directory
	rm -rf build # Remove build directories

#
# Specific bootloader targets.
#各编译目标的具体操作，主要规定了编译需要使用的Makefile文件，硬件目标TARGET_HW，链接脚本LINKER_FILE，编译目标TARGET_FILE_NAME
#

fmuk66v3_bl: $(MAKEFILE_LIST) $(LIBKINETIS)
	${MAKE} ${MKFLAGS} -f  Makefile.k66 TARGET_HW=FMUK66_V3  LINKER_FILE=kinetisk66.ld TARGET_FILE_NAME=$@

fmuk66e_bl: $(MAKEFILE_LIST) $(LIBKINETIS)
	${MAKE} ${MKFLAGS} -f  Makefile.k66 TARGET_HW=FMUK66_E  LINKER_FILE=kinetisk66.ld TARGET_FILE_NAME=$@

auavx2v1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=AUAV_X2V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

kakutef7_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=KAKUTEF7 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

pix32v5_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=HOLYBRO_PIX32_V5 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

px4fmu_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V2  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv3_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V3  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4fmuv4pro_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FMU_V4_PRO LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@ EXTRAFLAGS=-DSTM32F469

px4fmuv5_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=PX4_FMU_V5 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

px4fmuv5x_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=PX4_FMU_V5X LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

mindpxv2_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=MINDPX_V2 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4discovery_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_DISCOVERY_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4flow_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_FLOW_V1  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

px4aerocore_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=PX4_AEROCORE_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

crazyflie_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CRAZYFLIE LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

crazyflie21_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CRAZYFLIE21 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

omnibusf4sd_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=OMNIBUSF4SD LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

cube_f4_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=CUBE_F4  LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

cube_f7_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=CUBE_F7 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

avx_v1_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=AV_X_V1 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

smartap_airlink_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=SMARTAP_AIRLINK LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

smartap_pro_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=SMARTAP_PRO LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

modalai_fc_v1_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f7 TARGET_HW=MODALAI_FC_V1 LINKER_FILE=stm32f7.ld TARGET_FILE_NAME=$@

uvify_core_bl:$(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=UVIFY_CORE LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

# Default bootloader delay is *very* short, just long enough to catch
# the board for recovery but not so long as to make restarting after a
# brownout problematic.
#
px4io_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f1 TARGET_HW=PX4_PIO_V1 LINKER_FILE=stm32f1.ld TARGET_FILE_NAME=$@

px4iov3_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f3 TARGET_HW=PX4_PIO_V3 LINKER_FILE=stm32f3.ld TARGET_FILE_NAME=$@

tapv1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=TAP_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

aerofcv1_bl: $(MAKEFILE_LIST) $(LIBOPENCM3)
	${MAKE} ${MKFLAGS} -f  Makefile.f4 TARGET_HW=AEROFC_V1 LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

#
# Show sizes
#
.PHONY: sizes
sizes:
	@-find build/*/ -name '*.elf' -type f | xargs size 2> /dev/null || :

#
# Binary management
#二进制文件压缩操作：deploy命令
#
.PHONY: deploy
deploy:
	zip -j Bootloader.zip build/*/*.bin

#
# Submodule management
#子模块libopencm3操作与管理，进行的操作如下：
# 1. 更新git子工程libopencm3。
# 2. 调用Tools/check_submodules.sh检查当前libopencm3的版本信息是否正确。
#

$(LIBOPENCM3): checksubmodules
	${MAKE} -C $(LIBOPENCM3) lib

.PHONY: checksubmodules
checksubmodules:
	$(Q) ($(BL_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

# Astyle
# --------------------------------------------------------------------
.PHONY: check_format format

check_format:
	$(call colorecho,'Checking formatting with astyle')
	@$(SRC_DIR)/Tools/check_code_style_all.sh
	@cd $(SRC_DIR) && git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@$(SRC_DIR)/Tools/check_code_style_all.sh --fix
