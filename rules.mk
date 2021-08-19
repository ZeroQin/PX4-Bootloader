#
# Common rules for makefiles for the PX4 bootloaders
#

BUILD_DIR	 = $(BUILD_DIR_ROOT)/$(TARGET_FILE_NAME)#./build/px4fmuv2_bl

COBJS		:= $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(SRCS)))#./build/.../%.o
AOBJS		:= $(addprefix $(BUILD_DIR)/, $(patsubst %.S,%.o,$(ASRCS)))#./build/.../%.S
SUBDIRS		:= $(sort $(addprefix $(BUILD_DIR)/, $(dir $(ASRCS))) $(addprefix $(BUILD_DIR)/, $(dir $(SRCS))) )#所有目标文件子目录(.o .s)
OBJS		:= $(COBJS) $(AOBJS)#所有目标文件
DEPS		:= $(COBJS:.o=.d)

ELF		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).elf#./build/target/target.elf
HEX		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).hex#./build/target/target.hex
BINARY		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).bin#./build/target/target.bin

FLAGS		+= -Xlinker -Map=$(BUILD_DIR)/${TARGET_FILE_NAME}.map

all:	debug $(BUILD_DIR) $(ELF) $(BINARY) $(HEX)

debug:
	@echo ARCH=$(ARCH)xx
	@echo ARCH_SRCS=$(ARCH_SRCS)
	@echo SRCS=$(SRCS)
	@echo COBJS=$(COBJS)
	@echo ASRCS=$(ASRCS)
	@echo AOBJS=$(AOBJS)
	@echo SUBDIRS=$(SUBDIRS)


# Compile and generate dependency files
$(BUILD_DIR)/%.o:	%.c
	@echo Generating object $@
	$(CC) -c -MMD $(FLAGS) -o $@ $<

$(BUILD_DIR)/%.o:	%.S
	@echo Generating object $@
	$(CC) -c -MMD $(FLAGS) -o $@ $*.S

# Make the build directory
$(BUILD_DIR):
	$(shell mkdir -p $(BUILD_DIR) >/dev/null)
	$(shell mkdir -p $(SUBDIRS) >/dev/null)
#$(SUBDIRS):
#	@echo mkdir $(BUILD_DIR) $(SUBDIRS)
#	$(shell mkdir -p $(BUILD_DIR)) >/dev/null)
#	mkdir -p $(BUILD_DIR) $(SUBDIRS)

$(ELF):		$(OBJS) $(MAKEFILE_LIST)
	$(CC) -o $@ $(OBJS) $(FLAGS)

$(BINARY):	$(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BINARY)

$(HEX):	$(ELF)
	$(OBJCOPY) -Oihex $(ELF) $(HEX)
	
%.d: %.c
	set -e; rm -f $@; \
	$(CC) -MM $(FLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

# Dependencies for .o files
-include $(DEPS)#包括这些文件，如果没找到则忽略
