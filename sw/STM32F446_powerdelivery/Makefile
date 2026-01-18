CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

TARGET = usbpdfw_stm32f4xx
BUILD_DIR = build

# debug build?
DEBUG = 0

# optimization
OPT = -O2

#####################################
# source
######################################
C_SOURCES = \
  Src/hw_init_table.c \
  Src/hw_init_data.c \
  Src/print.c \
  Src/enc_4b5b.c \
  Src/pd_crc.c \
  Src/uart.c \
  Src/goodcrc_frames.c \
  Src/main.c

ASM_SOURCES = \
  Src/startup_stm32f446xx.s

#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -DSTM32F446xx

# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IInc
C_INCLUDES += -I../../
C_INCLUDES += -IST_headers/STM32F4xx_HAL_Driver/Inc
C_INCLUDES += -IST_headers/CMSIS/Include
C_INCLUDES += -IST_headers/CMSIS/Device/ST/STM32F4xx/Include
C_INCLUDES += -IST_headers/

# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m4 $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall #-fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m4 $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall #-fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Src/arm-gcc-link.ld
# libraries
LIBS =
LIBDIR =
LDFLAGS = -mthumb -mcpu=cortex-m4 -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: packed_data $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

packed_data: $(BUILD_DIR)/hw_init_table.o
	cd tools && $(MAKE)

#######################################
# build the application
#######################################

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR) $(MOD_FW_DIR)
	cd tools && $(MAKE) clean

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
