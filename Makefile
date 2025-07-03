# SPDX-License-Identifier: MIT
# Copyright (c) 2024 The Pybricks Authors

# Builds the TI StarterWare to run on the EV3.

THIS_MAKEFILE := $(lastword $(MAKEFILE_LIST))
TOP := .$(patsubst %Makefile,%,$(THIS_MAKEFILE))
PBTOP := $(TOP)/../pybricks-micropython
BUILD ?= $(TOP)/bin

CROSS_COMPILE ?= arm-none-eabi-
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CPP = $(CC) -E
CXX = $(CROSS_COMPILE)g++
GDB = $(CROSS_COMPILE)gdb
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
AR = $(CROSS_COMPILE)ar
ECHO = @echo

INC += -I.
INC += -I$(TOP)
INC += -I$(TOP)/include
INC += -I$(TOP)/include/hw
INC += -I$(TOP)/include/armv5
INC += -I$(TOP)/include/armv5/am1808
INC += -I$(BUILD)

GIT = git
ZIP = zip
PYBRICKSDEV = pybricksdev
CFLAGS_MCU = -mcpu=arm926ej-s -Dgcc -Dam1808 # -c -g -fdata-sections -ffunction-sections -Wall -Dgcc -Dam1808 -O0

CFLAGS_WARN = -Wall -Werror -Wextra -Wno-unused-parameter -Wno-maybe-uninitialized -Wno-error=sign-compare -Wno-error=implicit-fallthrough= -Wno-error=empty-body
CFLAGS = $(INC) -std=c99 -nostdlib -fshort-enums $(CFLAGS_MCU) $(CFLAGS_WARN) $(COPT) $(CFLAGS_EXTRA)
CLAGS += -Dasm(t)=__asm__ __volatile__(t)

# linker scripts
LD_FILES = $(TOP)/link.ld
LDFLAGS = $(addprefix -T,$(LD_FILES)) -Wl,-Map=$@.map -Wl,--cref -Wl,--gc-sections

# avoid doubles
CFLAGS += -fsingle-precision-constant -Wdouble-promotion

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -Og -ggdb
else ifeq ($(DEBUG), 2)
CFLAGS += -Os -DNDEBUG -flto
else
CFLAGS += -Os -DNDEBUG -flto
CFLAGS += -fdata-sections -ffunction-sections
endif

LIBS = "$(shell $(CC) $(CFLAGS) -print-libgcc-file-name)"

# Skipping uart_irda_cir.c, gpio_v2.c, and hsi2c.c usbphyGS70.c, which
# partially overlap with uart.c, gpio.c, and i2c.c, usbphyGS70.c
TI_AM1808_SRC_C = $(addprefix ,\
	drivers/cppi41dma.c \
	drivers/cpsw.c \
	drivers/dcan.c \
	drivers/dmtimer.c \
	drivers/ecap.c \
	drivers/edma.c \
	drivers/ehrpwm.c \
	drivers/elm.c \
	drivers/emac.c \
	drivers/emifa.c \
	drivers/gpio.c \
	drivers/gpmc.c \
	drivers/hs_mmcsd.c \
	drivers/i2c.c \
	drivers/lan8710a.c \
	drivers/lidd.c \
	drivers/mailbox.c \
	drivers/mcasp.c \
	drivers/mcspi.c \
	drivers/mdio.c \
	drivers/phy.c \
	drivers/pruss.c \
	drivers/psc.c \
	drivers/raster.c \
	drivers/rtc.c \
	drivers/spi.c \
	drivers/timer.c \
	drivers/tsc_adc.c \
	drivers/uart.c \
	drivers/usb.c \
	drivers/usbphyGS60.c \
	drivers/vpif.c \
	drivers/watchdog.c \
	system_config/armv5/gcc/cp15.c \
	system_config/armv5/gcc/cpu.c \
	system_config/armv5/am1808/interrupt.c \
	system_config/armv5/am1808/startup.c \
	utils/uartStdio.c \
	utils/string0.c \
	)

EV3_SRC_S = $(addprefix system_config/,\
	armv5/gcc/init.S \
	armv5/am1808/gcc/exceptionhandler.S \
	)

EVM_PLATFORM_SRC_C = $(addprefix ,\
	platform/evmAM1808/syscfg.c \
	platform/evmAM1808/uart.c \
	platform/evmAM1808/uartConsole.c \
	platform/evmAM1808/edma.c \
	platform/evmAM1808/spi.c \
	)

TI_AM1808_PRJ_C = $(addprefix ,\
	examples/evmAM1808/spi/spiflash.c \
    )

OBJ = $(addprefix $(BUILD)/, $(TI_AM1808_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(TI_AM1808_PRJ_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(EVM_PLATFORM_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(EV3_SRC_S:.S=.o))
$(addprefix $(BUILD)/, $(EV3_SRC_S:.S=.o)): CFLAGS += -D__ASSEMBLY__

# Main firmware build targets
TARGETS := $(BUILD)/firmware.zip

all: $(TARGETS)

clean:
	rm -rf $(BUILD)

FW_SECTIONS :=

$(BUILD)/firmware.elf: $(LD_FILES) $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LIBS)
	$(Q)$(SIZE) -A $@

$(BUILD)/%.o: %.c
	$(ECHO) "CC $<"
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	$(ECHO) "CC $<"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) -c -o $@ $<

# firmware blob without checksum
$(BUILD)/firmware-base.bin: $(BUILD)/firmware.elf
	$(ECHO) "BIN creating firmware base file"
	$(Q)$(OBJCOPY) -O binary $(FW_SECTIONS) $^ $@
	$(ECHO) "`wc -c < $@` bytes"
	$(Q)dd if=/dev/zero bs=1 count=1 seek=$$((128 * 1024 - 1)) of=$@ conv=notrunc

FW_VERSION = 1.0.0

# firmware.zip file
ZIP_FILES := \
	$(BUILD)/firmware-base.bin \
	$(TOP)/firmware.metadata.json \
	ReadMe_OSS.txt \

$(BUILD)/firmware.zip: $(ZIP_FILES)
	$(ECHO) "ZIP creating firmware package"
	$(Q)$(ZIP) -j $@ $^

deploy: $(BUILD)/firmware.zip
	$(Q)$(PYBRICKSDEV) flash $<
