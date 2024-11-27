#******************************************************************************
#
# makefile - Rules for building the GPIO example
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
# ALL RIGHTS RESERVED
#
#
#******************************************************************************

#
# Locating the root directory 
#
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am1808
EVM=evmAM1808

#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ${ROOT}/build/armv5/cgt/makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD}

#
# Application Location
#
APP=${ROOT}examples/${EVM}/cache_mmu/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/cache_mmu

#
# Application source files
#
APP_SRC=$(APP)/*.c

#
# Required library files
#
APP_LIB=-ldrivers.lib  \
        -lutils.lib    \
        -lplatform.lib \
        -lsystem_config.lib

#
# Rules for building the application and library 
#
all: debug release

debug: 
	make TARGET_MODE=debug lib
	make TARGET_MODE=Debug bin

release:
	make TARGET_MODE=release lib
	make TARGET_MODE=Release bin

lib:
	@for i in ${DIRS};                                \
	do                                                \
		if [ -f $${i}/makefile ] ;                    \
		then                                          \
			make $(TARGET_MODE) -C $${i} || exit $$?; \
		fi;                                           \
	done;


bin:
# Below commands generate .out and .bin with cache cleaning mechanism 
# not implemented. Expected result is "a to z" followed by "A to Z" alphabet
# has to be printed on serial console.
# Since cache cleaning is not implemented expected result won't be
# printed on serial console.
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/uartEdma_Cache.out -m $(TARGET_MODE)/uartEdma_Cache.map \
              $(TARGET_MODE)/*.obj -l"libc.a" "uartEdma_Cache.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/uartEdma_Cache.out $(APP_BIN)/$(TARGET_MODE)/uartEdma_Cache.out
# Below commands generate .out and .bin with cache clean mechanism implemented.
# Expected result is "a to z" followed by "A to Z" alphabet has to be
# printed on serial console. Since cache cleaning is implemented expected 
# result is printed on serial console.
	$(CC)  $(CFLAGS) -D CACHE_FLUSH $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/uartEdma_CacheFlush.out -m $(TARGET_MODE)/uartEdma_CacheFlush.map \
              $(TARGET_MODE)/*.obj -l"libc.a" "uartEdma_Cache.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/uartEdma_CacheFlush.out $(APP_BIN)/$(TARGET_MODE)/uartEdma_CacheFlush.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib
