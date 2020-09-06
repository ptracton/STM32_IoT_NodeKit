#
# CUSTOMIZE ENVIRONMENT VARIABLES
#
TARGET = Wifi
STM32L4_CUBEIDE_ROOT_DIR = /home/ptracton/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0

##
## Prefix of the GCC ARM Cross Compiler
##
PREFIX	= arm-none-eabi

##
## Tools we use for building
##
CC	    = $(PREFIX)-gcc
LD	    = $(PREFIX)-gcc
ASM     = $(PREFIX)-as
OBJCOPY	= $(PREFIX)-objcopy
OBJDUMP	= $(PREFIX)-objdump
AR     	= $(PREFIX)-ar
GDB	    = $(PREFIX)-gdb
FLASH	= $(shell which st-flash)
MKDIR   = mkdir
RM      = rm

##
## OpenOCD setup
## TODO: FIX THIS FOR OUR BOARD
##
OOCD		     = openocd
OOCD_INTERFACE	 = stlink-v2
OOCD_BOARD	     = stm32f3discovery
OCDFLAGS         = -f board/stm32f3discovery.cfg
OCD_DIR          = /usr/share/openocd/scripts

##
## Project Directories
##
SRC_DIR = WiFi_HTTP_Server/Src
OBJ_DIR = objects
INC_DIR = WiFi_HTTP_Server/Inc
EXEC_DIR = executable
DEP_DIR = dep
LD_DIR = WiFi_HTTP_Server/STM32CubeIDE
LDSCRIPT =$(LD_DIR)/STM32L4S5VITX_FLASH.ld
ASM_SRC_DIR = WiFi_HTTP_Server/STM32CubeIDE/Application/Startup

##
## The files we are creating
##
ELF_FILE  = $(EXEC_DIR)/$(TARGET).elf
LIST_FILE = $(EXEC_DIR)/$(TARGET).lst
HEX_FILE  = $(EXEC_DIR)/$(TARGET).hex
SREC_FILE = $(EXEC_DIR)/$(TARGET).srec
BIN_FILE  = $(EXEC_DIR)/$(TARGET).bin

##
## Start up file
##
ASM_SOURCES      += $(wildcard $(ASM_SRC_DIR)/*.s) #WiFi_HTTP_Server/STM32CubeIDE/Application/Startup/startup_stm32l4s5vitx.s
ASM_OBJECTS      += $(patsubst $(ASM_SRC_DIR)/%.s,$(OBJ_DIR)/%.o,$(ASM_SOURCES)) 
ELF_REQUIREMENTS += $(ASM_OBJECTS)
VPATH            += GCC
ASMFLAGS         += -mcpu=cortex-m4 -mthumb -warn --fatal-warnings

##
## Project sources converted to objects
##
SOURCES          += $(wildcard $(SRC_DIR)/*.c)
OBJECTS          += $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))
DEPS              = $(addprefix $(DEP_DIR),$(SOURCES:.c=.d))
CINCLUDES        += -I$(INC_DIR)
VPATH            += $(SRC_DIR)
ELF_REQUIREMENTS += $(OBJECTS) 

##
## Project sources converted to objects
##
USER_SRC_DIR      = WiFi_HTTP_Server/STM32CubeIDE/Application/User/
USER_SOURCES     += $(wildcard $(USER_SRC_DIR)/*.c)
USER_OBJECTS     += $(patsubst $(USER_SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(USER_SOURCES))
DEPS              = $(addprefix $(DEP_DIR),$(USER_SOURCES:.c=.d))
VPATH            += $(USER_SRC_DIR)
ELF_REQUIREMENTS += $(USER_OBJECTS) 

##
## Project sources converted to objects
##
COMMON_ROOT_DIR  =  Common
COMMON_SRC_DIR    = $(COMMON_ROOT_DIR)/Src
COMMON_SOURCES   += $(wildcard $(COMMON_SRC_DIR)/*.c)
COMMON_OBJECTS   += $(patsubst $(COMMON_SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(COMMON_SOURCES))
DEPS              = $(addprefix $(DEP_DIR),$(COMMON_SOURCES:.c=.d))
CINCLUDES        += -I$(COMMON_ROOT_DIR)/Inc
VPATH            += $(COMMON_SRC_DIR)
ELF_REQUIREMENTS += $(COMMON_OBJECTS) 

##
## HAL Drivers
##
HAL_ROOT_DIR      = $(STM32L4_CUBEIDE_ROOT_DIR)/Drivers/STM32L4xx_HAL_Driver
HAL_SRC_DIR       = $(HAL_ROOT_DIR)/Src
HAL_INC_DIR       = $(HAL_ROOT_DIR)/Inc
HAL_SOURCES      += $(wildcard $(HAL_SRC_DIR)/*.c)
HAL_SOURCES_FIX  := $(filter-out $(HAL_SRC_DIR)/stm32f3xx_hal_msp_template.c,$(HAL_SOURCES))
HAL_OBJECTS      += $(patsubst $(HAL_SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(HAL_SOURCES_FIX))
VPATH            += $(HAL_SRC_DIR)
CINCLUDES        += -I$(HAL_INC_DIR)
ELF_REQUIREMENTS += $(HAL_OBJECTS)

##
## CMSIS Platform Drivers
##
CMSIS_ROOT_DIR    =   $(STM32L4_CUBEIDE_ROOT_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx
CMSIS_INC_DIR     =  $(CMSIS_ROOT_DIR)/Include
CINCLUDES        += -I$(CMSIS_INC_DIR)

##
## CMSIS Core Drivers
##
CMSIS_CORE_ROOT_DIR  = $(STM32L4_CUBEIDE_ROOT_DIR)/Drivers/CMSIS/Core
CMSIS_CORE_INC_DIR   = $(CMSIS_CORE_ROOT_DIR)/Include
CINCLUDES        += -I$(CMSIS_CORE_INC_DIR)

##
## Board Support Package (BSP)
##
BSP_ROOT_DIR      = $(STM32L4_CUBEIDE_ROOT_DIR)/Drivers/BSP/B-L4S5I-IOT01
BSP_SRC_DIR       = $(BSP_ROOT_DIR)
BSP_INC_DIR       = $(BSP_ROOT_DIR)
BSP_SOURCES      += $(wildcard $(BSP_SRC_DIR)/*.c)
BSP_SOURCES_FIX   = $(filter-out $(BSP_SRC_DIR)/stm32l4s5i_iot01_qspi.c,$(BSP_SOURCES))
BSP_SOURCES_FIX2 += $(filter-out $(BSP_SRC_DIR)/stm32l4s5i_iot01_nfctag.c,$(BSP_SOURCES_FIX))
BSP_OBJECTS      += $(patsubst $(BSP_SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(BSP_SOURCES_FIX2))
VPATH            += $(BSP_SRC_DIR)
CINCLUDES        += -I$(BSP_INC_DIR)
ELF_REQUIREMENTS += $(BSP_OBJECTS)

##
## BSP HTS221
##
BSP_HTS221_ROOT_DIR      = $(STM32L4_CUBEIDE_ROOT_DIR)/Drivers/BSP/Components/hts221
BSP_HTS221_SRC_DIR       = $(BSP_HTS221_ROOT_DIR)
BSP_HTS221_INC_DIR       = $(BSP_HTS221_ROOT_DIR)
BSP_HTS221_SOURCES      += $(wildcard $(BSP_HTS221_SRC_DIR)/*.c)
BSP_HTS221_OBJECTS      += $(patsubst $(BSP_HTS221_SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(BSP_HTS221_SOURCES))
VPATH                   += $(BSP_HTS221_SRC_DIR)
CINCLUDES               += -I$(BSP_HTS221_INC_DIR)
ELF_REQUIREMENTS        += $(BSP_HTS221_OBJECTS)


##
## Options passed to the C compiler
##
#CFLAGS  += -DSTM32L475
#CFLAGS  += -DSTM32L475xx
CFLAGS  += -DSTM32L4S5xx
CFLAGS  += -DUSE_HAL_DRIVER
CFLAGS  += -DUSE_FULL_ASSERT
CFLAGS	+= -Os -g  $(CINCLUDES)\
	   -fno-common -mcpu=cortex-m4 -mthumb \
	   -mfloat-abi=hard -mfpu=fpv4-sp-d16 -MD

##
## This used to work well before CUBE, but CUBE is a mess
## with violations of this.  You can turn it on all or in
## part but the build is ugly then
##
#CFLAGS  += -Wall -Wextra -Wpedantic -Werror


##
## Flags for linking image together
##
LDFLAGS		+= --static -lc  \
		   -T$(LDSCRIPT) -Wl,--gc-sections \
		   -mthumb -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

##
## The directories to make for this build.
##
MAKE_DIRS += $(OBJ_DIR) $(EXEC_DIR) $(DEP_DIR)

##
## The list of steps to build the image
##
BUILD_LIST += dirs $(ELF_FILE) $(LIST_FILE) $(HEX_FILE) $(SREC_FILE) $(BIN_FILE)

.PHONY: all dirs clean

all: $(BUILD_LIST) 

-include $(DEPS)

##
## Create the directories needed for this build
##
dirs:
	@echo "Making directories $(MAKE_DIRS)"
	@$(MKDIR) -p $(MAKE_DIRS)

##
## Link together the executable
##
$(ELF_FILE): $(ELF_REQUIREMENTS)
	@echo "Linking $(ELF_FILE)"
	@$(LD) -o $(ELF_FILE) $(ELF_REQUIREMENTS) $(LDFLAGS)


$(HEX_FILE) :$(ELF_FILE)
	@echo "Creating $(HEX_FILE)"
	@$(OBJCOPY) -Oihex $(ELF_FILE) $(HEX_FILE)

$(SREC_FILE):$(ELF_FILE)
	@echo "Creating $(SREC_FILE)"
	@$(OBJCOPY) -Osrec $(ELF_FILE)  $(SREC_FILE)

$(BIN_FILE):$(ELF_FILE)
	@echo "Creating $(BIN_FILE)"
	@$(OBJCOPY) -Obinary $(ELF_FILE) $(BIN_FILE)

$(LIST_FILE):$(ELF_FILE)
	@echo "Creating $(LIST_FILE)"
	@$(OBJDUMP) -S $(ELF_FILE) >  $(LIST_FILE)

##
## Turn out C code into objects in our $(OBJ_DIR)
##
$(OBJ_DIR)/%.o:%.c
	@echo "Compiling  $<"
	@$(CC) $(CFLAGS) -o $@ -c $< -MMD -MF dep/$(*F).d

##
## Turn out ASM code into objects in our $(OBJ_DIR)
##
$(OBJ_DIR)/%.o:$(ASM_SRC_DIR)/%.s
	@echo "Assembling $<"
	@$(ASM) $(ASMFLAGS)  -o $@ -c $< 

##
## Start up OPENOCD for our target board
##
openocd:
	$(OCD) -s $(OCD_DIR) $(OCDFLAGS)

##
## Program target via stlink took
##
program: dirs $(BIN_FILE)
	$(FLASH) write $(BIN_FILE) 0x8000000

##
## Clean up files and directories
##
clean:
	find . -name "*~" | xargs $(RM) -f
	$(RM) -rf $(OBJ_DIR)
	$(RM) -rf $(EXEC_DIR)
	$(RM) -rf $(DEP_DIR)
