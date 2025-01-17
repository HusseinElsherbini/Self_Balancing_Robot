# Compiler and tool definitions
CC=arm-none-eabi-gcc
OBJDMP=arm-none-eabi-objdump
MACH=cortex-m4
OUTPUT_DIR := out
PROJ_NAME := sbr

# Compiler flags for C files
# -mfloat-abi=hard and -mfpu=fpv4-sp-d16 are specific to STM32F401's FPU
CFLAGS= -c -mcpu=$(MACH) -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16 \
        $(MACROS) -I./inc -I./inc/freeRtos -I./inc/SEGGER \
        -std=gnu11 -Wall -O0 -g3 -DDEBUG -w

# Assembler flags - similar to C flags but for .s files
ASFLAGS= -c -mcpu=$(MACH) -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16 -x assembler-with-cpp -I./inc -I./inc/freeRtos -I./inc/SEGGER

# Linker flags - using nano.specs for smaller code size
LDFLAGS= -mcpu=$(MACH) -mfloat-abi=hard -mthumb --specs=nano.specs \
         -T stm32_ls.ld -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map

# Alternative linker flags for semihosting (debug output via debugger)
LDFLAGS_SH= -mcpu=$(MACH) -mfloat-abi=hard -mthumb --specs=rdimon.specs \
            -T stm32_ls.ld -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map

# Debug configuration flags
PRINT_IMU_DATA=0
USE_TASK_ANALYSIS=0
PRINT_AUX_BOARD_COMMUNICATION=0
DEBUG_LEVEL=3

# Macro definitions that will be passed to compiler
MACROS = \
  -DPRINT_IMU_DATA=$(PRINT_IMU_DATA) \
  -DUSE_TASK_ANALYSIS=$(USE_TASK_ANALYSIS) \
  -DDEBUG_LEVEL=$(DEBUG_LEVEL) \

# Directory creation helper
dir_guard=@mkdir -p $(@D)

# Pattern matching helper
.SECONDEXPANSION:
PERCENT = %

# Assembly source files (separated from C sources)
ASM_SRCS = \
./src/SEGGER/SEGGER_RTT_ASM_ARMv7M.s

# C source files
C_SRCS = \
./src/led.c \
./src/syscalls.c \
./src/sw_init.c \
./src/cortex.c \
./src/motor.c \
./src/platform.c \
./src/imu.c \
./src/i2c.c \
./src/sys_interrupts.c \
./src/spi.c \
./src/rotary_encoder.c \
./src/freeRtos/tasks.c \
./src/freeRtos/queue.c \
./src/freeRtos/port.c \
./src/freeRtos/list.c \
./src/freeRtos/heap_4.c \
./src/freeRtos/timers.c \
./src/SEGGER/SEGGER_RTT.c \
./src/SEGGER/SEGGER_RTT_printf.c \
./src/SEGGER/SEGGER_RTT_Syscalls_GCC.c \
./src/SEGGER/SEGGER_SYSVIEW.c \
./src/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.c \
./src/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c \
./src/smart_port.c \
./src/pwm.c \
./src/timer.c \
./src/gpio.c \
./src/kalman_filter.c \
./startup/stm32_startup.c \
./src/aux_board_communication.c \
./src/dma.c \
./src/task_analysis.c \
./src/pid.c \
./src/sbus_receiver.c \
./src/hall_effect.c \
./src/logger.c \
./src/adc.c \
./src/balancing.c \
./main.c

# Output directories
OBJDIR := $(OUTPUT_DIR)/Obj
OUT_BIN := $(OUTPUT_DIR)/bin

# Generate lists of object files
C_OBJS := $(C_SRCS:.c=.o)
C_OBJS := $(notdir $(C_OBJS))
C_OBJS := $(foreach OBJECT,$(C_OBJS),$(addprefix $(OBJDIR)/,$(OBJECT)))

ASM_OBJS := $(ASM_SRCS:.s=.o)
ASM_OBJS := $(notdir $(ASM_OBJS))
ASM_OBJS := $(foreach OBJECT,$(ASM_OBJS),$(addprefix $(OBJDIR)/,$(OBJECT)))

# Combine all object files
ALL_OBJS = $(C_OBJS) $(ASM_OBJS)

# Build targets
all: final

# Rule for assembly files
$(ASM_OBJS): %.o : $$(filter $$(PERCENT)/$$(notdir %).s, $(ASM_SRCS))
	$(dir_guard)
	$(CC) $(ASFLAGS) $< -o $@

# Rule for C files
$(C_OBJS): %.o : $$(filter $$(PERCENT)/$$(notdir %).c, $(C_SRCS))
	$(dir_guard)
	$(CC) $(CFLAGS) $< -o $@

# Link everything together
$(OUT_BIN)/$(PROJ_NAME).elf: $(ALL_OBJS)
	$(dir_guard)
	$(CC) $(LDFLAGS) -o $@ $^ -lm
	$(OBJDMP) -D $(OUT_BIN)/$(PROJ_NAME).elf > out/$(PROJ_NAME).txt

# Main target
final: $(ALL_OBJS) $(OUT_BIN)/$(PROJ_NAME).elf

# Clean target
clean:
	rm -rf $(OUTPUT_DIR)

# Load target for OpenOCD
load:
	openocd -f board/stm32f4.cfg

# Semi-hosting target (for debugging)
semi: main.o led.o stm32_startup.o
	$(CC) $(LDFLAGS_SH) -o $(OUTPUT_DIR)/final_sh.elf $^ -lm