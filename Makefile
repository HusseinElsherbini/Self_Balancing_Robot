CC=arm-none-eabi-gcc
OBJDMP=arm-none-eabi-objdump
MACH=cortex-m4
OUTPUT_DIR := out
PROJ_NAME := sbr
CFLAGS= -c -mcpu=$(MACH) -mfloat-abi=soft -mthumb -I./inc -std=gnu11 -Wall -O0 -g3 -DDEBUG -w
LDFLAGS= -mcpu=$(MACH) -mfloat-abi=soft -mthumb --specs=nano.specs -T stm32_ls.ld -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map
LDFLAGS_SH= -mcpu=$(MACH) -mfloat-abi=soft -mthumb --specs=rdimon.specs -T stm32_ls.ld -Wl,-Map=$(OUTPUT_DIR)/$(PROJ_NAME).map 

dir_guard=@mkdir -p $(@D)

.SECONDEXPANSION:
PERCENT = %

C_SRCS += \
./src/led.c \
./src/syscalls.c \
./src/sw_init.c \
./src/cortex.c \
./src/platform.c \
./src/task_scheduler.c \
./src/imu.c \
./src/i2c.c \
./src/sys_interrupts.c \
./src/spi.c \
./src/rotary_encoder.c \
./src/pwm.c  \
./src/timer.c \
./src/gpio.c \
./src/kalman_filter.c \
./startup/stm32_startup.c \
./src/aux_board_communication.c \
./src/dma.c  \
./main.c 

OBJDIR :=$(OUTPUT_DIR)/Obj
OUT_BIN :=$(OUTPUT_DIR)/bin

OBJS := $(C_SRCS:.c=.o)
OBJS := $(notdir $(OBJS))
OBJS := $(foreach OBJECT,$(OBJS),$(addprefix $(OBJDIR)/,$(OBJECT)))

all: final 
	
semi:main.o led.o stm32_startup.o final_sh.elf 

$(OBJS): %.o : $$(filter $$(PERCENT)/$$(notdir %).c, $(C_SRCS))
	$(dir_guard)
	$(CC) $(CFLAGS) $< -o  $@
	
$(OUT_BIN)/$(PROJ_NAME).elf: $(OBJS)
	$(dir_guard)
	$(CC) $(LDFLAGS) -o $@ $^ -lm
	$(OBJDMP) -D $(OUT_BIN)/$(PROJ_NAME).elf > out/$(PROJ_NAME).txt


final: $(OBJS) $(OUT_BIN)/$(PROJ_NAME).elf 

clean:
	rm -rf $(OUTPUT_DIR)

load:
	openocd -f board/stm32f4.cfg
