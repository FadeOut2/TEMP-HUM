# Use '>' instead of TAB for recipes (prevents "missing separator")
.RECIPEPREFIX := >

# ===== Project =====
PROJECT := TEMP-HUM
BUILD   := build
TARGET  := $(BUILD)/$(PROJECT)

# ===== Toolchain =====
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
LD      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size
OPENOCD := openocd

# ===== Flags (minimal) =====
CPUFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
DEFS     := -DSTM32F767xx
CSTD     := -std=c17
OPT      := -O2
INCLUDES := -ICore/Inc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32F7xx/Include

# ===== Linker script =====
LDSCRIPT := ./STM32F767XX_FLASH.ld

# ===== Sources =====
SRCS := \
  Core/Src/main.c \
  Core/Src/dht_data.c \
  Core/Src/system_stm32f7xx.c \
  startup_stm32f767xx.s

# ===== Objects =====
OBJS := $(addprefix $(BUILD)/,$(SRCS:=.o))

# ===== Default =====
.PHONY: all
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin size

# ===== Compile =====
$(BUILD)/%.c.o: %.c
> mkdir -p $(dir $@)
> $(CC) $(CPUFLAGS) $(OPT) $(CSTD) $(DEFS) $(INCLUDES) -g3 -c $< -o $@

$(BUILD)/%.s.o: %.s
> mkdir -p $(dir $@)
> $(AS) $(CPUFLAGS) $(DEFS) $(INCLUDES) -g3 -x assembler-with-cpp -c $< -o $@

$(BUILD)/%.S.o: %.S
> mkdir -p $(dir $@)
> $(AS) $(CPUFLAGS) $(DEFS) $(INCLUDES) -g3 -x assembler-with-cpp -c $< -o $@

# ===== Link (no toolchain CRT; we use our startup) =====
$(TARGET).elf: $(OBJS)
> $(LD) $(CPUFLAGS) -nostartfiles $(OBJS) -T $(LDSCRIPT) --specs=nosys.specs -o $@

# ===== Artifacts =====
$(TARGET).hex: $(TARGET).elf
> $(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
> $(OBJCOPY) -O binary $< $@

# ===== Utils =====
.PHONY: size
size: $(TARGET).elf
> $(SIZE) --format=berkeley $<

.PHONY: flash
flash: $(TARGET).elf
> $(OPENOCD) -f interface/stlink.cfg -f target/stm32f7x.cfg -c "init" -c "halt" -c "program $< verify reset exit"

.PHONY: clean
clean:
> rm -rf $(BUILD)
