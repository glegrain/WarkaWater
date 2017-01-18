###############################################################################
## Toolchain
CC         = arm-none-eabi-gcc
AR         = arm-none-eabi-ar
LD         = arm-none-eabi-gcc
OBJCOPY    = arm-none-eabi-objcopy
OBJDUMP    = arm-none-eabi-objdump
SIZE       = arm-none-eabi-size
GDB        = arm-none-eabi-gdb

OPENOCD_DIR = /Applications/GNU\ ARM\ Eclipse/OpenOCD/0.10.0-201610281609-dev

## STM32Cube software path
CUBE_DIR   = ../STM32Cube_FW_L0_V1.7.0

################################################################################

## Linker script
LDFILE   := STM32L053R8Tx_FLASH.ld

## Preprocessor Symbols
DEFS     := -DUSE_HAL_DRIVER
DEFS     += -DSTM32L053xx
DEFS     += -DUSE_STM32L0XX_NUCLEO


OBJECTS  := main.o
OBJECTS  += startup_stm32l053xx.o
OBJECTS  += system_stm32l0xx.o
OBJECTS  += stm32l0xx_it.o
OBJECTS  += stm32l0xx_hal_msp.o
OBJECTS  += dht.o
OBJECTS  += dewpoint.o
OBJECTS  += ms5540c.o

OBJECTS  += stm32l0xx_hal.o
# OBJECTS  += stm32l0xx_hal_adc.o
# OBJECTS  += stm32l0xx_hal_adc_ex.o
# OBJECTS  += stm32l0xx_hal_comp.o
# OBJECTS  += stm32l0xx_hal_comp_ex.o
OBJECTS  += stm32l0xx_hal_cortex.o
# OBJECTS  += stm32l0xx_hal_crc.o
# OBJECTS  += stm32l0xx_hal_crc_ex.o
# OBJECTS  += stm32l0xx_hal_cryp.o
# OBJECTS  += stm32l0xx_hal_cryp_ex.o
# OBJECTS  += stm32l0xx_hal_dac.o
# OBJECTS  += stm32l0xx_hal_dac_ex.o
OBJECTS  += stm32l0xx_hal_dma.o
# OBJECTS  += stm32l0xx_hal_firewall.o
# OBJECTS  += stm32l0xx_hal_flash.o
# OBJECTS  += stm32l0xx_hal_flash_ex.o
# OBJECTS  += stm32l0xx_hal_flash_ramfunc.o
OBJECTS  += stm32l0xx_hal_gpio.o
# OBJECTS  += stm32l0xx_hal_i2c.o
# OBJECTS  += stm32l0xx_hal_i2c_ex.o
# OBJECTS  += stm32l0xx_hal_irda.o
# OBJECTS  += stm32l0xx_hal_iwdg.o
# OBJECTS  += stm32l0xx_hal_lcd.o
# OBJECTS  += stm32l0xx_hal_lptim.o
# OBJECTS  += stm32l0xx_hal_lptim_ex.o
# OBJECTS  += stm32l0xx_hal_pcd.o
# OBJECTS  += stm32l0xx_hal_pcd_ex.o
# OBJECTS  += stm32l0xx_hal_pwr.o
# OBJECTS  += stm32l0xx_hal_pwr_ex.o
# OBJECTS  += stm32l0xx_hal_qspi.o
OBJECTS  += stm32l0xx_hal_rcc.o
# OBJECTS  += stm32l0xx_hal_rcc_ex.o
# OBJECTS  += stm32l0xx_hal_rng.o
# OBJECTS  += stm32l0xx_hal_rtc.o
# OBJECTS  += stm32l0xx_hal_rtc_ex.o
# OBJECTS  += stm32l0xx_hal_smartcard.o
# OBJECTS  += stm32l0xx_hal_smartcard_ex.o
# OBJECTS  += stm32l0xx_hal_smbus.o
OBJECTS  += stm32l0xx_hal_spi.o
OBJECTS  += stm32l0xx_hal_tim.o
# OBJECTS  += stm32l0xx_hal_tim_ex.o
# OBJECTS  += stm32l0xx_hal_tsc.o
OBJECTS  += stm32l0xx_hal_uart.o
# OBJECTS  += stm32l0xx_hal_uart_ex.o
# OBJECTS  += stm32l0xx_hal_usart.o
# OBJECTS  += stm32l0xx_hal_wwdg.o
# OBJECTS  += stm32l0xx_ll_adc.o
# OBJECTS  += stm32l0xx_ll_comp.o
# OBJECTS  += stm32l0xx_ll_crc.o
# OBJECTS  += stm32l0xx_ll_crs.o
# OBJECTS  += stm32l0xx_ll_dac.o
# OBJECTS  += stm32l0xx_ll_dma.o
# OBJECTS  += stm32l0xx_ll_exti.o
# OBJECTS  += stm32l0xx_ll_gpio.o
# OBJECTS  += stm32l0xx_ll_i2c.o
# OBJECTS  += stm32l0xx_ll_lptim.o
# OBJECTS  += stm32l0xx_ll_lpuart.o
# OBJECTS  += stm32l0xx_ll_pwr.o
# OBJECTS  += stm32l0xx_ll_rcc.o
# OBJECTS  += stm32l0xx_ll_rng.o
# OBJECTS  += stm32l0xx_ll_rtc.o
# OBJECTS  += stm32l0xx_ll_spi.o
# OBJECTS  += stm32l0xx_ll_tim.o
# OBJECTS  += stm32l0xx_ll_usart.o
# OBJECTS  += stm32l0xx_ll_utils.o

## Search Paths for All Prerequisites
VPATH    :=  ./Src
VPATH    += :$(CUBE_DIR)/Drivers/STM32L0xx_HAL_Driver/Src
VPATH    += :$(CUBE_DIR)/Drivers/BSP/STM32L0xx_Nucleo
VPATH    += :$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc/

# Include Path for All Headers
INCLUDES := -I.
INCLUDES += -I./Inc
INCLUDES += -I$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L0xx/Include
INCLUDES += -I$(CUBE_DIR)/Drivers/CMSIS/Include
INCLUDES += -I$(CUBE_DIR)/Drivers/STM32L0xx_HAL_Driver/Inc
INCLUDES += -I$(CUBE_DIR)/Drivers/BSP/STM32L0xx_Nucleo
INCLUDES += -I$(CUBE_DIR)/Drivers/BSP/Adafruit_Shield
INCLUDES += -I$(CUBE_DIR)/Drivers/BSP/Components/Common
INCLUDES += -I$(CUBE_DIR)/Drivers/BSP/Components/st7735

################################################################################

## Compiler flags
CFLAGS    := -Wall -g -std=c99 -O0
CFLAGS    += -mlittle-endian -mcpu=cortex-m0plus -march=armv6s-m -mthumb
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCLUDES) $(DEFS)
# line-wrapping
CFLAGS    += -fmessage-length=80

## Linker flags
# LDFLAGS   := -Wl,--gc-sections -Wl,-Map=$(TARGET).map $(LIBS) -T$(LDFILE)
LDFLAGS   := -Wl,--gc-sections,-Map=out.map,-cref -T $(LDFILE)
# ASFLAGS = $(CFLAGS) -x assembler-with-cpp
# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

# Objdump flags
OBJDUMPFLAGS = -St

################################################################################

.PHONY: all clean debug run

all: out.elf

out.elf: $(OBJECTS)
	#$(CC) $(CFLAGS) $(LDFLAGS) startup_$(MCU_LC).s $^ -o $@
	@echo "[LD]      out.elf"
	$(LD) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJECTS)
	# $(OBJDUMP) $(OBJDUMPFLAGS) out.elf > out.list
	$(SIZE) out.elf

# main.out: $(LIBS) $(OBJS)
# 	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.o: %.c
	@echo "[CC]      $(notdir $<)"
	$(CC) -c $(CFLAGS) $< -o $@
	# $(CC) -MM $(CFLAGS) $< > $*.d

%.o: %.s
	@echo "[CC]      $(notdir $<)"
	$(CC) -c $(CFLAGS) $< -o $@

debug: out.elf
#	arm-none-eabi-gdb -tui --eval-command="target extended-remote :4242" out.elf
	arm-none-eabi-gdb -tui --eval-command="target extended-remote :3333" out.elf

run: out.elf
	$(OPENOCD_DIR)/bin/openocd \
	-f $(OPENOCD_DIR)/scripts/board/stm32l0discovery.cfg \
	-c "init" \
	-c "arm semihosting enable" \
	-c "reset halt" \
	-c "flash write_image erase $(CURDIR)/out.elf" \
	-c "reset run"
clean:
	rm -f *.o
	rm -f *.d
	rm -f *.elf
	rm -f *.map
	rm -f *.list

