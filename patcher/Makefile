# Derived from libopencm3 Makefiles

OBJS		= main.o data.o synchronization.o transport.o netapi.o init.o
BINARY		= patcher

PREFIX		?= arm-none-eabi
CC		= $(DISTCC) $(PREFIX)-gcc
LD		= $(DISTCC) $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
SIZE		= $(PREFIX)-size
CFLAGS		+= --std=gnu99 -Os -g -Wall -Wextra -DSTM32F1 \
		   -I/opt/arm-cortex-m3/$(PREFIX)/include -I.. -I../common \
		   -fno-common -mcpu=cortex-m3 -mthumb -msoft-float -MD \
		   -DENABLE_CC3000_PATCHING -DF_CPU=24000000 -DRAM_SIZE=8192
LDSCRIPT	= $(BINARY).ld
LDFLAGS		+= -lopencm3_stm32f1 -lc -lnosys -T$(LDSCRIPT) -nostartfiles \
			-Wl,--gc-sections -mthumb -march=armv7 \
			-mfix-cortex-m3-ldrd -msoft-float

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif

.SUFFIXES: .elf .bin .hex .srec .list .images
.SECONDEXPANSION:
.SECONDARY:

all: images

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list
	@echo "*** $* images generated ***"
	$(Q)$(SIZE) $(*).elf

%.bin: %.elf
	@printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@printf "  OBJCOPY $(*).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@printf "  OBJDUMP $(*).list\n"
	$(Q)$(OBJDUMP) -S $(*).elf > $(*).list

%.elf: $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) -o $(*).elf $(OBJS) $(LDFLAGS)

%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(Q)rm -f *.o
	$(Q)rm -f *.d
	$(Q)rm -f *.elf
	$(Q)rm -f *.bin
	$(Q)rm -f *.hex
	$(Q)rm -f *.srec
	$(Q)rm -f *.list

%.flash: %.bin
	@printf "  FLASH   $<\n"
	$(Q)../flash.py $<

.PHONY: images clean

-include $(OBJS:.o=.d)

