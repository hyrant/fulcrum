# Derived from libopencm3 Makefiles

OBJS		= main.o http.o serial.o led.o cli.o gpio.o
BINARY		= output

PREFIX		?= arm-none-eabi
CC		= $(DISTCC) $(PREFIX)-gcc
LD		= $(DISTCC) $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
SIZE		= $(PREFIX)-size
CFLAGS		+= --std=gnu99 -Os -g -Wall -Wextra -DSTM32F1 -I.. \
		   -fno-common -mcpu=cortex-m3 -mthumb -msoft-float -MD
LDSCRIPT	= $(BINARY).ld
LDFLAGS		+= -lopencm3_stm32f1 -lc -lnosys -T$(LDSCRIPT) -nostartfiles \
		   -Wl,--gc-sections -mthumb -march=armv7 -mfix-cortex-m3-ldrd \
		   -msoft-float

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif

.SUFFIXES: .elf .bin .list .images
.SECONDEXPANSION:
.SECONDARY:

all: images

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.list
	@echo "*** $* images generated ***"
	$(Q)$(SIZE) $(*).elf

%.bin: %.elf
	@printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

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
	$(Q)rm -f *.list

.PHONY: images clean

-include $(OBJS:.o=.d)

