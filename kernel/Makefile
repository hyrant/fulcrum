# Derived from libopencm3 Makefiles

OBJS		= entry.o synchronization.o init.o transport.o netapi.o \
				control.o setup.o util.o version.o exports.o \
				server.o request.o response.o post.o
BINARY		= kernel

VERSION		?= $(or $(shell git describe --tags 2>/dev/null), \
			$(shell git rev-parse HEAD 2>/dev/null), \
			1.0)
			
F_CPU		?= 24000000

PREFIX		?= arm-none-eabi
CC		= $(DISTCC) $(PREFIX)-gcc
LD		= $(DISTCC) $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
SIZE		= $(PREFIX)-size
CFLAGS		+= --std=gnu99 -Os -g -Wall -Wextra -DSTM32F1 -I.. -I../common \
		   -fno-common -mcpu=cortex-m3 -mthumb -msoft-float -MD \
		   -DFULCRUM_VERSION='"$(VERSION)"' \
		   -DF_CPU=$(F_CPU)
LDSCRIPT	= $(BINARY).ld
LDFLAGS		+= -lc -lnosys -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections \
		   -mcpu=cortex-m3 -msoft-float

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
	
version.c: version.h $(subst version.o,,$(OBJS))
	$(Q)touch $@
	
exports.c: exports.csv genexports.pl
	$(Q)./genexports.pl $<

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
	$(Q)rm -f exports.ld exports.c ../api.ld

%.flash: %.bin
	@printf "  FLASH   $<\n"
	$(Q)../flash.py $<

.PHONY: images clean

-include $(OBJS:.o=.d)

