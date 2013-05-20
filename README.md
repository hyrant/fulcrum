A system to support small microcontroller applications that communicate over
WiFi.

Fulcrum is a system consisting of a reference board design and a minimal kernel.
It is designed around the idea of a co-hosted application: the application code
shares the same flash and processor as the wireless interface.

It is fairly low level, however if you're looking for something much easier
to use and high level I'd recommend taking a look at 
http://www.sparkdevices.com/  I'm not associated with them in any way, but we
clearly had the same sort of idea.  Theirs looks like it's much more user
friendly.


Requirements
============

A working arm-none-eabi gcc.  Use https://github.com/esden/summon-arm-toolchain
to make it easy.

To actually flash the firmware, you will need a way of talking to the UART
on the board.  Likely any XBee-Computer interface will work as the pins are
the same.  I've personally used http://www.adafruit.com/products/247

For fabricating the boards, a service with a tolerance of 8mil should be be
sufficient.  I use http://www.oshpark.com/


General Goals
=============

The idea behind the creation of this was to create a module that could easily
be reprogrammed remotely but exposed most the STM32F1 microcontroller without
needing a second set of components (e.x. the "precursor" to this was a 
combination of RN-XV modules hooked to STM32F1s with bootloaders).  When
I saw the CC3000, it immediately became apparent that it was a very good fit
to what I was trying to do (low cost and small, with no RF design experience
required).

Following that, there are a large number of safeties that attempt to keep
the module remotely accessible even if the hosted application code crashes. 
This is no way stops any sort of "malicious" activity on the part of that
code, but it (usually) catches programming errors and allows for them to be
reprogrammed remotely.  I use the modules for remote automation, so having
to dig them out and fix a bad firmware is highly undesirable.

To make it easy to use the kernel implements both a simple preemptive threading
approach and all the interfaces needed to communicate with the network. 
Programming is accomplished through a simple web interface with the application
code being able to hook into it and serve its own pages (useful for
configuration).  The interface to change the IP and networking parameters are
NOT exposed to the user code, since that could easily render the module
not remotely accessible.

Usage
=====

Board Setup
-----------

Flashing the board is accomplished by holding the boot pad high then pulsing
the reset one low.  Depending on if write protect was enabled, it's also 
necessary to hold the boot pad high during flashing (since t required a reboot).

Initially, flash the patcher.  Once it completes (the LED goes into fast blink/
off cycles).  Flash the main kernel.

Once the kernel is installed, use the script moduleconfig.py to send the
appropriate configuration string to it.  It should also be possible to use
TI's SimpleConfig for completely wireless configuration, but I haven't
tested that.

Once fully configured the board will listen on port 80 for HTTP requests.
That is simply open a web browser to http://<IP>/fulcrum to access the 
programming interface.  The application can provide other URLs as well.

Board Reset 
-----------

The board can be reset to its initial state (setup parameters discarded and
re-entering setup mode) by holding the LED pin low during a reset.  That is,
ground the resistor near the LED (on th side closer to the microcontroller)
and then pull reset low momentarily.

Application Creation
--------------------

First, take a look at the two examples ("example" and "rnxv").  Basically 
they simply need to be located in the correct location and they will run
"transparently".  To interface with the kernel itself, use the generated
ld file to get the necessary symbols.

The application can provide new URLs for the device to serve using the kernel
HTTP engine.  Both examples demonstrate this by override in the root page.

Limitations
-----------

The kernel reserves the first 1k of ram and the first 25k of flash.  It also
requires exclusive use of DMA channels 2 and 3, EXTI15, the IWDG, and the 
SYSTICK timer.  A hook is provided to access the EXTI10-15 IRQ, but it will be 
fired frequently from the kernel's use of EXTI15.  Because the IWDG is 
configured for ~3 seconds, the user application code cannot block IRQ handlers
or with interrupts disabled for long.  However, the threading interface means
that individual threads are allowed to block (they just get preempted and the
kernel thread serviced) so this is generally not an issue.

No attempt was made to limit the power usage so the idle draw is 100-200mA.
The peak power is somewhat higher (the CC3000 says it peaks at 340mA alone).
