Production binaries created via the IAR EW for ARM output converter. Useful if you have wiped the Maple bootloader.

Both the .hex and the .bin files are the same only packed in different format. You need a proper JTAG or SWD debugger tool with a 10-pin adapter (or set of jumper cables) You need a flash programmer software (or a working OpenOCD script) to upload the binaries to the OLIMEXINO-STM32.

Additionally, you might be able to restore the Maple bootloader via the built-in STM32 bootloader (without the need of a debugger or programmer)

----

The easiest (and tested) way to program the OLIMEXINO-STM32 would be using the following Olimex products:

1. ARM-JTAG-COOCOX - JTAG/SWD programmer/debugger - https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-COOCOX/open-source-hardware

2. ARM-JTAG-10-20 - adapter for the small 10 pin connector - https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-20-10/

The good thing about CoCoox IDE is that it has an easy-to-use flash software component named CoFlash. In case you don't have other ARM experience 

----

The second way for programming is useful if you already own an OpenOCD debugger (OLIMEX ARM-USB-TINY-H, OLIMEX ARM-USB-OCD-H, or any other OpenOCD-compatible debugger). You can upload the demo with an OpenOCD command line that looks like:

openocd -f /openocd/scripts/interface/olimex-arm-usb-tiny-h.cfg -f /openocd/scripts/target/stm32f1x.cfg -c "init" -c "program <bootloaderfile>.hex verify reset"

Note that the cfg files are different for different OpenOCD debuggers and edit that part. Also your local paths might be different as well as the name of the executable in your OpenOCD version.

----

The third way to program the board doesn't require a debugger. The method, however, requires to establish connection with UART1 of the board which might still require additional hardware - cable that evens the logic voltage levels (CMOS-TTL).

After that you need to download ST's Flash Loader Demo tool and enter bootloader mode. This is done if you hold button "BUT" down and then press briefly the reset button. Release button "BUT" and you should be in ST bootloader mode. Note that you would need to install dirvers probably at this point.

After that use the ST's Flash Loader Demo to upload the hex files of the archive.