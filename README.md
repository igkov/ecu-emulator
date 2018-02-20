# Engine ECU emulator

Engine ECU CAN-BUS emulator (Mitsubishi based). Make simple CAN-BUS events and process some PID-requests. This project used for stand-alone debug <a href="https://github.com/igkov/bcomp11">BCOMP software</a>.

More information on <a href="igorkov.org/bcomp11">igorkov.org/bcomp11</a> and <a href="igorkov.org/bcomp11v2">igorkov.org/bcomp11v2</a> pages.

Compilation
---------

Use <a href="http://www.keil.com">Keil MDK</a> or <a href="https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads">GCC ARM</a>.

Program MCU
---------

For program MCU, please use USB-UART converter with DTR/RST control and programming utility as <a href="http://www.flashmagictool.com/">Flash Magic</a> or <a href="https://sourceforge.net/projects/lpc21isp/">lpc21isp</a>.

