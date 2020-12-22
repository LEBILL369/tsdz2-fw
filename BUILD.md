# Build

Requirements:
* SDCC 4.0+ (http://sdcc.sourceforge.net/)
* Make for Windows (http://gnuwin32.sourceforge.net/packages/make.htm) 

1. Install SDCC, make sure to check box to add to your PATH.
2. Install Make for Windows (not needed if you already have make through cygwin etc.)
3. Run make

# Flash

Requirements:
* STVP

Microcontroller: STM8S105x6

1. Run: make hex
2. Use STVP to flash program main.hex.
