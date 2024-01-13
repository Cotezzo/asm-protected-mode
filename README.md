# asm-protected-mode
## Introducton
Simple bootloader to test switching from 16-bit real mode to 32-bit protected mode in assembly.
Made as a training to the actual implementation for my [Porcheria-OS](https://github.com/Cotezzo/porcheria-os) project.

## Covered Topics
- Real Mode vs Protected Mode
- P/S2 Controller and A20 Line
- Global Descriptor Table (GDT)
- CPU's Control Register 0
- VGA Text UI

## Usage
### Installation
The main necessary packages (apt) are:
- `nasm`: assembler for our x86 ISA.
- `qemu-system`: emulation software.
- `dosfstools` and `mtools`: filesystem utils.

### Makefile
The Makefile creates a bootable image that can be run on QEMU. The supported targets are:
- `make` or `make all`: build the image.
- `make run`: build the image and run QEMU.
- `make dbg`: build the image and run QEMU + GDB debugger.
- `make clean`: deletes the project target directory.

All the assembler, emulation and debugging configurations are set in the Makefile.
The output directory for binary and image files is the `target` directory.

### Other useful tools
- `gdb`: debugging software that can be used to watch the state of run instructions, CPU registers and memory step by step, in order to better understand machine code execution and find bugs.
- `GHex` (or any other hex editor): useful to study the "anatomy" of the finished image file and make sure that the file system and files are in the expected state and location.