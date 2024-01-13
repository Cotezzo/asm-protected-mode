# Define source directory where the .asm is stored.
# Define build directory where the build output is stored.
SRC_DIR=./src
TARGET_DIR=./target

# Define source assembly code file.
# Define output binary and image files.
SRC_ASM=${SRC_DIR}/main.asm
TARGET_BIN=${TARGET_DIR}/main.bin
TARGET_IMG=${TARGET_DIR}/floppy.img

# Define assembler command and options.
ASM=nasm
ASM_OPTIONS=-f bin -o ${TARGET_BIN}

# Define the emulator command and options to start the floppy image.
# Intel Architecture, 32bit (known as i386).
# ISA: x86-32, 32bit version of x86 16bit instructions.
# The -fda option is used to load the .img file as a disk.
# Use -d to log specified items ("-d help" for list, "-D file" to log in file).
EMU=qemu-system-i386
EMU_OPTIONS=-fda ${TARGET_IMG} -d int,cpu_reset -no-reboot -D ${TARGET_DIR}/floppy.log

# Define .gdb debug script file and content (\ \n for multiline support).
GDB_SCRIPT_PATH=${TARGET_DIR}/debug-script.gdb
GDB_CONFIG="\
\nset disassembly-flavor intel\
\ntarget remote | ${EMU} ${EMU_OPTIONS} -S -gdb stdio -m 32\
\nlayout asm\
\nb *0x7c00\
\nc\
\nx/16xh 0x7dfe"


# Default behaviour for the command 'make': create img if something changed.
all: ${TARGET_IMG}

# Command to build and run with QEMU the operating system.
run: all
	${EMU} ${EMU_OPTIONS}

# Build and debug os with QEMU and GDB; load .gdb config and scripts.
dbg: all
	echo ${GDB_CONFIG} > ${GDB_SCRIPT_PATH}
	gdb -tui -x ${GDB_SCRIPT_PATH}

# In order to create the .img file, the .bin file must be created first.
# If an updated .bin file has been created, copy the binary and rename it
# to a .img file.
# Since we are assuming our "floppy" has a 1.44MB of memory, we adjust the
# dimension of our file to 1_440_000 bytes using the truncate command.
${TARGET_IMG}: ${TARGET_BIN}
	cp ${TARGET_BIN} ${TARGET_IMG}
	truncate -s 1440k ${TARGET_IMG}

# In order to create the .bin file, the .asm file must be modified.
# If the .asm source has not been modified since the last 'make' command,
# the following instructions won't be run.
# If the .asm has to be assembled, run the assembler command (nasm) and create
# the binary file. Once the .bin has been created, the .img requirements will
# be updated and relative commands would be run.
${TARGET_BIN}: ${TARGET_DIR} ${SRC_ASM}
	${ASM} ${SRC_ASM} ${ASM_OPTIONS}

${TARGET_DIR}:
	mkdir -p ${TARGET_DIR}

# No requirement needed to run 'make clean': just delete ./build/*.
clean:
	rm ${TARGET_DIR}/*