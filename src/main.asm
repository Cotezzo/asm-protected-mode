
; ==== MEMORY AND ARCH DIRECTIVES ============================================================================ ;
org 0x7C00
bits 16

; ==== BOOTLOADER MAIN ======================================================================================= ;
main:
    [bits 16]

    ; Setup segment registers and stack pointer
    mov ax, 0
    mov ds, ax
    mov ss, ax
    mov es, ax
    mov sp, 0x7C00

    ; Print something in real mode
    mov si, text16r_test
    call print16r

    ; ==== SWITCH TO 32BIT PROTECTED MODE ==================================================================== ;
    ; Necessary steps to switch from 16rm to 32pm:
    ; - Disable interrupts
    ; - Enable A20 Line (if disabled)
    ; - Load GDT
    ; - Enable Protected Mode in the CPU
    ; - Setup segment registers (+ far jump)
    ; https://wiki.osdev.org/Protected_Mode
    ; https://www.intel.com/content/dam/support/us/en/documents/processors/pentium4/sb/25366821.pdf (9.9)

    ; ==== A20 LINE ============================== ;
    ; Interrupts must not interfere while switching
    ; from REAL 16bit mode to PROTECTED 32bit mode
    cli

    ; Store if the A20 Line is enabled in the E flag
    call a20_check;7c13
    cmp ax, 1

    ; If A20 is disabled, enable it and check again
    je .a20_enabled
    mov si, text16r_a20_enabling
    call print16r
    call a20_enable
    call a20_check
    cmp ax, 1
    je .a20_enabled

    ;* A20 Failure
    ; If the A20 is still disabled, halt execution
    mov si, text16r_a20_disabled
    call print16r
    hlt

    ;* A20 Success
    ; Print that the A20 Line is enabled and go on
    .a20_enabled:
    mov si, text16r_a20_enabled
    call print16r

    ; ==== LOAD GDT ============================== ;
    lgdt [gdt_descriptor]

    ; ==== CPU SWITCH ============================ ;
    ; Control Register 0: special 32b register used
    ; to disable/enable CPU features.
    ; We only need to set the first bit in order to
    ; switch to protected mode.
    mov eax, cr0                                    ; Read control register 0 value
    or al, 1                                        ; Set first bit
    mov cr0, eax                                    ; Update CR0 value

    ; ==== JUMP TO PROTECTED CODE SEGMENT ======== ;
    ; Now, segments refer to the GDT entry to be
    ; used. 0x08 refers to the offset of the GDT
    ; 32bit protected mode code segment descriptor.
    ; The target must be a 32bit address, so dword
    ; is used to specify the size.
    ; Perform far jump to selector 08 to load CS
    ; with proper PM32 descriptor
    ;> jmp dword 0x08:.32pm;7c46
    jmp dword gdt.gdt_selector_32pm_cs-gdt:.32pm

    ; ==== 32BIT PROTECTED MODE ================== ;
    .32pm:
    [bits 32]
    ; Mode switch completed, re-enable interrupts
    sti

    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax

    ; Print something in protected mode
    mov esi, text32p_test
    call print32p

    ; ==== SWITCH BACK TO 16BIT REAL MODE ==================================================================== ;
    ; Necessary steps to switch from 16rm to 32pm:
    ; - Disable interrupts
    ; - Jump to 16bit protected mode code segment
    ; - Eventually load 16b selector in SS, DS, ES..
    ; https://wiki.osdev.org/Protected_Mode
    ; https://www.intel.com/content/dam/support/us/en/documents/processors/pentium4/sb/25366821.pdf (9.9)

    ; Interrupts must not interfere while switching
    cli

    ; ==== JUMP TO 16BIT PROTECTED CODE SEGMENT == ;
    ; Jump to 16b segment (word -> address size)
    jmp word gdt.gdt_selector_16pm_cs-gdt:.16pm

    ; ==== 16BIT PROTECTED MODE ================== ;
    .16pm:
    [bits 16]

    ; ==== CPU SWITCH ============================ ;
    ; Unset first bit to switch back to real mode
    mov eax, cr0                                    ; Read control register 0 value
    and al, ~1                                      ; Set first bit
    mov cr0, eax                                    ; Update CR0 value

    ; ==== JUMP TO 16BIT REAL MODE =============== ;
    ; Setup CS register for real mode
    jmp word 0x00:.16rm

    ; ==== 16BIT REAL MODE ======================= ;
    .16rm:

    ; Setup segment registers, gdt is not used now
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax

    ; Mode switch completed, re-enable interrupts
    sti



    ; Print something in real mode
    mov si, text16r_test
    call print16r

    ; Disable interrupts and halt execution
    cli
    hlt

; ==== METHODS =============================================================================================== ;
; Methods called by the main:
; - print16r
; - print32p
; - a20_check
; - a20_enable
; ==== PRINT ===================================== ;
; Define special characters and strings to print.
%define LF 0x0D
%define CR 0x0A
text16r_test: db `Real!`, LF, CR, 0
text16r_a20_enabling: db `Enabling A20 Line...`, LF, CR, 0
text16r_a20_disabled: db `Could not enable A20 Line`, LF, CR, 0
text16r_a20_enabled: db `A20 Line Enabled`, LF, CR, 0

; To move to a new line in VGA new lines, move
; cursor accordingly (buffer pointer), no CR/LF.
; New lines are used on an higher level to move
; into correct position.
%define VGA_TEXT_BUFFER 0xB8000
text32p_test: db `Fake!`, 0   ; VGA, no CR/LF

;* Prints a string stored in memory to TTY.
;* Input:
;* - SI: string memory address
print16r:
    [bits 16]
    push ax                                         ; Store AX
    push bx                                         ; Store BX
    push si                                         ; Store SI

    mov ah, 0x0E                                    ; Interrupt method: 10, E: Write Character in TTY
    mov bh, 0                                       ; Set page number used by int
    .loop:                                          ; Loop start
        lodsb                                       ; Load content of ds:[si] in AL, inc SI
        test al, al                                 ; Check whether the character was NULL (last one)
        jz .end                                     ; If it is (Z flag is set since AL is 0), exit

        int 0x10                                    ; Call int and display the character in AL
        jmp .loop

    .end:
    pop si                                          ; Restore previous SI value
    pop bx                                          ; Restore previous BX value
    pop ax                                          ; Restore previous AX value
    ret                                             ; If Z flag is set, return (pop ip)

;* Prints a string stored in memory to TTY.
;* Input:
;* - SI: string memory address
print32p:
    [bits 32]
    push eax
    push esi                                         ; Store SI
    push edi

    mov edi, VGA_TEXT_BUFFER

    mov ah, 0x01                                    ; Forecolor (4b) & Foreground (4b) color: Black, Green

    .loop:                                          ; Loop start
        lodsb                                       ; Load content of ds:[si] in AL, inc SI
        test al, al                                 ; Check whether the character was NULL (last one)
        jz .end                                     ; If it is (Z flag is set since AL is 0), exit

        mov [edi], al                               ; Write loaded character to VGA buffer
        inc edi                                     ; Move pointer to next byte
        mov [edi], ah                               ; Write charcter colors to VGA buffer
        inc edi                                     ; Move pointer to next byte
        jmp .loop

    .end:

    pop edi
    pop esi                                          ; Restore previous SI value
    pop eax
    ret                                             ; If Z flag is set, return (pop ip)

; ==== KEYBOARD CONTROLLER (PS/2 CONTROLLER) ===== ;
; To emulate old CPUs with 20 mem address bus (0-19)
; for retrocomp, 20th address line is forced to 0.
; In order to get full access to the memory bus,
; we need to disable this behaviour and enable it.
; The A20 gate is controlled by the motherboard's
; keyboard controller.
; https://wiki.osdev.org/%228042%22_PS/2_Controller

; I/O instructions:
; - in register, port ---> Receive data from port
; - out port, register --> Send data to port
; For I/O only the AX register can be used.

; Magic Numbers that idenfity the ports bound to the
; keyboard controller for data and command handling
%define PS2_PORT_DATA 0x60
%define PS2_PORT_COMMAND 0x64

; The keyboard controller handles specific commands
%define PS2_COMMAND_DISABLE_KEYBOARD 0xAD
%define PS2_COMMAND_ENABLE_KEYBOARD 0xAE
%define PS2_COMMAND_READ_FROM_OUTPUT 0xD0
%define PS2_COMMAND_WRITE_TO_OUTPUT 0xD1

;* Checks if the A20 Line is enabled.
;* Old BIOSes might not enable it by default.
;* Output:
;* - AX, 1 if the A20 line is enabled, 0 otherwise.
a20_check:
    [bits 16]
    ; In the bootloader, bytes 511 - 512 are 0xAA55:
    ; comparing [0:7DFE] to this value
    ; should always assert to true.
    ;> cmp byte [es:di], 0x55
    ;> cmp byte [es:di+1], 0xAA
    ; To check whether the A20 line wraps around or
    ; not, try to access the same address + 1MB and
    ; check the value against the one at [0:7DFE].
    ; If the address wraps around and the same
    ; values are accessed, the A20 line is disabled.

    ; Save used registers
    push es
    push di
    push ds
    push si
    
    ; Setup segments and index regs for mem access
    xor ax, ax
    mov es, ax                                      ; Set ES to 0x0000
    mov di, 0x7DFE                                  ; ES:DI = 0000:7DFE --> REAL ADDR: 0x007DFE

    not ax
    mov ds, ax                                      ; Set DS to 0xFFFF
    mov si, 0x7E0E                                  ; DS:SI = FFFF:7E0E --> REAL ADDR: 0x107DFE (ES:DI + 1MB)
                                                    ;  1    0    7    D    F    E
                                                    ; 0001 0000 0111 1101 1111 1110
                                                    ;    '-- 20th bit - if A20 line is disabled, this becomes 0

    ; Set return value to 1 by default
    mov ax, 1

    ; Lazy and insecure implementation:
    ; Read magic number from DS:SI
    ; If the comparison fails, exit
    ; ES:DI -----> 55
    ; ES:DI +1 --> AA
    ; DS:SI -----> 00   (If A20 already enabled)
    ; DS:SI +1 --> 00   (If A20 already enabled)
    ;> cmp byte [ds:si], 0x55
    ;> jne .exit ; Byte NOT equal, mem does NOT wrap

    ; Safe implementation:
    ; Clear previous ES:DI value with v1;
    ; Write v2 to DS:SI (different from v1);
    ; Compare ES:DI against v2.
    ; If memory wraps, modifying DS:SI affects
    ; ES:DI and the value is the same as v2.
    ; If not, A20 is already enabled (ES:DI value
    ; is still v1).
    ; The values previously stored at ES:DI and
    ; DS:SI are restored to their previous state.
    push word [es:di]
    push word [ds:si]
    mov byte [es:di], 0x00  ; Set ES:DI to v1
    mov byte [ds:si], 0xFF  ; Set DS:SI to v2
    cmp byte [es:di], 0xFF  ; Check ES:DI for v2
    pop word [ds:si]
    pop word [es:di]
    jne .exit   ; Byte NOT equal, mem does NOT wrap

    ; If no exit is called, all cmps set the E flag:
    ; A20 disabled, memory wraps around; ret 0
    xor ax, ax

    ; If exit is called, a cmp failed:
    ; A20 enabled, memory doesn't wrap around; ret 1
    ; 1 is the default value of ax, jumping here
    ; avoids the xor that sets ax to 1
    .exit:

    ; Restore used registers and return
    pop si
    pop ds
    pop di
    pop es
    ret

;* Uses the hardware I/O ports on the Keyboard 
;* Controller to try enabling the A20 Gate.
a20_enable:
    [bits 16]
    push ax

    ;! Before writing command to input buffer,
    ;! always wait for it to be empty first.

    ; Disable keyboard to avoid interruptions.
    call ps2_wait_ibuf_empty                        ; Wait for the input buffer to be empty
    mov al, PS2_COMMAND_DISABLE_KEYBOARD
    out PS2_PORT_COMMAND, al

    ; Send read command to get controller configuration
    ; byte.
    ; Wait until the config byte can be read from
    ; the output buffer, read it from data port
    ; and save to AL register.
    call ps2_wait_ibuf_empty                        ; Wait for the input buffer to be empty
    mov al, PS2_COMMAND_READ_FROM_OUTPUT
    out PS2_PORT_COMMAND, al
    call ps2_wait_obuf_full                         ; Wait for the output buffer to be full
    in al, PS2_PORT_DATA
    push ax                                         ; Save AL for later, it is used for I/O operations

    ; Send write command so that we can change
    ; the value of the config byte.
    call ps2_wait_ibuf_empty                        ; Wait for the input buffer to be empty
    mov al, PS2_COMMAND_WRITE_TO_OUTPUT
    out PS2_PORT_COMMAND, al

    ; Second bit of config byte toggles the A20
    ; line, set it and write new byte to DATA port.
    call ps2_wait_ibuf_empty                        ; Wait for the input buffer to be empty
    pop ax                                          ;! pop AFTER waiting, since wait does not restore AL value
    or al, 2
    out PS2_PORT_DATA, al

    ; Re-enable keyboard
    call ps2_wait_ibuf_empty                        ; Wait for the input buffer to be empty
    mov al, PS2_COMMAND_ENABLE_KEYBOARD
    out PS2_PORT_COMMAND, al

    ; Wait for process to end and restore registers
    call ps2_wait_ibuf_empty
    pop ax
    ret

;* Waits until the PS/2 input buffer can be used.
;! Uses AX register without restoring its state.
; The controller is slow, loop until the previous
; process finished and the input buffer is free.
ps2_wait_ibuf_empty:
    [bits 16]

    ; Command port returns the status register, it
    ; shows the controller's state with flags
    in al, PS2_PORT_COMMAND

    ; Second status register bit: input buffer state
    ; 0: the buffer is empty; 1: the buffer is full
    test al, 2  ; al & 2, discard result, set flags

    ; If the buffer is full, keep waiting
    jnz ps2_wait_ibuf_empty

    ; If the buffer is empty, return
    ret

;* Waits until the PS/2 output buffer is full.
;! Uses AX register without restoring its state.
; The controller is slow, loop until the previous
; process finished and the output buffer is filled.
ps2_wait_obuf_full:
    [bits 16]

    ; Command port returns the status register, it
    ; shows the controller's state with flags
    in al, PS2_PORT_COMMAND

    ; First status register bit: output buffer state
    ; 0: the buffer is empty; 1: the buffer is full
    test al, 1

    ; If the buffer is empty, keep waiting
    jz ps2_wait_obuf_full

    ; If the buffer is full, return
    ret


; ==== GLOBAL DESCRIPTOR TABLE (GDT) ============= ;
; In protected mode, segments are no longer used to
; access different memory banks, but to section
; memory in different parts, describing each with
; its purpose and required access levels.
; The GDT entries are 8 byte long, and describe
; these segments with an exact structure.
; https://wiki.osdev.org/Global_Descriptor_Table
; Base: segment start address (32 bits)
; Limit: segment length (20 bits)
; For flat memory model: base=0, limit=max mem addr
; There's no space for a 32bit limit, but we can use
; a flag ("granularity") which specifies the size of
; the memory blocks (1B or 4kB = 4096B = 2**10).
; Actually, the flag shifts the limit value to the
; left inserting 12 bits set to 1 (0x0 -> 0x0FFF).
; 0x000FFFFF + G flag = 0xFFFFFFFF --> 32bit limit

; Used GDT instructions / registers:
; - gdtr --> register that stores GDT's address
; - lgdt operand --> loads GDT's address to GDTR
;       given the GDT descriptor address

; Create GDT entries that we'll store and use
gdt:
    ; 1st entry: must be empty
    dq 0

    ; 2nd entry: 32b code segment, flat memory model
.gdt_selector_32pm_cs:
    dw 0xFFFF                                       ; Limit (0-15 bits)
    dw 0                                            ; Base (0-15 bits)
    db 0                                            ; Base (16-23 bits)
    db 1_00_1_1_0_1_0b                              ; Access Byte (reversed in little endian), left to right:
                                                    ; Present Bit, always 1 for a valid segment
                                                    ; Privilege (2b): privilige level required (Ring 0, 1, 2, 3)
                                                    ; Type: 0 for system segment, 1 for code or data segment
                                                    ; Executable: 0 for data segment, 1 for code segment
                                                    ; Direction/Confirming (code selectors):
                                                    ;   0: execution only allowed for specified ring privilege
                                                    ;   1: execution only allowed for equal/lower ring level,
                                                    ;      privilege remains but can read and execute the code
                                                    ;   (data selectors):
                                                    ;   0: Segments grows (ex: stack) up
                                                    ;   1: Segments grows down (offset > limit)
                                                    ; RW (code segments): 1 enables read (write never allowed)
                                                    ;    (data segments): 1 enables write (read always allowed)
                                                    ; Accessed: set by CPU when segment is accessed
    db 1_1_0_0_1111b                                ; Flags (reversed in little endian), left to right:
                                                    ; Granularity: 1 to shl the limit value (with 1s, not 0s)
                                                    ; Size: 0 for 16b protected mode, 1 for 32b protected mode
                                                    ; Long-Mode: 1 for 64b protected mode (Size must be 0)
                                                    ; Limit (16-19 bits)
    db 0                                            ; Base (24-31 bits)

    ; 3rd entry: 32b data segment, flat memory model
.gdt_selector_32pm_ds:
    dw 0xFFFF
    dw 0
    db 0
    db 1_00_1_0_0_1_0b                              ; Like 32bit code segment, but Executable bit set to 0
    db 1_1_0_0_1111b
    db 0

    ; 4th entry: 16b code segment, flat memory model
    ; used to switch back to real mode when needed
.gdt_selector_16pm_cs:
    dw 0xFFFF
    dw 0
    db 0
    db 1_00_1_1_0_1_0b
    db 0_0_0_0_1111b                                ; Like 32bit code segment, but Granularity and Size set to 0
    db 0

    ; 5th entry: 16b data segment, flat memory model
    ; used to switch back to real mode when needed
.gdt_selector_16pm_ds:
    dw 0xFFFF
    dw 0
    db 0
    db 1_00_0_0_0_1_0b                              ; Like 16bit code segment, but Executable bit set to 0
    db 0_0_0_0_1111b
    db 0

; In order to load GDT, another structure is
; requried: the GDT descriptor.
; It contains GDT's size - 1 (2B) and address (4B).
gdt_descriptor:
    dw gdt_descriptor - gdt - 1             ; Calculate size using address offsets
    dd gdt

; ==== PADDING AND SIGNATURE ================================================================================= ;
times 510-($-$$) db 0
dw 0xAA55