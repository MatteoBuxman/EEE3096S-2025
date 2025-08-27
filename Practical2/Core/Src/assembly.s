/*
* assembly.s
*
*/

@ DO NOT EDIT
.syntax unified
.text
.global ASM_Main
.thumb_func

@ DO NOT EDIT
vectors:
.word 0x20002000
.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

@ Some code is given below for you to start with
LDR R0, RCC_BASE @ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
LDR R1, [R0, #0x14]
LDR R2, AHBENR_GPIOAB @ AHBENR_GPIOAB is defined under LITERALS at the end of the code
ORRS R1, R1, R2
STR R1, [R0, #0x14]

@ Configure GPIOA pins 0-3 as inputs (ADDED)
LDR R0, GPIOA_BASE
MOVS R1, #0x00000000    @ Set PA0-PA3 as inputs (00 = input mode)
STR R1, [R0, #0x00]     @ Write to MODER register

LDR R0, GPIOA_BASE @ Enable pull-up resistors for pushbuttons
MOVS R1, #0b01010101
STR R1, [R0, #0x0C]

LDR R1, GPIOB_BASE @ Set pins connected to LEDs to outputs
LDR R2, MODER_OUTPUT
STR R2, [R1, #0]
MOVS R2, #0 @ NOTE: R2 will be dedicated to holding the value on the LEDs

main_loop:
@ Read GPIO inputs once for efficiency
LDR R0, GPIOA_BASE
LDR R3, [R0, #0x10]

@ Check if SW3 is pressed (freeze)
MOVS R4, #8
TST R3, R4
BEQ freeze_pattern

@ Check if SW2 is pressed (set to 0xAA)
MOVS R4, #4
TST R3, R4
BEQ set_pattern_AA

@ Normal counting logic
BL apply_delay
BL update_counter
B write_leds

set_pattern_AA:
MOVS R2, #0xAA
BL apply_delay
B write_leds

freeze_pattern:
@ Do nothing - just maintain current pattern
BL apply_delay
B write_leds

update_counter:
@ Use the already loaded R3 value from main_loop
MOVS R4, #1
TST R3, R4
BEQ increment_by_2

@ Default: increment by 1
ADDS R2, R2, #1
MOVS R4, #0xFF
ANDS R2, R2, R4
BX LR

increment_by_2:
ADDS R2, R2, #2
MOVS R4, #0xFF
ANDS R2, R2, R4
BX LR

apply_delay:
@ Use the already loaded R3 value from main_loop
MOVS R4, #2
TST R3, R4              @ FIXED: Removed stray 'a'
BEQ use_short_delay

@ Use long delay (0.7s)
LDR R4, LONG_DELAY_CNT
B delay_loop

use_short_delay:
LDR R4, SHORT_DELAY_CNT
B delay_loop

delay_loop:
SUBS R4, R4, #1
BNE delay_loop
BX LR

write_leds:
LDR R1, GPIOB_BASE
@ Clear the LED bits first, then set new pattern
LDR R3, [R1, #0x14]     @ Read current ODR value
MOVS R4, #0xFF          @ Create mask for LED bits
BICS R3, R3, R4         @ Clear lower 8 bits (LED positions)
ORRS R2, R2, R3         @ Combine LED pattern with other bits
STR R2, [R1, #0x14]     @ Write back to ODR
B main_loop

@ LITERALS; DO NOT EDIT
.align
RCC_BASE: .word 0x40021000
AHBENR_GPIOAB: .word 0b1100000000000000000
GPIOA_BASE: .word 0x48000000
GPIOB_BASE: .word 0x48000400
MODER_OUTPUT: .word 0x5555

LONG_DELAY_CNT: .word 1200000 @ Approximate 0.7 second delay
SHORT_DELAY_CNT: .word 600000 @ Approximate 0.3 second delay
