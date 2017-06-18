
#***************************************************************************************************************************

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.text
.global _putc
.global _puts

.equ SYS_PUTC, 0x03
.equ SYS_PUTS, 0x04
.equ SEMIHOST, 0xab

#***************************************************************************************************************************

_putc:
	push {r1,r0}

   	mov r1, sp
   	ldr r0, =SYS_PUTC
	bkpt SEMIHOST

	pop {r1,r0}

	bx lr

#***************************************************************************************************************************

_puts:
	push {r0,r1}

   	mov r1,r0
   	ldr r0, =SYS_PUTS
	bkpt SEMIHOST

	pop {r0,r1}

	bx lr
