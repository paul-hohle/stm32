
#***************************************************************************************************************************

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.text
.global _putc
.global _puts

#***************************************************************************************************************************

_putc:
	push { lr }
	push { r1 }
	push { r0 }

   	mov r1, sp
   	ldr r0, =0x03 
	bkpt 0xab

	pop { r0 }
	pop { r1 }
	pop { pc }

#***************************************************************************************************************************

_puts:
	push { r1,r0,lr }

   	mov r1,r0
   	ldr r0,=0x04
	bkpt 0xab

	pop { r1,r0,pc }

