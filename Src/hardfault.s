  .section .init, "ax"
  .thumb_func 
  .global HardFault_Handler
HardFault_Handler:

        movs r0,#4
        movs r1, lr
        tst r0, r1
        beq _MSP
        mrs r0, psp
        b _HALT
_MSP:
      mrs r0, msp
_HALT:
      ldr r1,[r0,#20]
      bl hard_fault_handler_c
      bkpt #0
