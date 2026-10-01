/* Cortex-M0+ (ARMv6-M): GCC lowers std::atomic acquire/release ordering to a
 * call to __sync_synchronize, which newlib does not provide on bare metal.
 * A DMB is the correct full barrier. Only assembled for the RP2040 build. */
#if !PICO_RP2350
    .syntax unified
    .cpu cortex-m0plus
    .thumb
    .section .text.__sync_synchronize, "ax", %progbits
    .global __sync_synchronize
    .type __sync_synchronize, %function
    .thumb_func
__sync_synchronize:
    dmb
    bx lr
    .size __sync_synchronize, . - __sync_synchronize
#endif
