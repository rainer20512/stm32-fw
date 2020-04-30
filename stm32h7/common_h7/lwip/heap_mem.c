#include "lwip/opt.h"

/* RHB added : Allocate the defined block of heap memory to a linker assigned memory area */

unsigned char mem_heap[MEM_SIZE] __attribute__((section(".lwipheap"))); /* LwIP heap memory */
//unsigned char mem_heap[MEM_SIZE]; /* LwIP heap memory */