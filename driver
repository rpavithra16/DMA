#include "uart.h"
#include <stdint.h>

#define DMA_SRC_ADDR        0x00061400
#define DMA_DST_ADDR        0x00061404
#define DMA_COUNT_ADDR      0x00061408
#define DMA_START_ADDR      0x0006140C
#define DMA_DONE_ADDR       0x00061410
#define DMA_HALF_DONE_ADDR  0x00061414
//#define DMA_ERROR_ADDR    0x00061418

#define MEMORY_SRC          0x81000000
#define MEMORY_DST          0x82000000  
#define WORD_COUNT          15

int main()
{
    volatile unsigned int *dma_src       = (unsigned int *)DMA_SRC_ADDR;
    volatile unsigned int *dma_dst       = (unsigned int *)DMA_DST_ADDR;
    volatile unsigned int *dma_count     = (unsigned int *)DMA_COUNT_ADDR;
    volatile unsigned int *dma_start     = (unsigned int *)DMA_START_ADDR;
    volatile unsigned int *dma_done      = (unsigned int *)DMA_DONE_ADDR;
    //volatile unsigned int *dma_error   = (unsigned int *)DMA_ERROR_ADDR;

    volatile unsigned int *src_mem = (unsigned int *)MEMORY_SRC;
    volatile unsigned int *dst_mem = (unsigned int *)MEMORY_DST;

    // Initialize source memory with dummy data
    for (int i = 0; i < WORD_COUNT; i++) {
        src_mem[i] = 0xBCA00000 + i;
    }
    for (int i = 0; i < WORD_COUNT; i++) {
        dst_mem[i] = 0x00000000;
    }

    // Configure DMA
    *dma_src   = MEMORY_SRC;
    *dma_dst   = MEMORY_DST;
    *dma_count = WORD_COUNT;
    *dma_start = 1;

    // Simple delay (instead of busy-waiting on DMA_DONE)
    for (int i = 0; i < 10000000; i++);

    // Verify transfer
    int success = 1;
    for (int i = 0; i < WORD_COUNT; i++) {
        if (dst_mem[i] != src_mem[i]) {
            success = 0;
            break;
        }
    }

    // Program ends — correctness can be verified via GDB memory dumps
    return success ? 0 : 1;
}
