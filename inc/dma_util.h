#ifndef DMA_UTIL_H
#define DMA_UTIL_H

#include "stm32f4xx.h"
#include <system_stm32f4xx.h>

typedef struct DMAInit {
    DMA_Stream_TypeDef * DMA_Stream;
    uint32_t PAR;
    uint32_t M0AR;
    uint32_t NDTR;
    uint32_t CR_CHSEL;
    uint32_t CR_MINC;
    uint32_t CR_PINC;
    uint32_t CR_DIR;
    uint32_t CR_PL;
    uint32_t CR_MSIZE;
    uint32_t CR_PSIZE;
    uint32_t CR_CIRC
} DMAInit;


void SetDMA(DMAInit * DMA_init);

#endif