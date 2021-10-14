#include "dma_util.h"

void SetDMA(DMAInit * DMA_init) {

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA_Stream_TypeDef * DMA_Stream = DMA_init->DMA_Stream;

    DMA_Stream->CR &= ~(DMA_SxCR_EN);

    DMA_Stream->PAR = DMA_init->PAR;
    DMA_Stream->M0AR = DMA_init->M0AR;
    DMA_Stream->NDTR = DMA_init->NDTR;

    DMA_Stream->CR = 0;
    DMA_Stream->CR |= DMA_init->CR_CHSEL;
    DMA_Stream->CR |= DMA_init->CR_MINC;
    DMA_Stream->CR |= DMA_init->CR_PINC;
    DMA_Stream->CR |= DMA_init->CR_DIR; 
    DMA_Stream->CR |= DMA_init->CR_PL;
    DMA_Stream->CR |= DMA_init->CR_MSIZE; 
    DMA_Stream->CR |= DMA_init->CR_PSIZE;
}