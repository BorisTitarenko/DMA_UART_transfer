#include "stm32f4xx.h"
#include <system_stm32f4xx.h>
#include "clock_util.h"
#include "dma_util.h"

#define DEFAULT_DELAY   1000


void SetGPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER5_0; // output mode
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5); // push-pull
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5); // no pull
}

void SetUART(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA2 alternative function TX
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; 
    GPIOA->AFR[0]  |= (7<<8);

    // PA3 alternative function RX
    GPIOA->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
    GPIOA->AFR[0]  |= (7<<12);

    USART2->BRR = (13<<0) | (22<<4); // 115200 baud rate 

    USART2->CR1 &= ~(USART_CR1_M); // 8 bit package
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    USART2->CR2 &= ~(USART_CR2_STOP_0); // 1 stop bit

    USART2->CR3 = USART_CR3_DMAT; // enable DMA for transmit
     
}

volatile char data[] = "I'm data\n";



void UART2_SendChar (uint8_t c)
{
	USART2->DR = c; // load the data into DR register
	while (!(USART2->SR & (1<<6)));  // Wait for TC to SET.. This indicates that the data has been transmitted
}
	
void UART2_SendString (char *string)
{
	while (*string) UART2_SendChar (*string++);
}


int main(void) {
    SystemInit();

    SetClock();

    SystemCoreClockUpdate();

    SetGPIO();

    SetUART();

    DMAInit DMA_init = {0};
    
    DMA_init.DMA_Stream = DMA1_Stream6;
    DMA_init.PAR = (uint32_t)&(USART2->DR);
    DMA_init.M0AR = (uint32_t)data;
    DMA_init.NDTR = sizeof(data) / sizeof(char);
    DMA_init.CR_CHSEL = DMA_SxCR_CHSEL_2; //channel 4
    DMA_init.CR_MINC = DMA_SxCR_MINC;
    DMA_init.CR_PL = DMA_SxCR_PL_0;
    DMA_init.CR_DIR = DMA_SxCR_DIR_0;

    SetDMA(&DMA_init);

    volatile uint32_t i = 0;
    while(1) {
        GPIOA->ODR ^= GPIO_ODR_ODR_5;
        for(i = 0; i < 8400000; ++i);

        DMA1->HIFCR |= DMA_HIFCR_CTCIF6; // clear end transfer interrupt bit
        DMA1_Stream6->CR |= DMA_SxCR_EN;
    }
}