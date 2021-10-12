#include "stm32f4xx.h"
#include <system_stm32f4xx.h>

#define DEFAULT_DELAY   1000

void SetClock(void) {
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait to become ready

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

    // configure flash for high speed clock
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	//Set PLL prescalers to 84 MHz
	RCC->PLLCFGR = 0x0;
    RCC->PLLCFGR |= (0 << 16);
	RCC->PLLCFGR |= (4 << 0);
	RCC->PLLCFGR |= (84 << 6);
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    // Enable PPL and wait for it become ready
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Set PPL as system clock and wait for it set
    RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);    
}


// Set HSE as clock source, Sysclock will be 8 mHz
void SetClockHSE(void) {
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait to become ready

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    RCC->CFGR |= RCC_CFGR_SW_HSE;
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSE));

}

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

    USART2->CR3 |= USART_CR3_DMAT; // enable DMA for transmit
     
}

volatile char data[] = "I'm data\n";

void DMASet(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA1_Stream6->PAR = (uint32_t)(&USART2->DR);
    DMA1_Stream6->M0AR = (uint32_t)(data);
    DMA1_Stream6->NDTR = sizeof(data) / sizeof(char);

    DMA1_Stream6->CR = 4 << DMA_SxCR_CHSEL // select channel 4
    | 0 << DMA_SxCR_PL // priority low
    | 0 << DMA_SxCR_MSIZE // 8 bit block size for memory
    | 0 << DMA_SxCR_PSIZE // 8 bit block size for UART
    | 1 << DMA_SxCR_MINC // enable memory increment
    | 0 << DMA_SxCR_PINC // disable peripheral increment
    | 0 << DMA_SxCR_CIRC // disable circular mode
    | 1 << DMA_SxCR_DIR; // memory to peripheral
}


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
    DMASet();
    volatile uint32_t i = 0;
    while(1) {
        GPIOA->ODR ^= GPIO_ODR_ODR_5;
        for(i = 0; i < 8400000; ++i);

        UART2_SendChar('A');
    }
}