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


void UART2_SendChar (uint8_t c)
{
	USART2->DR = c; // load the data into DR register
	while (!(USART2->SR & (1<<6)));  // Wait for TC to SET.. This indicates that the data has been transmitted
}
	
void UART2_SendString (char *string)
{
	while (*string) UART2_SendChar (*string++);
}

void SetADC(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // ADC1->SQR1 = 0;
    ADC1->CR1 = 0;
    ADC2->CR2 = 0;
    ADC->CCR |= ADC_CCR_ADCPRE_0;                   // ADC prescaler = 4, 10.5 MHz
    ADC1->CR1 &= ~(ADC_CR1_RES_0);                  // resolution 12 bit

    ADC1->CR2 |= ADC_CR2_EOCS;                      // Enable EOC after conversion
    ADC1->CR1 |= ADC_CR1_EOCIE;

    ADC1->CR2 &= ~(ADC_CR2_ALIGN);
	ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_DDS;
    //TODO: enable DMA

    ADC1->SQR1 |= ADC_SQR1_L_0;                     // 1 channel
    ADC1->SQR3 = ADC_SQR3_SQ1_4 | ADC_SQR3_SQ1_1;   // 18th channel
    ADC1->SMPR1 |= (7<<24);

    ADC->CCR |= ADC_CCR_TSVREFE;                    // enable temperature sensor
    ADC1->CR2 |= ADC_CR2_ADON;                     // Enable ADC
    uint32_t i = 0;
    while(i < 100000) ++i;

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 0);
    // minimum sampling time for TS = 17 ms
}

uint16_t data[10] = {0};

void DMA2_Stream0_IRQHandler(void) {
    DMA2->LIFCR |= DMA_LISR_TCIF0;
    //UART2_SendString("DATA");
    static int i = 0;
    static char temp_str[10];
    for (i = 0; i < 10; ++i){
        sprintf(temp_str, "%u ", data[i]);
        UART2_SendString(temp_str);
    }
    UART2_SendString("\n\r\n\r");

}

void ADC_IRQHandler(void) {
    ADC1->SR = 0;
    static char temp_str[10];
    float temperature = (((3.3 / 4096) * (float)(ADC1->DR) - 0.76) / 0.0025) + 25;
    sprintf(temp_str, "%.2f C\n\r", temperature);
    UART2_SendString(temp_str);
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
    DMA_init.CR_CHSEL = DMA_SxCR_CHSEL_2; // channel 4
    DMA_init.CR_MINC = DMA_SxCR_MINC;
    DMA_init.CR_PL = DMA_SxCR_PL_0;
    DMA_init.CR_DIR = DMA_SxCR_DIR_0;
    DMA_init.CR_MSIZE = DMA_SxCR_MSIZE_0;

    SetDMA(&DMA_init);

    DMAInit DMA_init2 = {0};
    DMA_init2.DMA_Stream = DMA2_Stream0; // channel 0 by default
    DMA_init2.CR_MINC = DMA_SxCR_MINC;
    DMA_init2.PAR = (uint32_t)&(ADC1->DR);
    DMA_init2.M0AR = (uint32_t)data;
    DMA_init2.NDTR = sizeof(data) / sizeof(char);
    DMA_init2.CR_CIRC = DMA_SxCR_CIRC;
    DMA_init2.CR_MSIZE = DMA_SxCR_MSIZE_0;

    SetDMA(&DMA_init2);

    // DMA2_Stream0->CR |= DMA_SxCR_TCIE; // enable interrupt

    NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    SetADC();

    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    DMA2->LIFCR |= DMA_LISR_TCIF0;
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    volatile uint32_t i = 0;
    for(i = 0; i < 8400000; ++i);
    
    char temp_str[10];
    while(1) {
        GPIOA->ODR ^= GPIO_ODR_ODR_5;
        for(i = 0; i < 8400000; ++i);
        
        //DMA1->HIFCR |= DMA_HIFCR_CTCIF6; // clear end transfer interrupt bit
        //DMA1_Stream6->CR |= DMA_SxCR_EN;
    }
}