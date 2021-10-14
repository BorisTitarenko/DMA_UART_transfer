#include "clock_util.h"

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
	RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (P << 16);
	RCC->PLLCFGR |= (M << 0);
	RCC->PLLCFGR |= (N << 6);
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    // Enable PPL and wait for it become ready
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Set PPL as system clock and wait for it set
    RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);    
}

void SetClockHSE(void) {
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait to become ready

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    RCC->CFGR |= RCC_CFGR_SW_HSE;
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSE));

}