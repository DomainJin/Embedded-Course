#include "myLibrary.h"

void RCC_Init ()
{
	uint32_t* RCC_CR = (uint32_t*)(RCC_Base_Addr + 0x00);
	uint32_t* RCC_PLLCFGR = (uint32_t*)(RCC_Base_Addr + 0x04);
	uint32_t* RCC_CFGR = (uint32_t*)(RCC_Base_Addr + 0x08);

	*RCC_CR |= (1 <<  16);
	while(!(*RCC_CR & (1 << 17)));

	*RCC_PLLCFGR |= (1 <<  22);

	//M là bit [0:5] của PLLCFGR - chia 8
	*RCC_PLLCFGR &= ~(0b111111 << 0);
	*RCC_PLLCFGR |= (8 <<  0);
	//N là bit [6:14] - nhân 200
	*RCC_PLLCFGR &= ~(0x1ff << 6);
	*RCC_PLLCFGR |= (200 <<  6);
	//P là bit [16:17] - chia 2
	*RCC_PLLCFGR &= ~(0b11 << 16);
	*RCC_PLLCFGR |= (0b00 << 16);

	*RCC_CR |= (1 <<  24);
	while(!(*RCC_CR & (1 << 25)));

	*RCC_CFGR &= ~(0b1111 << 0);
	*RCC_CFGR |= (2 << 0);
	while (((*RCC_CFGR >> 2) & 0x3) != 0x2);
}

// AHP1

//GPIO
void RCC_GPIOA_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 0);
}
void RCC_GPIOB_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 1);
}
void RCC_GPIOC_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 2);
}
void RCC_GPIOD_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 3);
}
void RCC_GPIOE_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 4);
}

//DMA
void RCC_DMA1_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 21);
}
void RCC_DMA1_Enable() {
	volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_Base_Addr + 0x30);
	*RCC_AHB1ENR |= (1 << 22);
}

//ABP2
