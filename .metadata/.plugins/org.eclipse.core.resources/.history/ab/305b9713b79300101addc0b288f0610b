/*
 * main.h
 *
 *  Created on: Jun 21, 2025
 *      Author: Dell
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stdint.h"

// GPIO base addresses
#define GPIOA_BASE_ADDR 0x40020000
#define GPIOB_BASE_ADDR 0x40020400
#define GPIOC_BASE_ADDR 0x40020800
#define GPIOD_BASE_ADDR 0x40020C00
#define GPIOE_BASE_ADDR 0x40021000

// SPI, UART, DMA, FLASH, I2C, RCC base addresses
#define SPI1_BASE_ADDR  0x40013000
#define SPI_Base_Addr   0x40013000
#define UART1_BASE_ADDR 0x40011000
#define DMA2_BASE_ADDR  0x40026400
#define FLASH_BASE_ADDR 0X40023C00
#define I2C_Base_Addr   0x40005400
#define RCC_Base_Addr   0x42003800

// EXTI base address
#define EXTI0_BASE_ADDR 0x40013C00 // line 0, position 6

// LSM303 address
#define LSM303_ACC_ADDR 0x19

// LED enum
typedef enum
{
	LED_Yellow,
	LED_Orange,
	LED_Red,
	LED_Blue,
} led_t;

// LED state enum
typedef enum
{
	OFF_LED,
	ON_LED,
} led_state_t;


// Function prototypes

void Leds_Init(void);
void Led_Ctrl(led_t led, led_state_t state);
void PA0_input_Init(void);
int  PA0_Read(void);
void EXTI0_IRQHandler(void);
void My_Handler(void);
void EXTI_Init(void);

void dma_init(void);
void uart_init(void);
void uart_send(char data);
char uart_recv(void);
void uart_send_string(char* str);

void USART1_IRQHandler(void);

__attribute__((section(".Ham_tren_Ram"))) void flash_unlock(void);
__attribute__((section(".Ham_tren_Ram"))) void flash_erase(int sector_number);
__attribute__((section(".Ham_tren_Ram"))) void flash_program(char *address, char* data_buf, int size);
__attribute__((section(".Ham_tren_Ram"))) void update(void);

void I2C_Init(void);
void I2C_Write(uint8_t slave_addr, uint8_t reg_addr, uint8_t value);
uint32_t I2C_Read(uint32_t slave_addr, uint32_t slave_register_addr);

void SPI_Init(void);
uint8_t SPI_Read_Reg(uint8_t reg_addr);
void SPI_Write_Reg(uint8_t reg_addr, uint8_t value);

void RCC_Init(void);

#endif /* MAIN_H_ */
