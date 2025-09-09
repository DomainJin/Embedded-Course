/*
 * main.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Dell
 */
#include "main.h"
#include "string.h"

#define GPIOA_BASE_ADDR 0x40020000
#define GPIOB_BASE_ADDR 0x40020400
#define GPIOC_BASE_ADDR 0x40020800
#define GPIOD_BASE_ADDR 0x40020C00
#define GPIOE_BASE_ADDR 0x40021000
#define SPI_Base_Addr 0x40013000
void Leds_Init()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// TODO: config PD12, PD13, PD14, PD15 in OUTPUT MDOE
	uint32_t *GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0b11111111 << 24);
	*GPIOD_MODER |= (0b01010101 << 24);

}

typedef enum
{
	LED_Yellow,
	LED_Orange,
	LED_Red,
	LED_Blue,
} led_t;

typedef enum
{
	OFF_LED,
	ON_LED,
} led_state_t;

void Led_Ctrl(led_t led, led_state_t state)
{
	// TODO: write state ODR
	uint32_t *GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(state == ON_LED){
		*GPIOD_ODR |= (0b1 << (12 + led));

	}
	if(state == OFF_LED){
		*GPIOD_ODR &= ~(0b1 << (12 + led));
	}
}

// config PA0 input: button signal
void PA0_input_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	uint32_t *GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b11 << 0);

}

int PA0_Read()
{
    uint32_t *GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
    uint32_t result = *GPIOA_IDR & 0b1;
    return result;
}

#define EXTI0_BASE_ADDR 0x40013C00 // line 0, position 6

uint32_t buttonSignal = 0;
void EXTI0_IRQHandler()
{
	if((buttonSignal = PA0_Read()))
	{
		Led_Ctrl(0,1);
	}
	else Led_Ctrl(0,0);

	//clear interrupt flag (pending register)
	uint32_t *EXTI0_PR = (uint32_t*)(EXTI0_BASE_ADDR + 0x14);
	*EXTI0_PR |= (1<<0);
}

void My_Handler()
{
	if((buttonSignal = PA0_Read()))
		{
			Led_Ctrl(1,1);
		}
		else Led_Ctrl(1,0);

		//clear interrupt flag (pending register)
		uint32_t *EXTI0_PR = (uint32_t*)(EXTI0_BASE_ADDR + 0x14);
		*EXTI0_PR |= (1<<0);
}

void EXTI_Init()
{
	uint32_t *EXTI0_IMR = (uint32_t*)(EXTI0_BASE_ADDR + 0x00);
	*EXTI0_IMR |= (1 << 0);
	uint32_t *EXTI0_RTSR = (uint32_t*)(EXTI0_BASE_ADDR + 0x08);
	*EXTI0_RTSR |= (1 << 0);
	uint32_t *EXTI0_FTSR = (uint32_t*)(EXTI0_BASE_ADDR + 0x0C);
	*EXTI0_FTSR |= (1 << 0);

	uint32_t *NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= (1 << 6);
	//position 6 address là 0x000 0058 -> Flash -> Can't write -> Dời vector table lên RAM

	//Vector table từ 0x0000 0000 - 0x0000 0198 (kích thước: 0x198 ~ 408 byte)
	//RAM có địa chỉ bất đầu 0x2000 0000
	uint8_t* flash = (uint8_t*)0x08000000;
	uint8_t* ram = (uint8_t*)0x20000000;
	for (int i = 0 ; i < 408 ; i++) {
	ram[i] = flash[i];
	}

	//Báo cho NVIC biết là vector table đã được dời lên RAM rồi. Khi có sự kiện ngắt xẩy ra, thì NVIC lên RAM để xem vector table
	uint32_t* VTOR = (uint32_t*)0xE000ED08;
	*VTOR = 0x20000000;

	uint32_t *my_f = (uint32_t*)(0x20000000 + 0x58);
	*my_f = (uint32_t)My_Handler;

	//Noticed: Vector table trên RAM: 0x2000 0000 - 0x2000 0198
	//Trong khi đó buttonSignal được câp phát ở 0x2000 0028
	//-> config Ram để né VTTB: vào file STM32F411VETX_FLASH.ld
}


#define UART1_BASE_ADDR 0x40011000
void dma_init();
void uart_init()

{
	// config PA9 - U1Tx, PA10 - U1Rx
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	 *GPIOA_MODER &= ~(0b1111 << 18);
	*GPIOA_MODER |= (0b10 << 18) | (0b10 << 20);
	uint32_t* GPIOA_AFRH = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*GPIOA_AFRH &= ~(0xFF << 4); // Clear AFRH9, AFRH10
	*GPIOA_AFRH |= (0x07 << 4) | (0x07 << 8); // Set AF7 for PA9 (bits 7:4), PA10 (bits 11:8)

	// uart - BR: 9600 bps, DF: 9 bitdata (include: 1 even parity + 8 bit data)
	__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t* UART1_CR1 = (uint32_t*)(UART1_BASE_ADDR + 0x0C);
	*UART1_CR1 |= (1 << 9) | (1<<10) | (1<<12);
	uint32_t* UART1_BRR = (uint32_t*)(UART1_BASE_ADDR + 0x08);

	*UART1_BRR =  (104 << 4) | 3;
#if 1 // recieve data by Interrupt
	*UART1_CR1 |= (1<<5); 	// enable USART send interrupt signal to NVIC whenever RXNE set
	uint32_t* ISER1 = (uint32_t*)(0xE000E104);
	*ISER1 |= (1<<5);	// NVIC accept interrupt signal

	*UART1_CR1 |= (1<<3) | (1<<2) | (1<< 13);
#else
	uint32_t* UART1_CR3 = (uint32_t*)(UART1_BASE_ADDR + 0x14);
	*UART1_CR3 |= (1 << 6); //bit 7 DMAR
	dma_init();

#endif
}

#define UART1_BASE_ADDR 0x40011000
#define DMA2_BASE_ADDR  0x40026400

char uart_rx_dma_buff[8];
void dma_init()
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	uint32_t* DMA_S5PAR = (uint32_t*)(DMA2_BASE_ADDR + 0x18 + 0x18 * 5); //5:stream
	*DMA_S5PAR = 0x40011004; //địa chỉ thanh ghi DR : uart base + DR offset
	uint32_t* DMA_S5M0AR = (uint32_t*)(DMA2_BASE_ADDR + 0x1C + 0x18 * 5);
	*DMA_S5M0AR = *uart_rx_dma_buff; // địa chỉ người nhận

	uint32_t* DMA_S5NDTR = (uint32_t*)(DMA2_BASE_ADDR + 0x14 + 0x18 * 5);
	*DMA_S5NDTR = sizeof(uart_rx_dma_buff); // 8 bytes

	uint32_t* DMA_S5CR = (uint32_t*)(DMA2_BASE_ADDR + 0x10 + 0x18*5);
	*DMA_S5CR |= (4 << 25) | (1 << 10) | (1<<0); // chanel 4 | bit 10  | enable
}


void uart_send(char data)

{
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE_ADDR +0x04);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE_ADDR +0x00);
	while(((*UART1_SR >> 7) & 1) == 0);
	*UART1_DR =  data;
	while(((*UART1_SR >> 6) & 1) == 0);
}

char uart_recv()
{
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE_ADDR +0x04);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE_ADDR +0x00);
	while(((*UART1_SR >> 5) & 1) == 0);
	char data_recv = *UART1_DR;
	return data_recv;
}

void uart_send_string(char* str)
{
	int str_len = strlen(str);
	for(int i = 0; i< str_len; i++)
	{
		uart_send(str[i]);
	}
}

char rx_buffer[2548];
int rx_index;
void USART1_IRQHandler()
{
	rx_buffer[rx_index] = uart_recv();
	rx_index++;
#if 0
	if(strstr(rx_buffer,"led on") != 0)
	{
		Led_Ctrl(LED_Blue,ON_LED);
		uart_send_string("turned on the red led \n");
		memset(rx_buffer, 0, sizeof(rx_buffer));
		rx_index = 0;
	}
	else if(strstr(rx_buffer,"led off") != 0)
	{
		Led_Ctrl(LED_Blue,OFF_LED);
		uart_send_string("turned off the red led \n");
		memset(rx_buffer, 0, sizeof(rx_buffer));
		rx_index = 0;
	}
#endif
}


#define FLASH_BASE_ADDR 0X40023C00

__attribute__((section(".Ham_tren_Ram"))) void flash_unlock()
{
	volatile uint32_t *FLASH_CR = (uint32_t*)(FLASH_BASE_ADDR + 0x10);
	volatile uint32_t *FLASH_KEYR = (uint32_t*)(FLASH_BASE_ADDR + 0x04);

	// Kiểm tra nếu đang khoá
	if (((*FLASH_CR >> 31) & 1) == 1)
	{
		//write key 1
		*FLASH_KEYR = 0x45678123;
		//write key 2
		*FLASH_KEYR = 0xCDEF89AB;
	}
}
__attribute__((section(".Ham_tren_Ram"))) void flash_erase(int sector_number)
{
	uint32_t *FLASH_SR = (uint32_t *)(FLASH_BASE_ADDR + 0x0C);
	uint32_t *FLASH_CR = (uint32_t *)(FLASH_BASE_ADDR + 0x10);
	flash_unlock();

	while(((*FLASH_SR >> 16) & 1) == 1);
	*FLASH_CR |= 1 << 1;

	*FLASH_CR &= ~(0xF << 3);
	*FLASH_CR |= sector_number << 3;

	*FLASH_CR |= 1 << 16 ;
	while(((*FLASH_SR >> 16) & 1) == 1);
}
__attribute__((section(".Ham_tren_Ram"))) void flash_program(char *address, char* data_buf, int size)
{
	uint32_t *FLASH_SR = (uint32_t *)(FLASH_BASE_ADDR + 0x0C);
	uint32_t *FLASH_CR = (uint32_t *)(FLASH_BASE_ADDR + 0x10);
	flash_unlock();

	while(((*FLASH_SR >> 16) & 1) == 1);
	*FLASH_CR |= 1 << 0;
	for (uint32_t i = 0; i < size; i += 4)
	{
	    uint32_t word = *(uint32_t *)(data_buf + i);
	    // Ghi word vào địa chỉ flash
	    *(volatile uint32_t *)(address + i) = word;
	}
	while(((*FLASH_SR >> 16) & 1) == 1);
}
__attribute__((section(".Ham_tren_Ram"))) void update()
{
	// disable all interrupt
	__asm("CPSID i");
	flash_erase(0);
	flash_program((char*)0x08000000, rx_buffer, 2548);
	//reset system
	uint32_t* AIRCR = (uint32_t*)0xE000ED0C;
	*AIRCR = (0x5FA << 16) | (1<<2);
}

//
#define I2C_Base_Addr 0x40005400
uint32_t* I2C_CR1 = (uint32_t*)(I2C_Base_Addr + 0x00);
uint32_t* I2C_CR2 = (uint32_t*)(I2C_Base_Addr + 0x04);
void I2C_Init()
{

 //config PB6 - I2C1_SCL, PB9 - I2C1_SDA
 __HAL_RCC_GPIOB_CLK_ENABLE();
 uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
 *GPIOB_MODER &= ~((0b11 << 12)|(0b11 << 18));
 *GPIOB_MODER |= (0b10 << 12) | (0b10 << 18);

 uint32_t* GPIOB_AFRL =  (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
 uint32_t* GPIOB_AFRH =  (uint32_t*)(GPIOB_BASE_ADDR + 0x24);
 *GPIOB_AFRL &= ~(0b1111 << 24);
 *GPIOB_AFRL |= (0b0100 <<  24);
 *GPIOB_AFRH &= ~(0b1111 << 4);
 *GPIOB_AFRH |= (0b0100 <<  4);

 __HAL_RCC_I2C1_CLK_ENABLE();
 *I2C_CR2 &= ~(0b11111 << 0); //config freq - bit[5:0]
 *I2C_CR2 |= (16 << 0);

 uint32_t* I2C_CCR = (uint32_t*)(I2C_Base_Addr + 0x1C);
 *I2C_CCR &= ~(0xfff << 0); //config prescaler - bit[11:0]
 *I2C_CCR |= (40 << 0);

 //enable
 *I2C_CR1 |= (1 << 0);
}

void I2C_Write(uint8_t slave_addr, uint8_t reg_addr, uint8_t value)
{
    uint32_t* I2C_CR1 = (uint32_t*)(I2C_Base_Addr + 0x00);
    uint32_t* I2C_SR1 = (uint32_t*)(I2C_Base_Addr + 0x14);
    uint32_t* I2C_SR2 = (uint32_t*)(I2C_Base_Addr + 0x18);
    uint32_t* I2C_DR  = (uint32_t*)(I2C_Base_Addr + 0x10);

    *I2C_CR1 |= (1 << 8); // START
    while (!(*I2C_SR1 & (1 << 0))) {}

    *I2C_DR = (slave_addr << 1) | 0;
    while (!(*I2C_SR1 & (1 << 1))) {}
    uint32_t temp = *I2C_SR1;
    temp = *I2C_SR2;
    (void)temp;
    if (*I2C_SR1 & (1 << 10)) {
        *I2C_SR1 &= ~(1 << 10);
        *I2C_CR1 |= (1 << 9);
        return;
    }

    while (!(*I2C_SR1 & (1 << 7))) {} // TXE
    *I2C_DR = reg_addr;

    while (!(*I2C_SR1 & (1 << 7))) {} // TXE
    *I2C_DR = value;

    while (!(*I2C_SR1 & (1 << 2))) {} // BTF

    *I2C_CR1 |= (1 << 9); // STOP
}
uint32_t I2C_Read(uint32_t slave_addr, uint32_t slave_register_addr)
{
    #define I2C_Base_Addr 0x40005400
    uint32_t* I2C_CR1 = (uint32_t*)(I2C_Base_Addr + 0x00);
    uint32_t* I2C_SR1 = (uint32_t*)(I2C_Base_Addr + 0x14);
    uint32_t* I2C_SR2 = (uint32_t*)(I2C_Base_Addr + 0x18);
    uint32_t* I2C_DR  = (uint32_t*)(I2C_Base_Addr + 0x10);

    uint32_t data;

    // Step 1: Start
    *I2C_CR1 |= (1 << 8);
    while (!(*I2C_SR1 & (1 << 0))) {}

    // Step 2: gửi địa chỉ slave + write
    *I2C_DR = (slave_addr << 1) | 0;
    while (!(*I2C_SR1 & (1 << 1))) {}

    // Step 3: Clear ADDR
    uint32_t temp = *I2C_SR1;
    temp = *I2C_SR2;

    // Step 4: kiểm tra AF
    if (*I2C_SR1 & (1 << 10)) {
        *I2C_SR1 &= ~(1 << 10);
        *I2C_CR1 |= (1 << 9);
        return 0;
    }

    // Step 5: gửi địa chỉ thanh ghi
    *I2C_DR = slave_register_addr;
    while (!(*I2C_SR1 & (1 << 2))) {} // BTF

    // Step 6: gửi lại START (repeated start)
    *I2C_CR1 |= (1 << 8);
    while (!(*I2C_SR1 & (1 << 0))) {}

    // Step 7: gửi địa chỉ slave + READ
    *I2C_DR = (slave_addr << 1) | 1;
    while (!(*I2C_SR1 & (1 << 1))) {}

    // Step 8: Clear ADDR
    temp = *I2C_SR1;
 temp = *I2C_SR2;

    // Step 9: Tắt ACK (chuẩn bị nhận byte cuối)
    *I2C_CR1 &= ~(1 << 10); // ACK = 0

    // Step 10: chờ dữ liệu về (RXNE)
    while (!(*I2C_SR1 & (1 << 6))) {}

    // Step 11: gửi STOP
    *I2C_CR1 |= (1 << 9);

    // Step 12: đọc dữ liệu
    data = (uint8_t)(*I2C_DR);

    (void)temp;
    return data;
}

#define SPI1_BASE_ADDR 0x40013000
void SPI_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    //Config chân, PA5 SCLK, PA7 SPI1_MOSI, PA6 SPI1_MISO, PE3 CS_I2C

    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    uint32_t* GPIOA_AFRL =  (uint32_t*)(GPIOA_BASE_ADDR + 0x20);

    // AF cònig
    *GPIOA_MODER &= ~((0b11 << 10) | (0b11 << 12) | (0b11 << 14));
    *GPIOA_MODER |=  ((0b10 << 10) | (0b10 << 12) | (0b10 << 14));

    // AF05 cho 3 pin A5,6,7
    *GPIOA_AFRL &= ~((0xF << 20) | (0xF << 24) | (0xF << 28));
    *GPIOA_AFRL |=  ((0x5 << 20) | (0x5 << 24) | (0x5 << 28));

    //PE3 CS_I2C
    uint32_t* GPIOE_MODER = (uint32_t*)(GPIOE_BASE_ADDR + 0x00);
    *GPIOE_MODER &= ~(0b11 << 6);
    *GPIOE_MODER |= (0b01 << 6);

    uint16_t* SPI_CR1 = (uint16_t*)(SPI1_BASE_ADDR + 0x00);
    // Master mode
    *SPI_CR1 |= (1 << 2);

    // Set BR = 0b100
    *SPI_CR1 |=  (0b100 << 3);

    //Set bit SSI và SSM = 1
    *SPI_CR1 |= (1 << 8);
    *SPI_CR1 |= (1 << 9);

    // SPI enable
    *SPI_CR1 |= (1 << 6);
}

uint8_t SPI_Read_Reg(uint8_t reg_addr)
{
    // Địa chỉ thanh ghi
    volatile uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
    volatile uint16_t* SPI1_DR   = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
    volatile uint16_t* SPI1_SR   = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

    uint8_t dummy = 0x00;
    uint8_t data  = 0;

    // 1. Kéo chân SS (PE3) xuống mức thấp (0) để chọn Slave
    *GPIOE_ODR &= ~(1 << 3);

    // 2. Gửi địa chỉ OR 0x80 (bit 7 = 1 là đọc)
    while(!(*SPI1_SR & (1 << 1))); // TXE: Data register empty
    *SPI1_DR = reg_addr | 0x80;

    // 3. Đợi nhận xong, đọc dữ liệu rác
    while(!(*SPI1_SR & (1 << 0))); // RXNE: Data received
    (void)*SPI1_DR; // Đọc bỏ byte rác

    // 4. Gửi dummy để tạo xung clock
    while(!(*SPI1_SR & (1 << 1)));
    *SPI1_DR = dummy;

    // 5. Đợi nhận xong, lấy giá trị thực
    while(!(*SPI1_SR & (1 << 0)));
    data = *SPI1_DR;

    // 6. Kéo chân SS (PE3) lên mức cao (1) để kết thúc
    *GPIOE_ODR |= (1 << 3);

    return data;
}

void SPI_Write_Reg(uint8_t reg_addr, uint8_t value)
{
    volatile uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
    volatile uint16_t* SPI1_DR   = (uint16_t*)(SPI1_BASE_ADDR + 0x0C);
    volatile uint16_t* SPI1_SR   = (uint16_t*)(SPI1_BASE_ADDR + 0x08);

    // 1. Kéo chân SS (PE3) xuống mức thấp (0)
    *GPIOE_ODR &= ~(1 << 3);

    // 2. Gửi địa chỉ OR 0x00 (bit 7 = 0: write), ghi vào DR
    while(!(*SPI1_SR & (1 << 1))); // TXE: Data register empty
    *SPI1_DR = reg_addr | 0x00;

    // 3. Đọc dữ liệu rác từ DR
    while(!(*SPI1_SR & (1 << 0))); // RXNE: Data received
    (void)*SPI1_DR;

    // 4. Gửi giá trị muốn ghi, ghi vào DR
    while(!(*SPI1_SR & (1 << 1)));
    *SPI1_DR = value;

    // 5. Đọc dữ liệu rác từ DR
    while(!(*SPI1_SR & (1 << 0)));
    (void)*SPI1_DR;

    // 6. Kéo chân SS (PE3) lên mức cao (1)
    *GPIOE_ODR |= (1 << 3);
}

#define RCC_Base_Addr 0x42003800
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
	//N là bit [6:14] - chia 200
	*RCC_PLLCFGR &= ~(0x1ff << 6);
	*RCC_PLLCFGR |= (200 <<  6);
	//P là bit [16:17] - chia 2
	*RCC_PLLCFGR &= ~(0b11 << 16);
	*RCC_PLLCFGR |= (2 << 16);

	*RCC_CR |= (1 <<  24);
	while(!(*RCC_CR & (1 << 25)));

	*RCC_CFGR &= ~(0b1111 << 0);
	*RCC_CFGR |= (2 << 0);

}

#define LSM303_ACC_ADDR 0x19
uint32_t data_i2c;

int main()
{
	HAL_Init();
	Leds_Init();
	PA0_input_Init();
	EXTI_Init();
	uart_init();
	I2C_Init();
	SPI_Init();
	uart_send_string("I'm Jin \r\n");
	uart_send_string("START PROGRAM Blue_v.1.0 \r\n");
	I2C_Write(LSM303_ACC_ADDR, 0x20, 0x12);
	data_i2c = I2C_Read(LSM303_ACC_ADDR, 0x20);
	uint8_t id = SPI_Read_Reg(0x0F);
#if 0
	char msg[] = "Helloo \r\n";
	for(int i = 0; i < sizeof(msg); i++)
		{
			uart_send(msg[i]);
		}
#endif

	while(1)
		{
			Led_Ctrl(LED_Blue,ON_LED);
			uart_send_string("I'm Jin \r\n");
			HAL_Delay(1000);
			Led_Ctrl(LED_Blue,OFF_LED);
			HAL_Delay(1000);
			if(rx_index == sizeof(rx_buffer))
			{
				uart_send_string("FW_RED_v1.0 recieving done !!! \r\n");
				update();
			}


		}
}
