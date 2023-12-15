/*
 * Copyright (C) 2023 Lars Boegild Thomsen <lbthomsen@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define DATA_BPS 2000000

/*
 * To implement the STDIO functions you need to create
 * the _read and _write functions and hook them to the
 * USART you are using. This example also has a buffered
 * read function for basic line editing.
 */
int _write(int fd, char *ptr, int len);

uint32_t now = 0;
uint32_t systick = 0;
uint32_t last_tick = 0;
uint32_t last_blink = 0;

void sys_tick_handler(void)
{
    ++systick;
}

static void clock_setup(void)
{

    rcc_clock_setup_pll(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    /* 168MHz / 8 => 21000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); 

    /* 21000000/21000 = 1000 overflows per second - every 1ms one interrupt */
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(20999);

    systick_interrupt_enable();

    /* Start counting. */
    systick_counter_enable();

    /* Enable GPIOD clock for LED & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for USART1. */
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);

    rcc_periph_clock_enable(RCC_DMA1);
}

static void usart_setup(void)
{
    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 921600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);

        /* Setup USART2 parameters. */
    usart_set_baudrate(USART2, DATA_BPS);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);

        /* Setup USART3 parameters. */
    usart_set_baudrate(USART3, DATA_BPS);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART3);
}

static void gpio_setup(void)
{
    /* Setup GPIO pin GPIO13 on GPIO port C for LED. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

    /* Setup GPIO pins for USART1. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

    /* Setup USART1 pins as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /* Setup GPIO pins for USART2. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);

    /* Setup USART2 pins as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    /* Setup GPIO pins for USART3. */
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);

    /* Setup USART3 pins as alternate function. */
    gpio_set_af(GPIOC, GPIO_AF7, GPIO10 | GPIO11);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();

    while (1) {

        now = systick;

        if (now - last_blink >= 500)
        {
            gpio_toggle(GPIOC, GPIO13); /* LED on/off */
            last_blink = now;
        }

        if (now - last_tick >= 1000)
        {
            printf("Tick %lu\n", now);
            last_tick = now;
        }

    
    }

    return 0;
}

/*
 * Called by libc stdio fwrite functions
 */
int _write(int fd, char *ptr, int len)
{
    int i = 0;

    /*
     * Write "len" of char from "ptr" to file id "fd"
     * Return number of char written.
     *
     * Only work for STDOUT, STDIN, and STDERR
     */
    if (fd > 2)
    {
        return -1;
    }
    while (*ptr && (i < len))
    {
        usart_send_blocking(USART1, *ptr);
        if (*ptr == '\n')
        {
            usart_send_blocking(USART1, '\r');
        }
        i++;
        ptr++;
    }
    return i;
}

