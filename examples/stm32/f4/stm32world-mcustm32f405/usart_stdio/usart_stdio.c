/*
 * Copyright (C) 2023 Lars Boegild Thomsen <lbthomsen@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/*
 * To implement the STDIO functions you need to create
 * the _read and _write functions and hook them to the
 * USART you are using. This example also has a buffered
 * read function for basic line editing.
 */
int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);
void get_buffered_line(void);

/*
 * This is a pretty classic ring buffer for characters
 */
#define BUFLEN 127

uint32_t systick = 0;
uint32_t last_tick = 0;
uint32_t last_blink = 0;
static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN + 1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)
static inline int inc_ndx(int n)
{
    return ((n + 1) % BUFLEN);
}
static inline int dec_ndx(int n) { return (((n + BUFLEN) - 1) % BUFLEN); }

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
}

static void gpio_setup(void)
{
    /* Setup GPIO pin GPIO12 on GPIO port D for LED. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();

    /* Blink the LED (PD12) on the board with every transmitted byte. */
    while (1)
    {

        uint32_t now = systick;

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

/* back up the cursor one space */
static inline void back_up(void)
{
    end_ndx = dec_ndx(end_ndx);
    usart_send_blocking(USART1, '\010');
    usart_send_blocking(USART1, ' ');
    usart_send_blocking(USART1, '\010');
}

/*
 * A buffered line editing function.
 */
void get_buffered_line(void)
{
    char c;

    if (start_ndx != end_ndx)
    {
        return;
    }
    while (1)
    {
        c = usart_recv_blocking(USART1);
        if (c == '\r')
        {
            buf[end_ndx] = '\n';
            end_ndx = inc_ndx(end_ndx);
            buf[end_ndx] = '\0';
            usart_send_blocking(USART1, '\r');
            usart_send_blocking(USART1, '\n');
            return;
        }
        /* ^H or DEL erase a character */
        if ((c == '\010') || (c == '\177'))
        {
            if (buf_len == 0)
            {
                usart_send_blocking(USART1, '\a');
            }
            else
            {
                back_up();
            }
            /* ^W erases a word */
        }
        else if (c == 0x17)
        {
            while ((buf_len > 0) &&
                   (!(isspace((int)buf[end_ndx]))))
            {
                back_up();
            }
            /* ^U erases the line */
        }
        else if (c == 0x15)
        {
            while (buf_len > 0)
            {
                back_up();
            }
            /* Non-editing character so insert it */
        }
        else
        {
            if (buf_len == (BUFLEN - 1))
            {
                usart_send_blocking(USART1, '\a');
            }
            else
            {
                buf[end_ndx] = c;
                end_ndx = inc_ndx(end_ndx);
                usart_send_blocking(USART1, c);
            }
        }
    }
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

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int _read(int fd, char *ptr, int len)
{
    int my_len;

    if (fd > 2)
    {
        return -1;
    }

    get_buffered_line();
    my_len = 0;
    while ((buf_len > 0) && (len > 0))
    {
        *ptr++ = buf[start_ndx];
        start_ndx = inc_ndx(start_ndx);
        my_len++;
        len--;
    }
    return my_len; /* return the length we got */
}
