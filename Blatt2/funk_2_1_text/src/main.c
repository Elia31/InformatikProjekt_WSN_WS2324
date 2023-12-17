#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define UART1_NODE DT_NODELABEL(uart1) //DT_N_S_soc_S_uart_40028000
static const struct device *uart_dev = DEVICE_DT_GET(UART1_NODE);

static uint8_t tx_buf[15];
static int tx_buf_length;
static int counter=0;

void uart1_cb(const struct device *dev,  struct uart_event *evt, void *user_data)
{ 
    switch (evt->type) 
	{ 
        default: 
            break; 
    } 
} 

int main(void)
{ 
    if (!device_is_ready(uart_dev)) 
    { 
        printk("uart_dev not ready\n"); 
        return; 
    } 
    uart_callback_set(uart_dev, uart1_cb, NULL); 
    while (1) 
    { 
        k_msleep(SLEEP_TIME_MS); 
        tx_buf_length= sprintf(tx_buf, "Counter: %d\n\r", counter); 
        uart_tx(uart_dev, tx_buf , tx_buf_length, 100 * USEC_PER_MSEC); 
        counter++; 
    } 
}