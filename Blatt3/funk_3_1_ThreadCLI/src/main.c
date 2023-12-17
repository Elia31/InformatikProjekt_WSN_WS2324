#include <zephyr/kernel.h> 
#include <zephyr/drivers/gpio.h> 
#include <zephyr/sys/printk.h>
 
#define SLEEP_TIME_MS   1000

int main(void)
{ 
    while (1) 
	{ 
        k_msleep(SLEEP_TIME_MS); 
    } 
}
