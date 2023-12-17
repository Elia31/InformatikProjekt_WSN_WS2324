#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

int main(void)
{
	int counter = 0;

	while (1)
	{
		printk("Counter: %d\n", counter);
        counter++;
        k_msleep(SLEEP_TIME_MS);
	}
}
