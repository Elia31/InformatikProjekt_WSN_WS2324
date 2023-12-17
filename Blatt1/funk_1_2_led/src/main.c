#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* define the pins we use */
#define LED1_PIN NRF_GPIO_PIN_MAP(0, 13)
#define LED2_PIN NRF_GPIO_PIN_MAP(0, 14)
#define LED3_PIN NRF_GPIO_PIN_MAP(0, 15)
#define LED4_PIN NRF_GPIO_PIN_MAP(0, 16)


int main(void)
{
	/* configure the pins*/
	nrf_gpio_cfg_output(LED1_PIN);
    nrf_gpio_cfg_output(LED2_PIN);
    nrf_gpio_cfg_output(LED3_PIN);
    nrf_gpio_cfg_output(LED4_PIN);

	/* initalize all pins to be off (if they happen to be on)*/
	nrf_gpio_pin_clear(LED1_PIN);
	nrf_gpio_pin_clear(LED2_PIN);
	nrf_gpio_pin_clear(LED3_PIN);
    nrf_gpio_pin_clear(LED4_PIN);

	while (1)
	{
		/* leds turned on in order */
		nrf_gpio_pin_set(LED1_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_set(LED2_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_set(LED3_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_set(LED4_PIN);
		k_msleep(SLEEP_TIME_MS);

		/* leds turned off in order */
		nrf_gpio_pin_clear(LED1_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_clear(LED2_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_clear(LED3_PIN);
		k_msleep(SLEEP_TIME_MS);
		nrf_gpio_pin_clear(LED4_PIN);
		k_msleep(SLEEP_TIME_MS);

	}

}
