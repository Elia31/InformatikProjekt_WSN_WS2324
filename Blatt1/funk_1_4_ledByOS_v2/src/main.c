#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LED0_NODE       DT_NODELABEL(led0)
#define LED1_NODE       DT_NODELABEL(led1)
#define LED2_NODE       DT_NODELABEL(led2)
#define LED3_NODE       DT_NODELABEL(led3)

static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2_spec = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3_spec = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

void main(void) 
{
	gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led2_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led3_spec, GPIO_OUTPUT);

    while (1)
	{
        gpio_pin_set_dt(&led0_spec, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led1_spec, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led2_spec, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led3_spec, 1);
        k_msleep(SLEEP_TIME_MS);

        gpio_pin_set_dt(&led0_spec, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led1_spec, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led2_spec, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set_dt(&led3_spec, 0);
        k_msleep(SLEEP_TIME_MS);
    }
}
