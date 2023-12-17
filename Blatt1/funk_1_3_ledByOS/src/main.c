#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LED0_NODE       DT_NODELABEL(led0)
#define LED1_NODE       DT_NODELABEL(led1)
#define LED2_NODE       DT_NODELABEL(led2)
#define LED3_NODE       DT_NODELABEL(led3)

#define LED0_GPIO_NODE  DT_PHANDLE_BY_IDX(LED0_NODE, gpios, 0)
#define LED1_GPIO_NODE  DT_PHANDLE_BY_IDX(LED1_NODE, gpios, 0)
#define LED2_GPIO_NODE  DT_PHANDLE_BY_IDX(LED2_NODE, gpios, 0)
#define LED3_GPIO_NODE  DT_PHANDLE_BY_IDX(LED3_NODE, gpios, 0)

#define LED0_GPIO_NAME  DEVICE_DT_NAME(LED0_GPIO_NODE)
#define LED1_GPIO_NAME  DEVICE_DT_NAME(LED1_GPIO_NODE)
#define LED2_GPIO_NAME  DEVICE_DT_NAME(LED2_GPIO_NODE)
#define LED3_GPIO_NAME  DEVICE_DT_NAME(LED3_GPIO_NODE)

#define LED0_PIN        DT_GPIO_PIN(LED0_NODE, gpios)
#define LED1_PIN        DT_GPIO_PIN(LED1_NODE, gpios)
#define LED2_PIN        DT_GPIO_PIN(LED2_NODE, gpios)
#define LED3_PIN        DT_GPIO_PIN(LED3_NODE, gpios)

#define LED0_FLAG       DT_GPIO_FLAGS(LED0_NODE, gpios)
#define LED1_FLAG       DT_GPIO_FLAGS(LED1_NODE, gpios)
#define LED2_FLAG       DT_GPIO_FLAGS(LED2_NODE, gpios)
#define LED3_FLAG       DT_GPIO_FLAGS(LED3_NODE, gpios)

void main(void) 
{
    const struct device *led0_dev = device_get_binding(LED0_GPIO_NAME);
    const struct device *led1_dev = device_get_binding(LED1_GPIO_NAME);
    const struct device *led2_dev = device_get_binding(LED2_GPIO_NAME);
    const struct device *led3_dev = device_get_binding(LED3_GPIO_NAME);

    gpio_pin_configure(led0_dev, LED0_PIN, GPIO_OUTPUT | LED0_FLAG);
    gpio_pin_configure(led1_dev, LED1_PIN, GPIO_OUTPUT | LED1_FLAG);
    gpio_pin_configure(led2_dev, LED2_PIN, GPIO_OUTPUT | LED2_FLAG);
    gpio_pin_configure(led3_dev, LED3_PIN, GPIO_OUTPUT | LED3_FLAG);

    while (1) 
    {
        gpio_pin_set(led0_dev, LED0_PIN, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led1_dev, LED1_PIN, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led2_dev, LED2_PIN, 1);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led3_dev, LED3_PIN, 1);
        k_msleep(SLEEP_TIME_MS);

        gpio_pin_set(led0_dev, LED0_PIN, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led1_dev, LED1_PIN, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led2_dev, LED2_PIN, 0);
        k_msleep(SLEEP_TIME_MS);
        gpio_pin_set(led3_dev, LED3_PIN, 0);
        k_msleep(SLEEP_TIME_MS);
    }
}
