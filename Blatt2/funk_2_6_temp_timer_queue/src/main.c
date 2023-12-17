#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LED0_NODE   DT_NODELABEL(led0) //DT_N_S_leds_S_led_0
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

const struct device *mcp9808_dev = DEVICE_DT_GET_ANY(microchip_mcp9808);
static struct sensor_value temp;

void mytimer_cb(struct k_timer *dummy)
{
    gpio_pin_toggle_dt(&led0_spec);
    k_work_submit(&mcp9808_work);
}
K_TIMER_DEFINE(mytimer, mytimer_cb, NULL);

static void mcp9808_work_cb(struct k_work *work)
{
    int err;
    if (!device_is_ready(mcp9808_dev))
    {
        printk("mcp9808_dev not ready\n");
        return;
    }
    do
    {
        err = sensor_sample_fetch(mcp9808_dev);
        if (err < 0)
        {
            printk("MCP9808 write failed: %d\n", err);
            break;
        }

        err = sensor_channel_get(mcp9808_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (err < 0)
        {
            printk("MCP9808 read failed: %d\n", err);
            break;
        }

        printk("MCP9808: %.2f Cel\n", sensor_value_to_double(&temp));
    } while (false);
}
K_WORK_DEFINE(mcp9808_work, mcp9808_work_cb);



int main(void)
{
    gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT);
    k_timer_start(&mytimer, K_SECONDS(2), K_SECONDS(2));
    while (true)
    {
        k_msleep(SLEEP_TIME_MS);
    }
}