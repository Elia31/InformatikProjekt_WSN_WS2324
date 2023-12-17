#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
#define BUTTON0_NODE DT_NODELABEL(button0)
#define BUTTON1_NODE DT_NODELABEL(button1)

static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static const struct gpio_dt_spec button1_spec = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

static struct gpio_callback button0_cb;
static struct gpio_callback button1_cb;

void button0_pressed_callback(const struct device *gpiob, struct gpio_callback *cb,
                              gpio_port_pins_t pins) {
    gpio_pin_toggle_dt(&led0_spec);
}

void button1_pressed_callback(const struct device *gpiob, struct gpio_callback *cb,
                              gpio_port_pins_t pins) {
    gpio_pin_toggle_dt(&led1_spec);
}

int main(void) 
{
    gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
    gpio_pin_configure_dt(&button1_spec, GPIO_INPUT);
    
    gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&button1_spec, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button0_cb, button0_pressed_callback, BIT(button0_spec.pin));
    gpio_add_callback(button0_spec.port, &button0_cb);

    gpio_init_callback(&button1_cb, button1_pressed_callback, BIT(button1_spec.pin));
    gpio_add_callback(button1_spec.port, &button1_cb);

    while (1) 
    {
        k_msleep(SLEEP_TIME_MS);
    }
}
