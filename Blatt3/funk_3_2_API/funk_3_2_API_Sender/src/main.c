#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>
#include <openthread/udp.h>

#define SLEEP_TIME_MS 1000
#define BUTTON0_NODE DT_NODELABEL(button0) // DT_N_S_buttons_S_button_0
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_cb;

static void udp_send(void)
{
    otError error = OT_ERROR_NONE;
    const char *buf = "Hello Thread";

    otInstance *myInstance;
    myInstance = openthread_get_default_instance();
    otUdpSocket mySocket;

    otMessageInfo messageInfo;
    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString("ff03::1", &messageInfo.mPeerAddr);

    messageInfo.mPeerPort = 1234;

    do
    {
        error = otUdpOpen(myInstance, &mySocket, NULL, NULL);
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        otMessage *test_Message = otUdpNewMessage(myInstance, NULL);
        error = otMessageAppend(test_Message, buf, (uint16_t)strlen(buf));
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        error = otUdpSend(myInstance, &mySocket, test_Message, &messageInfo);
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        error = otUdpClose(myInstance, &mySocket);
    } while (false);

    if (error == OT_ERROR_NONE)
    {
        printk("Send.\n");
    }
    else
    {
        printk("udpSend error: %d\n", error);
    }
}

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb,
                             gpio_port_pins_t pins)
{
    udp_send();
}

int main(void)
{
    gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button0_cb, button_pressed_callback, BIT(button0_spec.pin));
    gpio_add_callback(button0_spec.port, &button0_cb);
    while (1)
    {
        k_msleep(SLEEP_TIME_MS);
    }
}
