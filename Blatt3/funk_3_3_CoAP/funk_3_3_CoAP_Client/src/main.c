#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>
#include <openthread/udp.h>
#include <openthread/coap.h>

#define SLEEP_TIME_MS 1000

#define BUTTON0_NODE DT_NODELABEL(button0) // DT_N_S_buttons_S_button_0
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_cb;

void button_pressed_cb(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins);
static void coap_send_data_request(void);
static void coap_send_data_response_cb(void *p_context, otMessage *p_message,
                                       const otMessageInfo *p_message_info, otError result);

void button_pressed_cb(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    coap_send_data_request();
}

#define TEXTBUFFER_SIZE 30
char myText[TEXTBUFFER_SIZE];
uint16_t myText_length = 0;

static void coap_send_data_request(void)
{
    otError error = OT_ERROR_NONE;
    otMessage *myMessage;
    otMessageInfo myMessageInfo;
    otInstance *myInstance = openthread_get_default_instance();
    const otMeshLocalPrefix *ml_prefix = otThreadGetMeshLocalPrefix(myInstance);
    uint8_t serverInterfaceID[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
    const char *myTemperatureJson = "{\"temperature\": 23.32}";

    do
    {
        myMessage = otCoapNewMessage(myInstance, NULL);
        if (myMessage == NULL)
        {
            printk("Failed to allocate message for CoAP Request\n");
            return;
        }

        otCoapMessageInit(myMessage, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);

        error = otCoapMessageAppendUriPathOptions(myMessage, "storedata");
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapMessageAppendContentFormatOption(myMessage, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapMessageSetPayloadMarker(myMessage);
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otMessageAppend(myMessage, myTemperatureJson, strlen(myTemperatureJson));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&myMessageInfo, 0, sizeof(myMessageInfo));
        memcpy(&myMessageInfo.mPeerAddr.mFields.m8[0], ml_prefix, 8);
        memcpy(&myMessageInfo.mPeerAddr.mFields.m8[8], serverInterfaceID, 8);
        myMessageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;

        error = otCoapSendRequest(myInstance, myMessage, &myMessageInfo, coap_send_data_response_cb, NULL);
    } while (false);

    if (error != OT_ERROR_NONE)
    {
        printk("Failed to send CoAP Request: %d\n", error);
        otMessageFree(myMessage);
    }
    else
    {
        printk("CoAP data send.\n");
    }
}

static void coap_send_data_response_cb(void *p_context, otMessage *p_message,
                                       const otMessageInfo *p_message_info, otError result)
{
    if (result == OT_ERROR_NONE)
    {
        printk("Delivery confirmed.\n");
    }
    else
    {
        printk("Delivery not confirmed: %d\n", result);
    }
}

void coap_init(void)
{
    otInstance *p_instance = openthread_get_default_instance();
    otError error = otCoapStart(p_instance, OT_DEFAULT_COAP_PORT);
    if (error != OT_ERROR_NONE)
        printk("Failed to start Coap: %d\n", error);
}

int main(void)
{
    coap_init();
    gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button0_cb, button_pressed_cb, BIT(button0_spec.pin));
    gpio_add_callback(button0_spec.port, &button0_cb);
    while (1)
    {
        k_msleep(SLEEP_TIME_MS);
    }
}
