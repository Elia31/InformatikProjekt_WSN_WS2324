#include <zephyr/kernel.h> 
#include <zephyr/sys/printk.h> 
#include <zephyr/drivers/sensor.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

const struct device *mcp9808_dev = DEVICE_DT_GET_ANY(microchip_mcp9808); 
static struct sensor_value temp;
 
int main(void){ 
    if (!device_is_ready(mcp9808_dev)) 
    { 
        printk("mcp9808_dev not ready\n"); return 1; 
    }    
    int err; 
    while (true) 
    { 
        do
        { 
            err = sensor_sample_fetch(mcp9808_dev); 
            if (err < 0)
            { 
                printk("MCP9808 write failed: %d\n", err); break; 
            } 
            err = sensor_channel_get(mcp9808_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp); 
            if (err < 0)
            {
                printk("MCP9808 read failed: %d\n", err); break; 
            } 
            printk("MCP9808: %.2f Cel\n", sensor_value_to_double(&temp)); 
        }while (false); 
        k_msleep(SLEEP_TIME_MS);
    } 
} 
