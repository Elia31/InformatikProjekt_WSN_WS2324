#include <zephyr/kernel.h> 
#include <zephyr/sys/printk.h> 
#include <zephyr/drivers/i2c.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define MCP9808_I2C_ADDRESS             0x18 
#define MCP9808_TEMPERATURE_REGISTER    0x05 

#define I2C_NODE        DT_NODELABEL(i2c0)          //DT_N_S_soc_S_i2c_40003000 
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE); 
static uint8_t i2c_buffer[2]; 

float mcp9808_calculateTemperature(uint8_t upperByte, uint8_t lowerByte)
{ 
    float temperature = (upperByte&0x0F)*16+(float)(lowerByte)/16; 
    if ((upperByte & 0x10) == 0x10) //Temperatur < 0°C 
    {
        temperature=256-temperature; //von 2^8 subtrahieren, da Zweierkompliment 
    }
    return temperature; 
} 

int main(void)
{ 
    int err; 
    if (!device_is_ready(i2c_dev)) 
    { 
        printk("i2c_dev not ready\n"); return 1; 
    }    
    while (true) 
    { 
        i2c_buffer[0]=MCP9808_TEMPERATURE_REGISTER; 
        do
        { 
            err = i2c_write(i2c_dev, i2c_buffer, 1, MCP9808_I2C_ADDRESS); 
            if (err < 0)
            { 
                printk("MCP9808 write failed: %d\n", err); break; 
            } 
            err = i2c_read(i2c_dev,i2c_buffer, 2, MCP9808_I2C_ADDRESS); 
            if (err < 0)
            {
                 printk("MCP9808 read failed: %d\n", err); break; 
            } 
            float temperature = mcp9808_calculateTemperature(i2c_buffer[0],i2c_buffer[1]); 
            printk("MCP9808: %.2f Cel \n", temperature); 
 
        } while (false); 
        k_msleep(SLEEP_TIME_MS); 
    } 
}
