#include <zephyr/kernel.h> 
#include <zephyr/drivers/adc.h> 
#include <zephyr/sys/printk.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define ADC_NODE        DT_NODELABEL(adc)  //DT_N_S_soc_S_adc_40007000 
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE); 

#define ADC_RESOLUTION      10 
#define ADC_CHANNEL         0 
#define ADC_PORT            SAADC_CH_PSELP_PSELP_AnalogInput0   //AIN0 
#define ADC_REFERENCE       ADC_REF_INTERNAL                    //0.6V 
#define ADC_GAIN            ADC_GAIN_1_5                        //ADC_REFERENCE*5

struct adc_channel_cfg channel_cfg = 
{ 
    .gain = ADC_GAIN, 
    .reference = ADC_REFERENCE, 
    .acquisition_time = ADC_ACQ_TIME_DEFAULT, 
    .channel_id = ADC_CHANNEL, 
#ifdef CONFIG_ADC_NRFX_SAADC 
    .input_positive = ADC_PORT 
#endif 
}; 

int16_t sample_buffer[1]; 
struct adc_sequence sequence = 
{ 
    .channels    = BIT(ADC_CHANNEL), 
    .buffer      = sample_buffer, 
    .buffer_size = sizeof(sample_buffer), 
    .resolution  = ADC_RESOLUTION 
};

int main(void)
{ 
    if (!device_is_ready(adc_dev)) 
    { 
        printk("adc_dev not ready\n"); return 1; 
    }    
    int err; 
    err= adc_channel_setup(adc_dev, &channel_cfg); 
    if (err != 0) 
    { 
        printk("ADC adc_channel_setup failed with error %d.\n", err); return 1; 
    }

    while (1) 
    { 
        err = adc_read(adc_dev, &sequence); 
        if (err != 0) 
        { 
            printk("ADC reading failed with error %d.\n", err); return 1; 
        } 
        int32_t mv_value = sample_buffer[0]; 
        int32_t adc_vref = adc_ref_internal(adc_dev);
        adc_raw_to_millivolts(adc_vref, ADC_GAIN,   ADC_RESOLUTION, &mv_value); 
        printk("ADC-Spannung: %d mV  \n", mv_value);
        k_msleep(SLEEP_TIME_MS); 
    } 
}


