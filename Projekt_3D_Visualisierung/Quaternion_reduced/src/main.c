#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/smf.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>
#include <openthread/udp.h>

#define I2C_NODE		DT_NODELABEL(i2c0)	//DT_N_S_soc_i2c_40003000
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

//* BNO055 Address A (Pin17 LOW) **/
#define BNO055_ADDRESS_A        (0x28)

/*Operationsmodus*/
#define OPERATION_MODE_CONFIG   (0x00)
#define OPERATION_MODE_NDOF     (0x0C)

/*Registeraddressen*/
#define BNO055_CHIP_ID_ADDR                     (0x00)
#define BNO055_PAGE_ID_ADDR                     (0x07)
#define BNO055_QUATERNION_DATA_W_LSB_ADDR       (0x20) // Datasheet (quat daten w)
#define BNO055_CALIB_STAT_ADDR                  (0x35)
#define BNO055_OPR_MODE_ADDR                    (0x3D)
#define BNO055_SYS_TRIGGER_ADDR                 (0x3F)

#define READ_SENSOR_INTERVALL       20
#define ERROR_SLEEP                 5000

/* LEDS for Calibration*/
#define LED0_NODE	DT_NODELABEL(led0)
#define LED1_NODE	DT_NODELABEL(led1)
#define LED2_NODE	DT_NODELABEL(led2)
#define LED3_NODE	DT_NODELABEL(led3)
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2_spec = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3_spec = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* Button for Calibration */
#define BUTTON0_NODE DT_NODELABEL(button0) // DT_N_S_buttons_S_button_0
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_cb;

/* Forward declaration of state table */
static const struct smf_state states[];

/* List of demo states */
enum state {
    READ_DEVICE_ID,
    SET_CONFIGMODE,
    SET_PAGEID0,
    SET_EXTERNALCRYSTAL,
    SET_OPMODE,
    READ_QUATERNIONREG,
    SEND_QUATERNIONREG,
    READ_CALIBRATIONREG,
};

/* user defined object */
struct s_object {
    struct smf_ctx ctx;
} s_obj;

int err;
int32_t sleep_msec = 0;
static uint8_t write_i2c_buffer[2];
static uint8_t read_i2c_buffer[8];

struct {
    uint8_t device_id;
    uint8_t mag_cali;
    uint8_t accel_cali;
    uint8_t gyro_cali;
    uint8_t system_cali;
    bool isCalibrated;
    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;
} bno055;

/* State READ DEVICE_ID */
static void read_deviceID_run(void *o) {
    write_i2c_buffer[0] = BNO055_CHIP_ID_ADDR;
    err = i2c_write_read(i2c_dev, BNO055_ADDRESS_A, write_i2c_buffer, 1, read_i2c_buffer, 1);
    if (err < 0) {
        printk("READ_DEVICE_ID failed %d\n", err);
        smf_set_terminate(SMF_CTX(&s_obj), -4);
    } else {
        bno055.device_id = read_i2c_buffer[0];
        printk("Chip ID: 0x%02X \n", bno055.device_id);
        sleep_msec = 1000;
        smf_set_state(SMF_CTX(&s_obj), &states[SET_CONFIGMODE]);
    }
}

/* State SET_CONFIGMODE */
static void set_configmode_run(void *o) {
    printk("SET_CONFIGMODE\n");
    write_i2c_buffer[0] = BNO055_OPR_MODE_ADDR;
    write_i2c_buffer[1] = OPERATION_MODE_CONFIG;
    err = i2c_write(i2c_dev, write_i2c_buffer, 2, BNO055_ADDRESS_A);
    if (err < 0) {
        printk("SET_CONFIGMODE failed: %d\n", err);
    }
	sleep_msec = 25;
    smf_set_state(SMF_CTX(&s_obj), &states[SET_PAGEID0]);
}

/* State SET_PAGEID0 */
static void set_pageid0_run(void *o) {
    printk("SET_PAGEID0\n");
    write_i2c_buffer[0] = BNO055_PAGE_ID_ADDR;
    write_i2c_buffer[1] = 0;
    err = i2c_write(i2c_dev, write_i2c_buffer, 2, BNO055_ADDRESS_A);
    if (err < 0) {
        printk("SET_PAGEID0 failed: %d\n", err);
    }
	sleep_msec = 0;
    smf_set_state(SMF_CTX(&s_obj), &states[SET_EXTERNALCRYSTAL]);
}

/* State SET_EXTERNALCRYSTAL */
static void set_externalcrystal_run(void *o) {
    printk("SET_EXTERNALCRYSTAL\n");
    write_i2c_buffer[0] = BNO055_SYS_TRIGGER_ADDR;
    write_i2c_buffer[1] = 0x80;
    err = i2c_write(i2c_dev, write_i2c_buffer, 2, BNO055_ADDRESS_A);
    if (err < 0) {
        printk("SET_EXTERNALCRYSTAL failed: %d\n", err);
    }
	sleep_msec = 10;
    smf_set_state(SMF_CTX(&s_obj), &states[SET_OPMODE]);
}

/* State SET_OPMODE */
static void set_opmode_run(void *o) {
    printk("SET_OPMODE\n");
    write_i2c_buffer[0] = BNO055_OPR_MODE_ADDR;
    write_i2c_buffer[1] = OPERATION_MODE_NDOF;
    err = i2c_write(i2c_dev, write_i2c_buffer, 2, BNO055_ADDRESS_A);
    if (err < 0) {
        printk("SET_OPMODE failed: %d\n", err);
    }
	sleep_msec = 20;
    smf_set_state(SMF_CTX(&s_obj), &states[READ_QUATERNIONREG]);
}

/* State READ_QUATERNIONREG */
static void read_quaternionreg_run(void *o) {    
    write_i2c_buffer[0] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    err = i2c_write_read(i2c_dev, BNO055_ADDRESS_A, write_i2c_buffer, 1, read_i2c_buffer, 8);
    if (err < 0) {
        printk("READ_QUATERNIONREG failed: %d\n", err);
    }

    bno055.quat_w = (((uint16_t)read_i2c_buffer[1]) << 8 | ((uint16_t)read_i2c_buffer[0]));
    bno055.quat_x = (((uint16_t)read_i2c_buffer[3]) << 8 | ((uint16_t)read_i2c_buffer[2]));
    bno055.quat_y = (((uint16_t)read_i2c_buffer[5]) << 8 | ((uint16_t)read_i2c_buffer[4]));
    bno055.quat_z = (((uint16_t)read_i2c_buffer[7]) << 8 | ((uint16_t)read_i2c_buffer[6]));

    sleep_msec = 0;
    smf_set_state(SMF_CTX(&s_obj), &states[SEND_QUATERNIONREG]);
}

/* State SEND_QUATERNIONREG */
static void send_quaternionreg_run(void *o) {
    otError error = OT_ERROR_NONE;
    char buffer [80];
    
    sprintf(buffer, "{\"quaternions\": [%d, %d, %d, %d]}", bno055.quat_w, bno055.quat_x, bno055.quat_y, bno055.quat_z);

    otInstance *myInstance;
    myInstance = openthread_get_default_instance();
    if (myInstance == NULL){printk("myInstance is NULL\n");}
    otUdpSocket mySocket;

    otMessageInfo messageInfo;
    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString("ff03::1", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = 3101;

    do
    {
        error = otUdpOpen(myInstance, &mySocket, NULL, NULL);
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        otMessage *msg = otUdpNewMessage(myInstance, NULL);
        if (msg == NULL)
        {
            k_msleep(100);
            error = otUdpClose(myInstance, &mySocket);
            printk("msg is NULL. udp closed\n");
            break;
        }
        error = otMessageAppend(msg, buffer, (uint16_t)strlen(buffer));
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        error = otUdpSend(myInstance, &mySocket, msg, &messageInfo);
        if (error != OT_ERROR_NONE)
        {
            break;
        }
        error = otUdpClose(myInstance, &mySocket);
    } while (false);

    if (error != OT_ERROR_NONE)
    {
        printk("udpSend error: %d\n", error);
    }

    if (bno055.isCalibrated) {
        sleep_msec = READ_SENSOR_INTERVALL;
        smf_set_state(SMF_CTX(&s_obj), &states[READ_QUATERNIONREG]);
    } else {
        sleep_msec = 0;
        smf_set_state(SMF_CTX(&s_obj), &states[READ_CALIBRATIONREG]);
    }
}

/* State READ_CALIBRATIONREG */
static void read_calibrationreg_run(void *o) {
    write_i2c_buffer[0] = BNO055_CALIB_STAT_ADDR;
    err = i2c_write_read(i2c_dev, BNO055_ADDRESS_A, write_i2c_buffer, 1, read_i2c_buffer, 1);
    if (err < 0) {
        printk("READ_CALIBRATIONREG failed: %d\n", err);
    }

    bno055.mag_cali = 		read_i2c_buffer[0] & 0x03;
    bno055.accel_cali = 	(read_i2c_buffer[0] >> 2) & 0x03;
    bno055.gyro_cali = 		(read_i2c_buffer[0] >> 4) & 0x03;
    bno055.system_cali = 	(read_i2c_buffer[0] >> 6) & 0x03;

    if (bno055.mag_cali == 3) {
        gpio_pin_set_dt(&led0_spec, 0);
    }

    if (bno055.accel_cali == 3) {
        gpio_pin_set_dt(&led1_spec, 0);
    }

    if (bno055.gyro_cali == 3) {
        gpio_pin_set_dt(&led2_spec, 0);
    }

    if (bno055.system_cali == 3) {
        gpio_pin_set_dt(&led3_spec, 0);
    }

    if (read_i2c_buffer[0] == 0xFF) {
        bno055.isCalibrated = true;
    } else {
        bno055.isCalibrated = false;
    }

    sleep_msec = READ_SENSOR_INTERVALL;
    smf_set_state(SMF_CTX(&s_obj), &states[READ_QUATERNIONREG]);
}

/* Populate state table */
static const struct smf_state states[] = {
    [READ_DEVICE_ID] = SMF_CREATE_STATE(NULL, read_deviceID_run, NULL),
    [SET_CONFIGMODE] = SMF_CREATE_STATE(NULL, set_configmode_run, NULL),
    [SET_PAGEID0] = SMF_CREATE_STATE(NULL, set_pageid0_run, NULL),
    [SET_EXTERNALCRYSTAL] = SMF_CREATE_STATE(NULL, set_externalcrystal_run, NULL),
    [SET_OPMODE] = SMF_CREATE_STATE(NULL, set_opmode_run, NULL),
    [READ_QUATERNIONREG] = SMF_CREATE_STATE(NULL, read_quaternionreg_run, NULL),
    [SEND_QUATERNIONREG] = SMF_CREATE_STATE(NULL, send_quaternionreg_run, NULL),
    [READ_CALIBRATIONREG] = SMF_CREATE_STATE(NULL, read_calibrationreg_run, NULL),
};

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb,
                             gpio_port_pins_t pins)
{
    /* Press button to skip checking if callibrated */

    // Turn of LEDs
    gpio_pin_set_dt(&led0_spec, 0);
    gpio_pin_set_dt(&led1_spec, 0);
    gpio_pin_set_dt(&led2_spec, 0);
    gpio_pin_set_dt(&led3_spec, 0);

    // set isCalibrated to true
    bno055.isCalibrated = true;

    // jump to read state
    smf_set_state(SMF_CTX(&s_obj), &states[READ_QUATERNIONREG]);
}

void main(void) {
    if (!device_is_ready(i2c_dev)) {
        printk("i2c_dev not ready!\n");
        return;
    }
    bno055.isCalibrated = false;

    /* LEDs */
    gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led2_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&led3_spec, GPIO_OUTPUT);
    gpio_pin_set_dt(&led0_spec, 1);
    gpio_pin_set_dt(&led1_spec, 1);
    gpio_pin_set_dt(&led2_spec, 1);
    gpio_pin_set_dt(&led3_spec, 1);

    /* Button */
    gpio_pin_configure_dt(&button0_spec, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button0_cb, button_pressed_callback, BIT(button0_spec.pin));
    gpio_add_callback(button0_spec.port, &button0_cb);

    int32_t ret;

	/* Set inital state */
    smf_set_initial(SMF_CTX(&s_obj), &states[READ_DEVICE_ID]);

    while (1) {
        ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) {
            printk("Error %d \n", ret);
            smf_set_initial(SMF_CTX(&s_obj), &states[READ_DEVICE_ID]);
            sleep_msec = ERROR_SLEEP;
        }
        k_msleep(sleep_msec);
    }
}