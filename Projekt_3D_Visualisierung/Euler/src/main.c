#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/smf.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>
#include <openthread/udp.h>
#include <math.h>

#define I2C_NODE		DT_NODELABEL(i2c0)	//DT_N_S_soc_i2c_40003000
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);


//* BNO055 Address A (Pin17 LOW) **/
#define BNO055_ADDRESS_A (0x28)
//* BNO055 Address B (Pin17 High) **/
#define BNO055_ADDRESS_B (0x29)

/*Operationsmodus*/
#define OPERATION_MODE_CONFIG	(0x00)
#define OPERATION_MODE_AMG		(0x07)
#define OPERATION_MODE_NDOF		(0x0C)

/*Powermodus*/
#define POWER_MODE_NORMAL		(0x00)
#define POWER_MODE_LOWPOWER		(0x01)
#define POWER_MODE_SUSPEND		(0x02)

/*Registeraddressen*/
#define BNO055_CHIP_ID_ADDR						(0x00)
#define BNO055_PAGE_ID_ADDR						(0x07)
#define BNO055_QUATERNION_DATA_W_LSB_ADDR		(0x20)
#define EUL_PITCH_LSB                           (0x1E) // Datasheet (euler daten direkt)
#define EUL_ROLL_LSB                            (0x1C) // Datasheet (euler daten direkt)
#define EUL_HEADING_LSB                         (0x1A) // Datasheet (euler daten direkt)
#define ACC_DATA_X_LSB                          (0x08) // Datasheet (acc daten x)       //eigentlich braucht man nur das
#define ACC_DATA_Y_LSB                          (0x0A) // Datasheet (acc daten y)
#define ACC_DATA_Z_LSB                          (0x0C) // Datasheet (acc daten z)
#define MAG_DATA_X_LSB                          (0x0E) // Datasheet (mag daten x)
#define MAG_DATA_Y_LSB                          (0x10) // Datasheet (mag daten y)
#define MAG_DATA_Z_LSB                          (0x12) // Datasheet (mag daten z)
#define GYR_DATA_X_LSB                          (0x14) // Datasheet (gyro daten x) 
#define GYR_DATA_Y_LSB                          (0x16) // Datasheet (gyro daten y)
#define GYR_DATA_Z_LSB                          (0x18) // Datasheet (gyro daten z)
#define BNO055_CALIB_STAT_ADDR					(0x35)
#define BNO055_OPR_MODE_ADDR					(0x3D)
#define BNO055_PWR_MODE_ADDR					(0x3E)
#define BNO055_SYS_TRIGGER_ADDR					(0x3F)

#define M_PI                                    3.14159265358979323846

#define READ_SENSOR_INTERVALL		20
#define ERROR_SLEEP					5000


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
    READ_EULREG,
    SEND_EULREG,
    READ_CALIBRATIONREG,
};

/* user defined object */
struct s_object {
    struct smf_ctx ctx;
	/* other state specified data add here */
} s_obj;

int err;
int32_t sleep_msec = 0;
static uint8_t write_i2c_buffer[3];
static uint8_t read_i2c_buffer[24];

struct {
    uint8_t device_id;
    uint8_t mag_cali;
    uint8_t accel_cali;
    uint8_t gyro_cali;
    uint8_t system_cali;
    bool isCalibrated;
    double eul_roll;
    double eul_pitch;
    double eul_yaw;

    double acc_x_m_s2;
    double acc_y_m_s2;
    double acc_z_m_s2;

    double gyr_x_dps;
    double gyr_y_dps;
    double gyr_z_dps;

    double mag_x_mT;
    double mag_y_mT;
    double mag_z_mT;
} bno055;

struct {
    double thetaM;
    double phiM;
    double thetaFold;
    double thetaFnew;
    double phiFold;
    double phiFnew;

    double thetaG;
    double phiG;

    double theta;
    double phi;

    double thetaRad;
    double phiRad;

    double Xm;
    double Ym;
    double psi;

    double dt;
    uint64_t millisOld;
} data;


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
    smf_set_state(SMF_CTX(&s_obj), &states[READ_EULREG]);
}

/* State READ_EULREG */
static void read_eulreg_run(void *o) {
    // DATEN
    write_i2c_buffer[0] = ACC_DATA_X_LSB;
    err = i2c_write_read(i2c_dev, BNO055_ADDRESS_A, write_i2c_buffer, 1, read_i2c_buffer, 18);
    if (err < 0) {
        printk("READ_EULREG failed: %d\n", err);
    }

    int16_t acc_x, acc_y, acc_z;

    int16_t gyr_x, gyr_y, gyr_z;
    double gyr_x_rps, gyr_y_rps, gyr_z_rps; // radians per second

    int16_t mag_x, mag_y, mag_z;

    int16_t eul_x, eul_y, eul_z;

    acc_x =		(((uint16_t)read_i2c_buffer[1]) << 8  | ((uint16_t)read_i2c_buffer[0])); 
	acc_y =		(((uint16_t)read_i2c_buffer[3]) << 8  | ((uint16_t)read_i2c_buffer[2]));
	acc_z =		(((uint16_t)read_i2c_buffer[5]) << 8  | ((uint16_t)read_i2c_buffer[4]));

	mag_x =		(((uint16_t)read_i2c_buffer[7]) << 8  | ((uint16_t)read_i2c_buffer[6])); 
	mag_y = 	(((uint16_t)read_i2c_buffer[9]) << 8  | ((uint16_t)read_i2c_buffer[8]));
	mag_z =		(((uint16_t)read_i2c_buffer[11]) << 8 | ((uint16_t)read_i2c_buffer[10]));

	gyr_x =		(((uint16_t)read_i2c_buffer[13]) << 8 | ((uint16_t)read_i2c_buffer[12])); 
	gyr_y =		(((uint16_t)read_i2c_buffer[15]) << 8 | ((uint16_t)read_i2c_buffer[14]));
	gyr_z =		(((uint16_t)read_i2c_buffer[17]) << 8 | ((uint16_t)read_i2c_buffer[16]));


    //abtastrate von gyro+accel:        100hz, mag: 20hz
    //accelerometer unit:               1 m/s^2 = 100 LSB
    //gyroscope unit:                   1 dps (degree per second) = 16 LSB
    //magnetometer unit:                1 mT = 16 LSB

    //acc zu m/s^2
    bno055.acc_x_m_s2 = acc_x / 100.0;
    bno055.acc_y_m_s2 = acc_y / 100.0;
    bno055.acc_z_m_s2 = acc_z / 100.0;

    //mag zu µT
    bno055.mag_x_mT = mag_x / 16.0;
    bno055.mag_y_mT = mag_y / 16.0;
    bno055.mag_z_mT = mag_z / 16.0;

    //gyr zu dps (degrees per second)
    bno055.gyr_x_dps = gyr_x / 16.0;
    bno055.gyr_y_dps = gyr_y / 16.0;
    bno055.gyr_z_dps = gyr_z / 16.0;

    //gyr zu rps (radians per second) !!! PI/180 = 0.01745329251
    //gyr_x_rps = (gyr_x_dps / 180.0) * M_PI;
    //gyr_y_rps = (gyr_y_dps / 180.0) * M_PI;
    //gyr_z_rps = (gyr_z_dps / 180.0) * M_PI;
    gyr_x_rps = bno055.gyr_x_dps * 0.01745329251;
    gyr_y_rps = bno055.gyr_y_dps * 0.01745329251;
    gyr_z_rps = bno055.gyr_z_dps * 0.01745329251;


    //hier berechnung von euler daten aus acc + gyr + mag ///////////////////////////////////////////////////////////

    //thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
    data.thetaM     = -atan2(bno055.acc_x_m_s2 / 9.81, bno055.acc_z_m_s2 / 9.81) / 2 / M_PI * 360;
    //phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
    data.phiM       = -atan2(bno055.acc_y_m_s2 / 9.81, bno055.acc_z_m_s2 / 9.81) / 2 / M_PI * 360;
    //phiFnew=.95*phiFold+.05*phiM;
    data.phiFnew    = 0.95 * data.phiFold + 0.05 * data.phiM;
    //thetaFnew=.95*thetaFold+.05*thetaM;
    data.thetaFnew  = 0.95 * data.thetaFold + 0.05 * data.thetaM;

    //dt=(millis()-millisOld)/1000.;
    //millisOld=millis();
    data.dt         = (k_uptime_get() - data.millisOld) / 1000.0;
    data.millisOld  = k_uptime_get();

    //theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
    data.theta      = (data.theta + gyr_y_rps * data.dt) * 0.95 + data.thetaM * 0.05;
    //phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
    data.phi        = (data.phi - gyr_x_rps * data.dt) * 0.95 + data.phiM * 0.05;
    //thetaG=thetaG+gyr.y()*dt;
    data.thetaG     = data.thetaG + gyr_y_rps * data.dt;
    //phiG=phiG-gyr.x()*dt;
    data.phiG       = data.phiG - gyr_x_rps * data.dt;

    //phiRad=phi/360*(2*3.14);
    data.phiRad     = data.phi / 360 * (2 * M_PI);
    //thetaRad=theta/360*(2*3.14);
    data.thetaRad   = data.theta / 360 * (2 * M_PI);

    //Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
    data.Xm         = bno055.mag_x_mT * cos(data.thetaRad) - bno055.mag_y_mT * sin(data.phiRad) * sin(data.thetaRad) + bno055.mag_z_mT * cos(data.phiRad) * sin(data.thetaRad);
    //Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
    data.Ym         = bno055.mag_y_mT * cos(data.phiRad) + bno055.mag_z_mT * sin(data.phiRad);

    //psi=atan2(Ym,Xm)/(2*3.14)*360;
    data.psi        = atan2(data.Ym, data.Xm) / (2 * M_PI) * 360;
    
    bno055.eul_roll = data.phi;
    bno055.eul_pitch = data.theta;
    bno055.eul_yaw = data.psi;

    //phiFold=phiFnew;
    data.phiFold = data.phiFnew;
    //thetaFold=thetaFnew;
    data.thetaFold = data.thetaFnew;


    //hier ende von berechnugen /////////////////////////////////////////////////////////////////////////////////////
    
    sleep_msec = 0;
    smf_set_state(SMF_CTX(&s_obj), &states[SEND_EULREG]);
}

/* State SEND_EULREG */
static void send_eulreg_run(void *o) {
    otError error = OT_ERROR_NONE;
    char buffer [150]; //muss noch angepasst werden am ende

    // euler roll pitch yaw, acc x y z, gyr x y z, mag x y z
    sprintf(buffer, "{\"euler\": [%lf, %lf, %lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]}", 
        bno055.eul_roll, bno055.eul_pitch, bno055.eul_yaw, bno055.acc_x_m_s2, bno055.acc_y_m_s2, bno055.acc_z_m_s2,
        bno055.gyr_x_dps, bno055.gyr_y_dps, bno055.gyr_z_dps, bno055.mag_x_mT, bno055.mag_y_mT, bno055.mag_z_mT);

    otInstance *myInstance;
    myInstance = openthread_get_default_instance();
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

    if (error == OT_ERROR_NONE)
    {
        //printk("Send.\n");
    }
    else
    {
        printk("udpSend error: %d\n", error);
    }

    if (bno055.isCalibrated) {
        printk("{\"euler\": [%lf, %lf, %lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]}\n", 
        bno055.eul_roll, bno055.eul_pitch, bno055.eul_yaw, bno055.acc_x_m_s2, bno055.acc_y_m_s2, bno055.acc_z_m_s2,
        bno055.gyr_x_dps, bno055.gyr_y_dps, bno055.gyr_z_dps, bno055.mag_x_mT, bno055.mag_y_mT, bno055.mag_z_mT);
        sleep_msec = READ_SENSOR_INTERVALL;
        smf_set_state(SMF_CTX(&s_obj), &states[READ_EULREG]);
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

    printk("mag: %d\t acc: %d\t gyro: %d\t sys: %d\n", bno055.mag_cali, bno055.accel_cali, bno055.gyro_cali, bno055.system_cali);

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
    smf_set_state(SMF_CTX(&s_obj), &states[READ_EULREG]);
}

/* Populate state table */
static const struct smf_state states[] = {
    [READ_DEVICE_ID] = SMF_CREATE_STATE(NULL, read_deviceID_run, NULL),
    [SET_CONFIGMODE] = SMF_CREATE_STATE(NULL, set_configmode_run, NULL),
    [SET_PAGEID0] = SMF_CREATE_STATE(NULL, set_pageid0_run, NULL),
    [SET_EXTERNALCRYSTAL] = SMF_CREATE_STATE(NULL, set_externalcrystal_run, NULL),
    [SET_OPMODE] = SMF_CREATE_STATE(NULL, set_opmode_run, NULL),
    [READ_EULREG] = SMF_CREATE_STATE(NULL, read_eulreg_run, NULL),
    [SEND_EULREG] = SMF_CREATE_STATE(NULL, send_eulreg_run, NULL),
    [READ_CALIBRATIONREG] = SMF_CREATE_STATE(NULL, read_calibrationreg_run, NULL),
};

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb,
                             gpio_port_pins_t pins)
{
    /* Press button to skip calibrating */

    // Turn of LEDs
    gpio_pin_set_dt(&led0_spec, 0);
    gpio_pin_set_dt(&led1_spec, 0);
    gpio_pin_set_dt(&led2_spec, 0);
    gpio_pin_set_dt(&led3_spec, 0);

    // set isCalibrated to true
    bno055.isCalibrated = true;

    // jump to read state
    smf_set_state(SMF_CTX(&s_obj), &states[READ_EULREG]);
}

void main(void) {
    if (!device_is_ready(i2c_dev)) {
        printk("i2c_dev not ready!\n");
        return;
    }
    bno055.isCalibrated = false;
    data.thetaFold = 0;
    data.phiFold = 0;
    data.thetaG=0;
    data.phiG=0;
    data.millisOld=k_uptime_get_32();

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