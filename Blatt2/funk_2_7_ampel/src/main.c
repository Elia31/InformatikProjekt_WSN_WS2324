#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/smf.h>

#define RED_SLEEP_TIME_MS           3000
#define RED_YELLOW_SLEEP_TIME_MS    1000
#define GREEN_SLEEP_TIME_MS         3000
#define YELLOW_SLEEP_TIME_MS        1000

static const struct smf_state states[];
enum state { RED, RED_YELLOW, GREEN, YELLOW };

struct s_object {
    struct smf_ctx ctx; /* This must be first */
    /* Other state specific data add here */
} s_obj;

int32_t sleep_msec;

static void red_run(void *o)
{
    printk("RED\n");
    sleep_msec = RED_SLEEP_TIME_MS;
    smf_set_state(SMF_CTX(&s_obj), &states[RED_YELLOW]);
}

static void red_yellow_run(void *o)
{
    printk("RED YELLOW\n");
    sleep_msec = RED_YELLOW_SLEEP_TIME_MS;
    smf_set_state(SMF_CTX(&s_obj), &states[GREEN]);
}

static void green_run(void *o)
{
    printk("GREEN\n");
    sleep_msec = GREEN_SLEEP_TIME_MS;
    smf_set_state(SMF_CTX(&s_obj), &states[YELLOW]);
}

static void yellow_run(void *o)
{
    printk("YELLOW\n");
    sleep_msec = YELLOW_SLEEP_TIME_MS;
    smf_set_state(SMF_CTX(&s_obj), &states[RED]);
}

static const struct smf_state states[] = {
    [RED] = SMF_CREATE_STATE(NULL, red_run, NULL),
    [RED_YELLOW] = SMF_CREATE_STATE(NULL, red_yellow_run, NULL),
    [GREEN] = SMF_CREATE_STATE(NULL, green_run, NULL),
    [YELLOW] = SMF_CREATE_STATE(NULL, yellow_run, NULL),
};

int main(void)
{
    int32_t ret;
    /* Set initial state */
    smf_set_initial(SMF_CTX(&s_obj), &states[RED]);
    while (1) 
    {
        ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) 
        {
            break;
        }
        k_msleep(sleep_msec);
    }
}
