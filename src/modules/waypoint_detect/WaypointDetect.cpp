//
// Created by sages on 23-3-20.
//

#include <math.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/relay_controls.h>

extern "C" __EXPORT int waypointdetect_main(int argc, char *argv[]);

class WayPointDetect: public ModuleBase<WayPointDetect>{
public:
    WayPointDetect();
    ~WayPointDetect();

    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static WayPointDetect *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;


private:

};

WayPointDetect::WayPointDetect() {

}

WayPointDetect::~WayPointDetect() {

}

void WayPointDetect::run() {


    int relay_sub_fd = orb_subscribe(ORB_ID(relay_controls));

    orb_set_interval(relay_sub_fd,200);

    px4_pollfd_struct_t fds[] = {
            {.fd = relay_sub_fd, .events=POLLIN},
    };

    while (true){

        int poll_ret = px4_poll(fds,1,1000);

        if (poll_ret == 0){
                        PX4_ERR("Got no data within a second");
        }

        else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */

                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }


            else{
                 if (fds[0].revents & POLLIN) {
                     struct relay_controls_s raw;

                     orb_copy(ORB_ID(relay_controls),relay_sub_fd,&raw);

                     PX4_INFO("%8.4f\t%8.4f",double(raw.number),double(raw.state));
                    #if defined  GPIO_GPIO0_OUTPUT
                     switch (raw.number){

                         case 0:
                             px4_arch_configgpio(GPIO_GPIO0_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT,raw.state);
                             break;
                         case 1:
                             px4_arch_configgpio(GPIO_GPIO1_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO1_OUTPUT,raw.state);
                             break;
                         case 2:
                             px4_arch_configgpio(GPIO_GPIO2_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO2_OUTPUT,raw.state);
                             break;
                         case 3:
                             px4_arch_configgpio(GPIO_GPIO3_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO3_OUTPUT,raw.state);
                             break;
                         case 4:
                             px4_arch_configgpio(GPIO_GPIO4_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO4_OUTPUT,raw.state);
                             break;
                         case 5:
                             px4_arch_configgpio(GPIO_GPIO5_OUTPUT);
                             px4_arch_gpiowrite(GPIO_GPIO5_OUTPUT,raw.state);
                             break;

                         default:
                             break;

                     }
                    #endif


                 }
            }
    }

}


int WayPointDetect::task_spawn(int argc, char **argv) {
    /* start the task */
    _task_id = px4_task_spawn_cmd("waypointdetect",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

int WayPointDetect::print_status() {
    PX4_INFO("all seems to be going well");
    return 0;
}

int WayPointDetect::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int WayPointDetect::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels
  to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and
  `manual_control_setpoint`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sensors", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

WayPointDetect *WayPointDetect::instantiate(int argc, char *argv[])
{
    //int example_param = 0;
    //bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'p':
                //example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                //example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    WayPointDetect *instance = new WayPointDetect();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int waypointdetect_main(int argc, char *argv[])
{
    return WayPointDetect::main(argc, argv);
}
