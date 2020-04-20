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

    px4_arch_configgpio(GPIO_GPIO0_OUTPUT);
    px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT,0);


    while (true){

        px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT,0);

        px4_usleep(1000000);

        px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT,1);

        px4_usleep(1000000);

        PX4_INFO("this should do something");

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
