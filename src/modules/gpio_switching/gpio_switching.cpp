#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <arch/board/board.h>

extern "C" __EXPORT  int gpio_switching_main(int argc, char *argv[]);

class Gpio_Switching: public ModuleBase<Gpio_Switching>{

public:
    Gpio_Switching();
    ~Gpio_Switching();

    static int task_spawn(int argc, char *argv[]);

    static Gpio_Switching *instantiate(int argc, char *argv[]);

    static int custom_command(int argc, char *argv[]);

    static int print_usage(const char *reason = nullptr);

    void run() override;

    int print_status() override;

private:
    bool state;
};

Gpio_Switching::Gpio_Switching() {

}

Gpio_Switching::~Gpio_Switching() {

}

void Gpio_Switching::run() {

    state = false;
    px4_arch_configgpio(GPIO_GPIO0_OUTPUT);

    while (true){
        px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT,!state);

        state = !state;

        px4_sleep(1);
    }
}

int Gpio_Switching::task_spawn(int argc, char **argv) {
/* start the task */
_task_id = px4_task_spawn_cmd("gpio_switching",
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

int Gpio_Switching::print_status() {
    PX4_INFO("current GPIO state: %i",state);
    return 0;
}

int Gpio_Switching::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int Gpio_Switching::print_usage(const char *reason)
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

Gpio_Switching *Gpio_Switching::instantiate(int argc, char *argv[])
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

    Gpio_Switching *instance = new Gpio_Switching();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int gpio_switching_main(int argc, char *argv[])
{
    return Gpio_Switching::main(argc, argv);
}
