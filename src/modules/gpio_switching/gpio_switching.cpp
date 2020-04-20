#include <px4_config.h>
#include "board_config.h"
#include <px4_time.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <poll.h>

#include <arch/board/board.h>
extern "C" __EXPORT  int gpio_switching_main();

int gpio_switching_main() {
    px4_arch_configgpio(GPIO_GPIO0_OUTPUT);

    int count = 0;

    while (count < 100) {

        px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT, 0);

        px4_usleep(1000000);

        px4_arch_gpiowrite(GPIO_GPIO0_OUTPUT, 1);

        px4_usleep(1000000);

        PX4_INFO("this should do something");

        count++;}
    return 0;
}
