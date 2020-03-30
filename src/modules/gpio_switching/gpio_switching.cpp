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
    px4_arch_gpiowrite(GPIO_TIM1_CH1OUT,1);
    PX4_INFO("Switching");
    return 0;
};
