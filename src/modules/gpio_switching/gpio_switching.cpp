#include "gpio_switching.h"
#include <px4_config.h>
#include "/home/rafael/dev/PX4/platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32/stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

void gpio_switching::main() {
    stm32_gpiowrite(GPIO_PIN14, false);
}
