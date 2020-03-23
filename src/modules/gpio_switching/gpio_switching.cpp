#include "gpio_switching.h"
#include <px4_config.h>
#include "board_config.h"

#include <arch/board/board.h>

void gpio_switching::main() {
    stm32_gpiowrite(GPIO_PIN14, false);
}
