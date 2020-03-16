#include "gpio_switching.h"
#include <px4_config.h>
#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

stm32_gpiowrite(GPIO_PIN14, false);
