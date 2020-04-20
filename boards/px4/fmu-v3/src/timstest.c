//
// Created by tim on 16-03-20.
//
#include <px4_config.h>

#include <stdbool.h>
#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

void test(){

    stm32_configgpio(GPIO_LED1);


}