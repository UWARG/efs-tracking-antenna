#ifndef DRIVER_CONFIG_HPP_
#define DRIVER_CONFIG_HPP_

#include <cstdint>
#include "main.h"
#include "usart.h"

/*
    UART Mapping
*/

#define drone_mavlink_uart    &huart2
#define antenna_mavlink_uart     &huart6

/*
    MAVlink Instances
*/
#include "SSM_D_MAVLink.hpp"

extern MAVLink* drone_mavlink;
extern MAVLink* antenna_mavlink;


#endif
