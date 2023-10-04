#ifndef DRIVER_CONFIG_HPP_
#define DRIVER_CONFIG_HPP_

#include <cstdint>
#include "main.h"
#include "usart.h"
/*
    MAVlink Instances
*/
#include "MAVLink.h"

/*
    UART Mapping
*/

#define drone_mavlink_uart    &huart3
#define antenna_mavlink_uart     &huart2


extern MAVLink* drone_mavlink;
extern MAVLink* antenna_mavlink;


#endif
