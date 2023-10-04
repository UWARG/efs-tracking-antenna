#include "CommonDataTypes.h"

/*
    creating mavlink instance
*/

MAVLink drone_mavlink_instance(drone_mavlink_uart);
MAVLink* drone_mavlink = &drone_mavlink_instance;

MAVLink antenna_mavlink_instance(antenna_mavlink_uart);
MAVLink* antenna_mavlink = &antenna_mavlink_instance;
