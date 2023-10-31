
#include "MAVLink.h"
#include "CommonDataTypes.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
    

    if(huart == drone_mavlink_uart){

        for (uint16_t i = 0; i < size; i++)
        {
            drone_mavlink->rx_circular_buffer_->write(drone_mavlink->raw_rx_msg_[i]);
        }
        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, drone_mavlink->raw_rx_msg_, sizeof(drone_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    if(huart == antenna_mavlink_uart){
        for (uint16_t i = 0; i < size; i++)
        {
            antenna_mavlink->rx_circular_buffer_->write(antenna_mavlink->raw_rx_msg_[i]);
        }
        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, antenna_mavlink->raw_rx_msg_, sizeof(drone_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}



 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
    if(huart == drone_mavlink_uart){
        for (uint16_t i = 0; i < sizeof(drone_mavlink->raw_rx_msg_); i++)
        {
            antenna_mavlink->rx_circular_buffer_->write(antenna_mavlink->raw_rx_msg_[i]);
        }

        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, antenna_mavlink->raw_rx_msg_, sizeof(drone_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    if(huart == antenna_mavlink_uart){
        for (uint16_t i = 0; i < sizeof(drone_mavlink->raw_rx_msg_); i++)
        {
            antenna_mavlink->rx_circular_buffer_->write(antenna_mavlink->raw_rx_msg_[i]);
        }

        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, antenna_mavlink->raw_rx_msg_, sizeof(drone_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

 }

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {
    if(huart == drone_mavlink_uart){
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, drone_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }
    if(huart == antenna_mavlink_uart){
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, antenna_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }

 }
