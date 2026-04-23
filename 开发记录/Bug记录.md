# Bug记录

1. 底盘有时会不受控制，倒不是疯转，像是电机被开环控制，无法控制转速。

   解决：

   当can消息接收队列没创建好，或满时，fifo中断对fifo消息直接return,次数过多会导致后续无法进入电机回调函数，电机状态不能更新，电机控制无法闭环，所以电机速度失控。
    在原本直接return的逻辑前添加can_list_fdcan_drain_fifo，将数据从fifo中读出来，保持fifo流畅。

   原因可能是can_list任务读取不及时，导致fifo会长时间占满。**但为什么fifo占满，后面就进不去电机回调，暂时不太清楚。**

   ```c
   static void can_list_fdcan_drain_fifo(FDCAN_HandleTypeDef *hfdcan,uint32_t rx_fifo) {
       FDCAN_RxHeaderTypeDef rx_header;
       uint8_t rx_data[64];
   
       while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, rx_fifo) > 0U) {
           if (HAL_FDCAN_GetRxMessage(hfdcan, rx_fifo, &rx_header, rx_data) !=
               HAL_OK) {
               break;
           }
       }
   }
   
   /**
    * @brief Rx FIFO 0 callback.
    * 
    * @param hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
    *        the configuration information for the specified FDCAN.
    * @param RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
    *        This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
    */
   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t RxFifo0ITs) {
       if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) {
           return;
       }
   
   #if CAN_LIST_USE_RTOS
       if (can_list_queue_handle == NULL) {
           can_list_fdcan_drain_fifo(hfdcan, FDCAN_RX_FIFO0);
           return;
       }
   
       send_msg_from_isr.hcan = hfdcan;
       send_msg_from_isr.rx_fifo = FDCAN_RX_FIFO0;
       if (xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL) !=
           pdPASS) {
           can_list_fdcan_drain_fifo(hfdcan, FDCAN_RX_FIFO0);
       }
   #else  /* CAN_LIST_USE_RTOS */
       can_message_process(hfdcan, FDCAN_RX_FIFO0);
   #endif /* CAN_LIST_USE_RTOS */
   }
   ```

2. 机械臂部分速度设置过快会产生抖动，吸盘关节会产生一定过冲，若有外力触碰小臂会造成整个结构的剧烈震荡。问题待排查。
