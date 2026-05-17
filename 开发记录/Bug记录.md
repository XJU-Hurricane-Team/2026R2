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


2. 机械臂用两个8006协同控制的关节总是震荡。

   解决：将达妙换为pvt控制模式，将上位机中p的值压低到10。

   原因：在速度位置控制模式时，达妙可能由于极小的较零偏差和can指令下发的先后顺序，以及机械结构的中心偏移，导致总有一个电机承受绝大部分的力，比如一个电机承受了17N的力，另一个电机才受不到1N的力，这种控制模式下一旦超出电机所控制的扭矩，就会产生震荡。pvt模式的好处是能控制最大扭矩，达到设定最大扭矩后不再超载增加，但切换模式后还是会产生震荡。发现原因是上位机的p设置为了默认的54，在机械臂运动过程中，电机受力急剧变化，由于惯性或者其他原因会产生偏离目标点的误差，当p=54时，会导致电机在矫正小角度时反复进行过冲校准，从而产生剧烈震荡。

3. microros板子上总是初始化失败，严重依赖上电顺序。
   解决：
   
   1. 提高串口波特率，由115200提升至921600。之前速率慢，当上位机端nav等框架一启动，会挤占agent对底层资源的使用。导致单片机与agent交互超时，初始化时失败率增高。
   
4. 机械臂小臂在超过180°转动时会反向。

   解决：过渡点设置到180°内，也可进行加减2*PI的校准，不过会影响角度误差的判断条件，修改位置较多，把过渡点改成180°附近是比较简单的一个方案，后续按情况决定是否采用后者方法。
   
5. 底盘3号轮电机反馈频率显著低于其他电机，且个别达妙电机使能失败频率有点高。