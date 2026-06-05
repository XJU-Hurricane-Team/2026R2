# Bug记录

1. 底盘有时会不受控制，倒不是疯转，像是电机被开环控制，无法控制转速。

   原因：

   当can消息接收队列没创建好，或满时，fifo中断对fifo消息直接return,次数过多会导致后续无法进入电机回调函数，电机状态不能更新，电机控制无法闭环，所以电机速度失控。
    在原本直接return的逻辑前添加can_list_fdcan_drain_fifo，将数据从fifo中读出来，保持fifo流畅。

   原因可能是FIFO设置的BLOCK模式，一旦FIFO 3个邮箱被填满，旧的消息帧就无法进入FIFO,又因为对于` HAL_FDCAN_RxFifo0Callback`的回调触发，只设置了`HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);`一种中断触发源。所以FIFO满了，新消息进不去，回调无法触发，FIFO中的信息读取不出来，所以逻辑闭环导致电机得不到反馈帧，开环失控。

   解决：
   
   1.  在原本can_list处理中直接return的逻辑前添加can_list_fdcan_drain_fifo。保证FIFO流动;
   2. 将FIFO接收改为覆盖模式，`    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0,  FDCAN_RX_FIFO_OVERWRITE);`
   3. 尚未进行：`void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);`FIFO的回调触发有多种事件类型。但是函数签名的第二个参数我们暂时没有使用，以后有时间可以完善，健壮一下CAN通信。
   
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

   解决：

   1. 3号电机电调有毛病。

6. 机械臂第四个电机反馈信息无法接收。

   原因：并发溢出问题。达妙电机一发一收，机械臂控制代码里，对四个电机瞬间发送
   ```
       dm_pvt_ctrl(&arm->damiao_1, arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
       dm_pvt_ctrl(&arm->damiao_2, -arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
       dm_pvt_ctrl(&arm->damiao_3, joint1_final_cmd, joint1_final_speed, 0.90f);
       dm_pvt_ctrl(&arm->damiao_4, joint2_final_cmd, joint2_final_speed, 0.90f);
   ```

   所以电机在很短时间内进行反馈，但软件代码中can_list处理FIFO消息速度跟不上，导致前三个电机消息将FIFO占满，第四个电机反馈消息被丢弃。如果拔掉一个电机，或者将第四个电机的发送提到前面，第四个电机接收也就恢复正常。

   解决：FIFO有两个，共六个邮箱。但之前过滤器配置全为0,相当于没有过滤，所以默认所有消息都存在FIFO0中。相当于只使用了FIFO0的三个邮箱，FIFO1的一直空置。

   现在更改过滤器配置，将前两个电机消息配在FIFO0中，后两个配在FIFO1中。四个电机接收反馈都正常。