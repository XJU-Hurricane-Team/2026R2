# 2026R2

2026 赛季 R2 控制工程，基于 STM32G4 + FreeRTOS。**该版本适用技能赛九宫藏宝**

>**注意及时更新开发记录中的文件**

### 开发提交流程要求：

1. 克隆 main 分支在本地
2. 在本地修改开发时，另行创建开发分支，如`git checkout -b chassis`，避免在主分支上直接开发。
3. 提交代码时，只提交自己创建的分支，如`git push origin chassis`。
4. 提交完成后，仓库云端应有main,chassis两个分支，将chassis合并至main分支
5. 删除云端和本地的chassis分支。
6. 下次开发时pull最新main分支，再创建分支进行开发。

**禁止直接在main分支上修改，提交**

---

## 项目简介

2026R2 是 2026 赛季 R2 机器人的电控工程项目，运行于 STM32G474VET6。工程以 FreeRTOS 为任务调度核心，使用 STM32 HAL 完成外设驱动，并通过 micro-ROS 与 ROS 2 上位机交换导航指令、机构控制命令和动作状态。

项目围绕完整比赛流程开发，覆盖全向底盘、抬升与上下台阶、机械臂抓取放置、矛头夹取、遥控操作、传感器检测和状态反馈。代码采用应用层、功能模块、板级驱动和通用工具分层组织，既支持遥控调试，也支持上位机参与的自动流程。

## 主要功能

### 底盘控制

- 全向轮底盘运动解算与电机闭环控制
- 遥控手动模式与 ROS 2 导航自动模式
- 自身坐标系和世界坐标系控制切换
- 接收 X/Y 方向线速度及 Z 轴角速度指令
- 支持台阶流程中的导航停止、继续等状态协同

### 抬升与台阶机构

- 抬升机构位置和速度控制
- 上台阶、下台阶自动动作序列
- 光电开关和 VL53L1 测距触发逻辑
- 动作期间的遥控按键互锁与急停处理

### 机械臂

- 多关节机械臂状态机控制
- 抓取准备，识别，放置，取出等多固定姿态和流程
- 根据上位机目标点执行动态抓取
- 气泵控制、轨迹规划及机械臂多解处理
- 通过按键切换已放置层数，并使用 WS2812 显示当前层级

### 夹爪与矛头夹取

- 夹爪初始化、准备、识别、检查和完成状态机
- 矛头夹取与遥控操作
- VL53L1 距离检测及识别抓取流程联动
- 夹爪动作与导航、上位机状态反馈协同

### 通信与调试

- 遥控器数据解析与按键回调注册
- micro-ROS 自定义串口传输
- ROS 2 Service 动作调度及 Topic 状态反馈
- 文本日志和浮点数组数据日志

## 系统架构

```text
ROS 2 上位机
  ├─ 导航速度 Topic
  ├─ 控制调度 Service
  └─ 状态/日志 Topic
          │ UART + micro-ROS
          ▼
STM32G474VETx + FreeRTOS
  ├─ 应用层：底盘、抬升、机械臂、夹爪、消息处理
  ├─ 功能层：运动解算、遥控器、日志、通信协议
  ├─ BSP 层：CAN、电机、舵机、I²C、测距、灯带
  └─ HAL 层：GPIO、DMA、UART、FDCAN、ADC、TIM
```

系统启动后依次完成 HAL/BSP、日志、CAN 列表和 micro-ROS 初始化，随后创建 micro-ROS 等 FreeRTOS 任务，并初始化底盘、夹爪、遥控消息处理和机械臂模块。

## 硬件与软件环境

| 类别 | 配置 |
| --- | --- |
| 主控 | STM32G474VETx，Cortex-M4F |
| 系统时钟 | 170 MHz |
| 实时系统 | FreeRTOS |
| 外设 | GPIO、DMA、USART/LPUART、FDCAN × 3、ADC、TIM |
| 电机与执行器 | DJI 电机、达妙电机、舵机、气泵等 |
| 传感与指示 | VL53L1 测距、接近开关、WS2812、板载 LED |
| ROS 通信 | micro-ROS Humble，自定义 UART Transport（USART1） |

## 软件目录

```text
2026R2/
├─ CubeMX/                       STM32CubeMX 生成的启动、HAL 和外设代码
│  ├─ 2026R2.ioc                CubeMX 工程配置
│  ├─ Core/                     main、外设初始化及中断代码
│  └─ Drivers/                  CMSIS 与 STM32G4 HAL
├─ User/
│  ├─ Application/              机器人业务逻辑和 FreeRTOS 任务
│  │  ├─ Src/chassis.c          底盘控制
│  │  ├─ Src/lift.c             抬升及台阶动作序列
│  │  ├─ Src/arm_ctrl.c         机械臂业务状态机
│  │  ├─ Src/catch.c            夹爪流程
│  │  ├─ Src/microros_ctrl.c    micro-ROS 接口
│  │  └─ Src/msg_process.c      遥控消息与按键处理
│  ├─ Bsp/                      板级外设和设备驱动
│  ├─ Modules/                  可复用功能模块
│  ├─ Utils/                    PID、轨迹规划、FIFO 等通用工具
│  └─ Middlewares/              FreeRTOS、micro-ROS、CMSIS-DSP 等
├─ 开发记录/                    接口、版本、问题和 Bug 记录
├─ .eide/eide.yml               EIDE 工程和编译下载配置
├─ .vscode/                     调试任务配置
└─ Build/                       编译输出目录
```

## 应用模块说明

| 模块 | 主要文件 | 职责 |
| --- | --- | --- |
| 系统任务 | `rtos_tasks.c` | 创建任务、初始化各业务模块、监控运行状态 |
| 底盘 | `chassis.c` | 遥控/自动运动控制、坐标系处理 |
| 抬升 | `lift.c` | 抬升目标控制、上下台阶序列、传感器触发 |
| 机械臂 | `arm_ctrl.c` | 抓取与放置状态机、动态目标和返回序列 |
| 夹爪 | `catch.c` | 夹爪状态流转、识别和抓取协同 |
| 消息处理 | `msg_process.c` | 遥控数据处理及按键功能注册 |
| micro-ROS | `microros_ctrl.c` | ROS 节点初始化、订阅、服务和状态发布 |

## micro-ROS 接口摘要

节点名称为 `board`，命名空间为空。详细字段、命令编号和时序要求请阅读 [`开发记录/上位机接口说明文档.md`](开发记录/上位机接口说明文档.md)。

| 名称 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/nav_speed_heading_data` | `custom_msg/msg/SpeedHeading` | 上位机 → 下位机 | 下发底盘线速度和角速度 |
| `/control_dispatch_srv` | `custom_msg/srv/ControlDispatch` | 双向 | 分派夹爪、机械臂和抬升命令 |
| `/nav_topic` | `std_msgs/msg/Int8` | 下位机 → 上位机 | 导航流程状态反馈 |
| `/control_dispatch_topic` | `std_msgs/msg/Int8` | 下位机 → 上位机 | 机构动作完成状态反馈 |
| `/log/msg` | `std_msgs/msg/String` | 下位机 → 上位机 | 文本日志 |
| `/log/data` | `std_msgs/msg/Float32MultiArray` | 下位机 → 上位机 | 带数据 ID 的实时数值日志 |

## 上电与运行流程

1. HAL 初始化并配置 170 MHz 系统时钟。
2. 初始化 GPIO、DMA、UART、三路 FDCAN、ADC 和定时器。
3. 调用 `bsp_init()` 初始化板级设备。
4. 启动 FreeRTOS 调度器。
5. 初始化日志、CAN 设备列表和 micro-ROS Support/Node/Executor。
6. 创建 micro-ROS 任务，初始化导航订阅与控制调度服务。
7. 初始化底盘、夹爪、遥控消息处理和机械臂。
8. 各控制任务进入周期运行，等待遥控输入、传感器事件或上位机命令。

**板载 LED/灯带在当前代码中的辅助含义：**

- LED0：周期翻转，作为基础任务存活指示。
- LED3：收到导航速度消息时限频翻转，表示 micro-ROS 导航数据活动。
- WS2812：显示车辆当前已装载kfs数量；0/1/2/3 分别对应熄灭、绿色、黄色、红色。

## 开发记录文档说明

| 文档 | 内容 |
| --- | --- |
| [`上位机接口说明文档.md`](开发记录/上位机接口说明文档.md) | micro-ROS 服务、主题、命令和反馈协议 |
| [`版本记录.md`](开发记录/版本记录.md) | 各版本新增功能与修复记录 |
| [`Bug记录.md`](开发记录/Bug记录.md)                       | 调试期间的 Bug 现象与处理记录        |
| [`问题清单.md`](开发记录/问题清单.md) | 已发现问题及后续处理线索 |
