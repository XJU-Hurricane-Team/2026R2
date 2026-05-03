/**
 * @file arm_ctrl.c
 * @author xinglu
 * @brief 机械臂控制 (应用层任务与点位逻辑)
 * @version 2.0
 * @date 2026-04-30
 */

#include "includes.h"
#include "arm_ctrl.h"

/* === 宏定义与配置区 === */
#define ARM_USE_REMOTE_KEY          0           // 是否使用遥控器按键切换点位 (0:禁用, 1:启用)
#define ARM_USE_BOARD_KEY           1           // 是否使用主板实体按键切换点位 (0:禁用, 1:启用)
#define ARM_USE_MICROROS_SWITCH     0           // 仅允许按键切换机械臂状态 (0:禁用, 1:启用)
#define ARM_SWITCH_KEY              10          // 遥控器映射键值定义
#define ARM_TASK_PERIOD_MS          20          // 机械臂控制任务周期 (毫秒)

#define FIRST_POINT_Z               493.991f    // 放置底层的基准高度 (毫米)

/* 函数前置声明 */
void robot_arm_switch_target(uint8_t key, remote_key_event_t event);
void robot_arm_apply_target(uint8_t index);
void robot_arm_task(void *pvParameters);
static void robot_arm_try_next_state(void);

/**
 * @brief 定义目标点位结构体 (笛卡尔空间)
 */
typedef struct {
	float y;        // 前后伸出距离 (mm)
	float z;        // 上下高度 (mm)
	float pitch;    // 末端倾角姿态 (弧度，水平面为0，向下为负)
} arm_target_point_t;

/* 全局与静态变量定义 */
static RobotArm g_robot_arm;                // 全局机械臂实体对象
static uint8_t g_arm_target_index;          // 当前全局目标点位索引 (0~4)
static TaskHandle_t g_robot_arm_task_handle;// RTOS 任务句柄
static uint8_t g_last_target_index;         // 记录上一个点位索引

// 当前选择的放置点索引 (0:底层, 1:中层, 2:顶层)
static uint8_t g_place_target_index = 0; 

// --- 新增：动态抓取坐标缓存 ---
static arm_target_point_t g_dynamic_target = {0}; 
static uint8_t g_has_dynamic_target = 0; // 是否已收到有效坐标的标志位

/**
 * @brief 全局预设点位数组
 * 分别对应：零点、准备、抓取、放置、取出。
 */
static const arm_target_point_t g_arm_target_points[5] = {
	{139.95f + 20.0f, 102.70f, 0.8955f},  // 0: 预设零点 (INIT)，姿态朝上折叠
	{533.142f, 400.0f, 0.0f},             // 1: 准备点位 (READY)，抬起手臂
	{533.142f, 91.5027f, 0.0f},           // 2: 抓取点位 (CATCH)，下降到抓取高度 (将动态被外部坐标覆盖)
	{-275.12f, 493.991f, -PI/2.0},        // 3: 放置点位 (PLACE)，向后方放置 (该点实际在应用中被下方数组覆盖)
	{533.142f, 300.0f, 0.0f},             // 4: 取出点位 (TAKEOUT)，抓取后的中间姿态
};

/**
 * @brief 独立的放置点数组 (多层货架逻辑)
 * 根据层数 (g_place_target_index)，Z轴高度依次增加，且姿态发生变化。
 */
static const arm_target_point_t g_arm_place_points[3] = {
	{-275.12f, FIRST_POINT_Z,          -PI/2.0},                      // 放置点 0 (底层): 垂直向下放 (-PI/2)
	{-275.12f + 175.0f, FIRST_POINT_Z + 175.0f, -PI},           // 放置点 1 (中层): 水平向后放 (-PI)
	{-275.12f + 175.0f, FIRST_POINT_Z + 175.0f + 350.0f, -PI},  // 放置点 2 (顶层): Z轴加高 350mm
};

/**
 * @brief 将整数索引映射为驱动层的状态枚举
 */
static arm_status_t arm_status_from_index(uint8_t index) {
	switch (index) {
	case 0: return ARM_STATE_INIT;
	case 1: return ARM_STATE_READY;
	case 2: return ARM_STATE_CATCH;
	case 3: return ARM_STATE_PLACE;
	case 4: return ARM_STATE_TAKEOUT;
	default: return ARM_STATE_INIT;
	}
}

/**
 * @brief 外部调用接口：注入动态抓取坐标
 * @note 供 MicroROS 或视觉模块调用。只有在 READY(就绪态) 接收到的坐标才会被缓存。
 */
void robot_arm_set_dynamic_catch_target(float y, float z, float pitch) {
    if (g_robot_arm.status == ARM_STATE_READY) {
        g_dynamic_target.y = z * 1000.0f + 533.142f - 300.0f; // 转换为毫米
        g_dynamic_target.z = pitch * 1000.0f + 400.0f; // 转换为毫米
        g_dynamic_target.pitch = 0.0f; // 固定姿态
        g_has_dynamic_target = 1; // 标记收到有效坐标
    }
}

/**
 * @brief 外部调用接口：手动设置当前要放置的箱子层数
 * @param place_idx 层数索引 (0~2分别代表底、中、顶层)
 */
void robot_arm_set_place_index(uint8_t place_idx) {
	if (place_idx < 3) {
		g_place_target_index = place_idx;
	}
}

/**
 * @brief 机械臂应用层初始化
 */
void robot_arm_init(void) {
	// 1. 初始化底层硬件与数据结构
	robot_arm_system_init(&g_robot_arm);
	
	// 2. 传递控制周期 dt 给底层滤波器 (ms 转换为 s)
	robot_arm_set_ctrl_dt(&g_robot_arm, (float)ARM_TASK_PERIOD_MS * 0.001f);

	// 3. 初始状态设定
	g_arm_target_index = 0;
	g_last_target_index = 0;
	g_place_target_index = 0; 
    g_has_dynamic_target = 0;
	robot_arm_apply_target(g_arm_target_index);

#if ARM_USE_REMOTE_KEY
	// 注册遥控器按键弹起事件，用于触发目标点切换
	remote_register_key_callback(ARM_SWITCH_KEY, REMOTE_KEY_PRESS_UP,
								 robot_arm_switch_target);
#endif

	// 4. 将任务创建放到初始化最后，确保参数配置完毕后再启动调度
	xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 3,
				&g_robot_arm_task_handle);
}

/**
 * @brief 机械臂周期控制任务 (FreeRTOS 线程)
 */
void robot_arm_task(void *pvParameters) {
	UNUSED(pvParameters);

	while (1) {
		robot_arm_update(&g_robot_arm);

#if ARM_USE_BOARD_KEY
		if (key_scan(0) == KEY0_PRESS) {
			robot_arm_try_next_state();
		}
#endif
		vTaskDelay(pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
	}
}

/**
 * @brief 尝试切换到下一个状态 (含前置条件校验)
 */
static void robot_arm_try_next_state(void) {
    g_arm_target_index = (g_arm_target_index + 1) % 5; 
    robot_arm_apply_target(g_arm_target_index);
}

/**
 * @brief 应用/下发预设目标点位
 */
void robot_arm_apply_target(uint8_t index) {
	// 自动累加放置层数逻辑 
	if (g_robot_arm.status == ARM_STATE_PLACE && index != 3) {
		g_place_target_index = (g_place_target_index + 1) % 3;
	}

	// 映射状态枚举
	g_robot_arm.status = arm_status_from_index(index);

	// 获取默认数组坐标
	float target_y = g_arm_target_points[index].y;
	float target_z = g_arm_target_points[index].z;
	float target_pitch = g_arm_target_points[index].pitch;

    // 抓取点动态覆盖逻辑
    if (g_robot_arm.status == ARM_STATE_CATCH) {
        if (g_has_dynamic_target) {
            target_y = g_dynamic_target.y;
            target_z = g_dynamic_target.z;
            target_pitch = g_dynamic_target.pitch;
            
            // 下发完坐标后，清空标志位，重新等待下一个坐标
            g_has_dynamic_target = 0; 
        }
    }

	// 放置点独立覆盖逻辑
	if (g_robot_arm.status == ARM_STATE_PLACE) {
		target_y = g_arm_place_points[g_place_target_index].y;
		target_z = g_arm_place_points[g_place_target_index].z;
		target_pitch = g_arm_place_points[g_place_target_index].pitch;
	}

	// 下发给驱动层
	robot_arm_set_target(&g_robot_arm, target_y, target_z, target_pitch);
	g_robot_arm.arm_motion_active = 0; // 重置运动完成标志
	g_last_target_index = index;
}
    
/**
 * @brief 遥控器切换点位回调函数
 */
void robot_arm_switch_target(uint8_t key, remote_key_event_t event) {
	UNUSED(key);

	// 仅在按键抬起时触发，防抖动误触
	if (event != REMOTE_KEY_PRESS_UP) {
		return;
	}

    robot_arm_try_next_state();
}