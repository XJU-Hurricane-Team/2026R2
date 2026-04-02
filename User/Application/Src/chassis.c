/**
 * @file    chassis.c
 * @author  Dominate0017
 * @brief   底盘控制任务
 * @version 0.1
 * @date    2026-04-2
 */

#include "includes.h"

#define SWITCH_WORLD_KEY      2             /* 切换世界/自身坐标系按键 */
#define SWITCH_AUTO_KEY       3             /* 切换自动/手动按键 */
#define SET_HALT_KEY          4             /* 切换自锁按键 */

typedef enum {
    CHASSIS_MODE_MANUAL = 0,
    CHASSIS_MODE_AUTO,
} chassis_mode_t;

typedef struct {
    chassis_mode_t mode;                    /* 底盘模式：手动/自动 */
    chassis_speed_t chassis_speed;          /* 底盘各个量纲的速度 */
    bool world_cordinate;                   /* 是否开启世界坐标系   ture:开启 false:关闭 */
    bool halt;                              /* 是否自锁     ture:自锁 false:不自锁 */ 
} chassis_handle_t;

static chassis_handle_t chassis_handle = {
    .mode = CHASSIS_MODE_MANUAL,
    .chassis_speed = {
        .target_speed.vx = 0,
        .target_speed.vy = 0,
        .target_speed.vw = 0,
    },
    .world_cordinate = false,
    .halt = true,
};

static dji_motor_handle_t dji_motor_handle[4];
static pid_t motor_pid[4];

static void chassis_tasks_init(void);
static void chassis_bottom_init(void);
static void chassis_switch_mode(uint8_t key, remote_key_event_t event);

static TaskHandle_t chassis_mode_task_handle;
void chassis_mode_task(void *pvParameters);
static TaskHandle_t chassis_calculation_task_handle;
void chassis_calculation_task(void *pvParameters);
static TaskHandle_t chassis_driver_task_handle;
void chassis_driver_task(void *pvParameters);

/**
 * @brief 任务一：底盘模式控制任务：选择手动/自动
 * @brief 手动：先判断是否自锁，根据遥控器值进行速度规划，若开启世界坐标系，再把规划后的速度变换为自身坐标系的速度
 * @brief 自动：直接把雷达的速度传进
 * @brief 输入：遥控器输入速度(g_remote_ctrl_data)或雷达规划速度(g_nuc_ctrl_data)
 * @brief 输出：规划后的小车速度(chassis_handle.chassis_speed.target_speed)
 * @param pvParameters 任务传入参数
 */
void chassis_mode_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        switch (chassis_handle.mode) {
            case CHASSIS_MODE_MANUAL: {
                float target_x = g_remote_ctrl_data.rs[0];
                float target_y = g_remote_ctrl_data.rs[1];
                float target_yaw = g_remote_ctrl_data.rs[2];
                
                if(chassis_handle.halt) {
                    target_x = 0.0f;
                    target_y = 0.0f;
                    target_yaw = 0.0f;
                }
                chassis_plan_step(target_x, target_y, target_yaw,
                              &chassis_handle.chassis_speed.target_speed.vx,
                              &chassis_handle.chassis_speed.target_speed.vy,
                              &chassis_handle.chassis_speed.target_speed.vw);

                if(chassis_handle.world_cordinate) {
                    omni_wheels_world_transform(&chassis_handle.chassis_speed.target_speed, g_nuc_pos_data.yaw);   //第二个参数待填
                }
                break;
            }

            case CHASSIS_MODE_AUTO: {
                float vx = g_nuc_ctrl_data.v * cosf(g_nuc_ctrl_data.yaw);
                float vy = g_nuc_ctrl_data.v * sinf(g_nuc_ctrl_data.yaw);
                float vw = g_nuc_ctrl_data.vw;

                if (chassis_handle.halt) {
                    vx = 0.0f;
                    vy = 0.0f;
                    vw = 0.0f;
                }
                chassis_handle.chassis_speed.target_speed.vx = vx;
                chassis_handle.chassis_speed.target_speed.vy = vy;
                chassis_handle.chassis_speed.target_speed.vw = vw;
                break;
            }
            default:
                break;
        }

        vTaskDelay(5);
    }
}

/**
 * @brief 任务二：底盘运动学解算任务：将规划后的小车目标速度逆解算为各个转子的目标电机的理论转速
 * @brief 输入：规划后的小车速度(chassis_handle.chassis_speed.target_speed)
 * @brief 输出：规划后每个轮子的转速(chassis_handle.chassis_speed.target_rpm)
 * @param pvParameters 任务传入参数
 */
void chassis_calculation_task(void *pvParameters)
{
    (void)pvParameters; 

    while(1) {
        omni_wheels_resolve((const chassis_speed_t*)&chassis_handle.chassis_speed, (volatile float*)chassis_handle.chassis_speed.target_rpm);
        vTaskDelay(5);
    }
}

/**
 * @brief 任务三：底盘电机驱动任务：进行转速闭环PID电流计算，并通过CAN输出给底层电机驱动
 * @brief 输入：规划后每个轮子的转速(chassis_handle.chassis_speed.target_rpm)和电机实际转速(dji_motor_handle[i].speed_rpm)
 * @param pvParameters 任务传入参数
 */
void chassis_driver_task(void *pvParameters)
{
    (void)pvParameters; 
    int16_t motor_out_current[4] = {0};
    
    while(1) {
        for(int i = 0; i < 4; i++) {
            float real_rpm = dji_motor_handle[i].speed_rpm;
            float target_rpm = chassis_handle.chassis_speed.target_rpm[i]; 
            float calc_current = pid_calc(&motor_pid[i], target_rpm, real_rpm);
            motor_out_current[i] = (int16_t)calc_current;
        }

        dji_motor_set_current(can1_selected, 0x200, motor_out_current[0], 
                              motor_out_current[1], motor_out_current[2], motor_out_current[3]);
        vTaskDelay(5);
    }
}

/* ==================================================== 底盘模式切换相关函数 ==================================================== */

/**
 * @brief 切换底盘模式、坐标系以及启停自锁
 *
 * @param key 触发的按键键码
 * @param event 遥控器输入事件（如释放、按下）
 */
static void chassis_switch_mode(uint8_t key, remote_key_event_t event) {
    UNUSED(event);
    switch (key) {
        case SWITCH_AUTO_KEY: {
            if (chassis_handle.mode == CHASSIS_MODE_AUTO) {
                log_message(LOG_INFO, "Switch to manual mode. ");
                chassis_handle.mode = CHASSIS_MODE_MANUAL;
            } else {
                log_message(LOG_INFO, "Switch to auto mode. ");
                chassis_handle.mode = CHASSIS_MODE_AUTO;
            }
        } break;

        case SWITCH_WORLD_KEY: {
            if (chassis_handle.world_cordinate) {
                log_message(LOG_INFO, "Set chassis to self coordinate. ");
                chassis_handle.world_cordinate = false;
            } else {
                log_message(LOG_INFO, "Set chassis to world coordinate. ");
                chassis_handle.world_cordinate = true;
            }
        } break;

        case SET_HALT_KEY: {
            if (chassis_handle.halt) {
                log_message(LOG_INFO, "Release chassis halt. ");
                chassis_handle.halt = false;
            } else {
                log_message(LOG_INFO, "Set chassis halt. ");
                chassis_handle.halt = true;
            }
        } break;
        default:
            break;
    }
}

/* ==================================================== 初始化相关函数 ====================================================*/

/**
 * @brief 底盘控制应用层初始化，包括底层参数配置、RTOS任务调度创建及遥控器按键回调注册
 */
void chassis_init(void){
    /* 底盘底层初始化 */
    chassis_bottom_init();

    /* 底盘控制任务初始化 */
    chassis_tasks_init();

    /* 按键注册 */
    remote_register_key_callback(SWITCH_WORLD_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_switch_mode);
    remote_register_key_callback(SWITCH_AUTO_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_switch_mode);
    remote_register_key_callback(SET_HALT_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_switch_mode);
                                 
}

/**
 * @brief 底盘底层初始化，配置电机的CAN ID通信、PID参数以及梯形速度规划器参数
 */
static void chassis_bottom_init(void){
      /* DJI电机初始化 */
    for(int i = 0; i < 4; i++) {
        dji_motor_init(&dji_motor_handle[i], DJI_M3508, CAN_Motor1_ID + i, can1_selected);
    }

    /* DJI电机PID初始化 */
    for(int i = 0; i < 4; i++) {
        pid_init(&motor_pid[i], 16384.0f, 800.0f, 0.0f, 20000.0f, DELTA_PID, 1.48f, 0.2f, 0.0f);
    }

    /* 规划器初始化 */
    for(int i = 0; i < 4; i++) {
        chassis_plan_config_t plan_cfg;
        chassis_plan_config_default(&plan_cfg);
        
        plan_cfg.max_speed_xy = 10.0f;   // 10.0 m/s
        plan_cfg.max_speed_w  = 5.0f;   // 5.0 rad/s
        
        plan_cfg.max_accel_xy = 0.75f;   // 0.75 m/s^2
        plan_cfg.max_accel_w  = 0.75f;   // 0.75 rad/s^2
        
        chassis_plan_init(&plan_cfg, CHASSIS_SPEED_PLAN_TRAPEZOID);
    }
}

/**
 * @brief 初始化底盘控制相关任务
 * 
 */
static void chassis_tasks_init(void) {
    BaseType_t task_create_res = pdFAIL;

    task_create_res = xTaskCreate(chassis_mode_task, "chassis_mode_task", 256,
                                  NULL, 4, &chassis_mode_task_handle);
    if (task_create_res != pdPASS) {
        return;
    }
    task_create_res =
        xTaskCreate(chassis_calculation_task, "chassis_calculation_task", 256,
                    NULL, 4, &chassis_calculation_task_handle);
    if (task_create_res != pdPASS) {
        return;
    }
    task_create_res =
        xTaskCreate(chassis_driver_task, "chassis_driver_task", 256,
                    NULL, 4, &chassis_driver_task_handle);

}