/**
 * @file    chassis.c
 * @author  Dominate0017
 * @brief   底盘控制任务
 * @version 0.3
 * @date    2026-04-5
 */

#include "includes.h"

#define SWITCH_WORLD_KEY      2             /* 切换世界/自身坐标系按键 */
#define SWITCH_AUTO_KEY       3             /* 切换自动/手动按键 */
#define SET_HALT_KEY          4             /* 切换自锁按键 */
#define LIFT_UP_KEY           5             /* 抬升升起按键 */
#define LIFT_DOWN_KEY         6             /* 抬升下降按键 */

#define CHASSIS_CAN_SELECT    can1_selected    
#define LIFT_CAN_SELECT       can2_selected
#define SENSOR_GPIO_PORT GPIOA  
#define FRONT_SENSOR_PIN GPIO_PIN_1
#define REAR_SENSOR_PIN GPIO_PIN_2

typedef enum {
    CHASSIS_MODE_MANUAL = 0,    /* 手动模式 */
    CHASSIS_MODE_AUTO,          /* 自动模式 */
    CHASSIS_MODE_LIFT_SEQ,      /* 抬升动作序列自动接管模式 */
} chassis_mode_t;

typedef enum {
    LIFT_STATE_NORMAL = 0,  /* 达妙不做动作 */
    LIFT_STATE_DOWN,        /* 车子升起 */
    LIFT_STATE_UP,          /* 车子下降 */
} lift_state_t;

typedef struct {
    uint8_t action;  /* 1=抬升, 2=下降 */
    uint8_t step;    /* 当前执行步标 */
    uint32_t timer;  /* 步状态内计时器 */
} chassis_lift_fsm_t;

typedef struct {
    chassis_mode_t mode;                    /* 底盘模式：手动/自动 */
    chassis_speed_t chassis_speed;          /* 底盘各个量纲的速度 */
    float g_current_rotor_degree[4];        /* 当前输出轴角度 */
    bool world_cordinate;                   /* 是否开启世界坐标系   ture:开启 false:关闭 */
    bool degree_lock;                       /* 防止任务中多次记录角度     ture:锁定 false:不锁定 */
    bool halt;                              /* 是否自锁     ture:自锁 false:不自锁 */ 

    lift_state_t lift_state;                 /* 达妙状态 */
    chassis_lift_fsm_t lift_fsm;             /* 抬升动作序列表 */
} chassis_handle_t;

static chassis_handle_t chassis_handle = {
    .mode = CHASSIS_MODE_MANUAL,
    .chassis_speed = {
        .target_speed.vx = 0.0f,
        .target_speed.vy = 0.0f,
        .target_speed.vw = 0.0f,
        .target_2006_rpm = 0.0f
    },
    .world_cordinate = false,
    .degree_lock = false,
    .halt = false,
    .lift_state = LIFT_STATE_NORMAL,
    .lift_fsm = {0, 0, 0}
};

static dji_motor_handle_t dji_3508_handle[4];
static dji_motor_handle_t dji_2006_handle;
static dm_handle_t dm_motor_handle[2];
static pid_t dji_3508_speed_pid[4];
static pid_t dji_3508_pos_pid[4];
static pid_t dji_2006_pid;

static void chassis_tasks_init(void);
static void chassis_bottom_init(void);
static void chassis_switch_mode(uint8_t key, remote_key_event_t event);
static void chassis_halt_degree_update(void);
static void chassis_pid_clear_state(pid_t *pid);

static TaskHandle_t chassis_mode_task_handle;
void chassis_mode_task(void *pvParameters);
static TaskHandle_t chassis_driver_task_handle;
void chassis_driver_task(void *pvParameters);

static bool get_front_photoelectric(void) {
    return HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, FRONT_SENSOR_PIN);
}
static bool get_rear_photoelectric(void) {
    return HAL_GPIO_ReadPin(SENSOR_GPIO_PORT, REAR_SENSOR_PIN);
}

/**
 * @brief 任务一：底盘模式控制任务：选择手动/自动/抬升序列模式
 * @brief 手动：先判断是否自锁，根据遥控器值进行速度规划，若开启世界坐标系，再把规划后的速度变换为自身坐标系的速度
 * @brief 自动：先判断是否自锁，直接把雷达的速度传进来
 * @brief 抬升序列：根据达妙的状态机进行不同阶段的速度规划，配合前后光电传感器完成上升/下降动作序列
 * @param pvParameters 任务传入参数
 */
void chassis_mode_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        /* 判断是否传入升起/下降的信号 */
        if (chassis_handle.mode != CHASSIS_MODE_LIFT_SEQ) {
            if (g_nuc_ctrl_data.lift == 1) {                         /* 1 表示触发上升 */ 
                chassis_handle.mode = CHASSIS_MODE_LIFT_SEQ;
                chassis_handle.lift_fsm.action = 1;
                chassis_handle.lift_fsm.step = 0;
                g_nuc_ctrl_data.lift = 0;
            } else if (g_nuc_ctrl_data.lift == 2) {                 /* 2 表示触发下降 */ 
                chassis_handle.mode = CHASSIS_MODE_LIFT_SEQ;
                chassis_handle.lift_fsm.action = 2;
                chassis_handle.lift_fsm.step = 0;
                g_nuc_ctrl_data.lift = 0;
            }
        }

        switch (chassis_handle.mode) {
            case CHASSIS_MODE_MANUAL: {
                chassis_handle.chassis_speed.target_2006_rpm = 0.0f; // 手动模式下保证2006不转
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
                chassis_handle.chassis_speed.target_2006_rpm = 0.0f; // 自动巡航模式下保证2006不转
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

            case CHASSIS_MODE_LIFT_SEQ: {
                float target_x = 0.0f, target_y = 0.0f, target_yaw = 0.0f;
                chassis_handle.chassis_speed.target_2006_rpm = 0.0f;         /* 默认在此模式中不转，只有在特定步段才驱动 */ 

                /* 上升序列 */
                if (chassis_handle.lift_fsm.action == 1) {
                    target_y = 15.0f;                                       /* 车子到达台阶面前，缓慢向前 */
                    if (chassis_handle.lift_fsm.step == 0) {
                        if (get_front_photoelectric() == true ) {           /* 前光电感应到台阶边缘，停止前进，车子进入升起标志位 */ 
                            target_y = 0.0f;
                        }
                        chassis_handle.lift_state = LIFT_STATE_UP;
                        chassis_handle.lift_fsm.timer = xTaskGetTickCount();
                        chassis_handle.lift_fsm.step = 1;
                    } 
                    else if (chassis_handle.lift_fsm.step == 1) {           /* 给达妙1s，车子升起 */
                        target_y = 0.0f;
                        if ((xTaskGetTickCount() - chassis_handle.lift_fsm.timer) > pdMS_TO_TICKS(1000)) {
                            chassis_handle.lift_fsm.step = 2; 
                        }
                    }                                                       /* 底盘悬空 */  
                    else if (chassis_handle.lift_fsm.step == 2) {
                        target_y = 0.0f;                                    /* 底盘无速度 */
                        chassis_handle.chassis_speed.target_2006_rpm = 1000.0f;                          /* 用2006缓慢向前 */
                        if(get_rear_photoelectric() == true) {              /* 后光电感应到台阶边缘，停止前进，车子进入下降标志位(实际杆子上升) */ 
                            chassis_handle.chassis_speed.target_2006_rpm= 0.0f;                         /* 2006停止运转 */
                            chassis_handle.lift_fsm.step = 3;
                            chassis_handle.lift_state = LIFT_STATE_DOWN;    /* 杆子上升（达妙恢复到30°） */ 
                            chassis_handle.lift_fsm.timer = xTaskGetTickCount();
                        }                                          
                    }
                    else if(chassis_handle.lift_fsm.step == 3) {            /* 给达妙1s，杆子升起 */
                        target_y = 0.0f;
                        if ((xTaskGetTickCount() - chassis_handle.lift_fsm.timer) > pdMS_TO_TICKS(1000)) {
                            chassis_handle.lift_state = LIFT_STATE_NORMAL;
                            chassis_handle.mode = CHASSIS_MODE_MANUAL;      /* 完成上升序列，进入手动模式 */ 
                            chassis_handle.lift_fsm.action = 0;
                        }
                    }
                } 

                /* 下降序列 */
                else if (chassis_handle.lift_fsm.action == 2) {
                    target_y = -15.0f;                                      /* 缓慢向后倒车 */
                    if (chassis_handle.lift_fsm.step == 0) {             
                        if (get_rear_photoelectric() == false) {            /* 后光电感应到台阶边缘，停止后退，车子进入升起标志位(实际是杆子下降) */
                            target_y = 0.0f;
                            chassis_handle.lift_state = LIFT_STATE_UP;      /* 杆子下降至触地平面(达妙转到90°) */
                            chassis_handle.lift_fsm.timer = xTaskGetTickCount();
                            chassis_handle.lift_fsm.step = 1;
                        }
                    } else if (chassis_handle.lift_fsm.step == 1) {         /* 给达妙1s，杆子下降 */
                        target_y = 0.0f;
                        if ((xTaskGetTickCount() - chassis_handle.lift_fsm.timer) > pdMS_TO_TICKS(1000)) {
                            chassis_handle.lift_fsm.step = 2;
                        }
                    } else if (chassis_handle.lift_fsm.step == 2) {         /* 前半段底盘接触台阶，为了统一，不用底盘缓慢移动，用2006 */   
                        target_y = 0.0f;                                    /* 车身悬空时底盘不再响应正常速度 */
                        chassis_handle.chassis_speed.target_2006_rpm = -1000.0f;                         /* 仅用2006向后倒退 */
                        if (get_front_photoelectric() == false) {           /* 前光电感应到台阶边缘，停止后退，车子进入下降标志位 */
                            chassis_handle.chassis_speed.target_2006_rpm = 0.0f;                         /* 2006停止运转 */
                            chassis_handle.lift_state = LIFT_STATE_DOWN;    /* 车子下降（达妙恢复30°收起位置） */  
                            chassis_handle.lift_fsm.timer = xTaskGetTickCount();
                            chassis_handle.lift_fsm.step = 3;
                        }
                    } else if (chassis_handle.lift_fsm.step == 3) {         /* 给达妙1s，车子下降 */
                        target_y = 0.0f;
                        if ((xTaskGetTickCount() - chassis_handle.lift_fsm.timer) > pdMS_TO_TICKS(1000)) {
                             chassis_handle.lift_state = LIFT_STATE_NORMAL;
                            chassis_handle.mode = CHASSIS_MODE_MANUAL;      /* 下降序列完成，进入手动模式 */
                            chassis_handle.lift_fsm.action = 0;
                        }
                    }
                }
                
                chassis_handle.chassis_speed.target_speed.vx = target_x;
                chassis_handle.chassis_speed.target_speed.vy = target_y;
                chassis_handle.chassis_speed.target_speed.vw = target_yaw;
                break;
            }

            default:
                break;
        }

        vTaskDelay(5);
    }
}

/**
 * @brief 任务二：底盘电机驱动任务：对小车速度进行逆解算得到每个轮子的目标RPM并闭环PID电流计算，并通过CAN输出给底层电机驱动
 * @brief 输入：规划后的小车速度(chassis_handle.chassis_speed.target_speed)和电机实际转速(dji_3508_handle[i].speed_rpm)
 * @param pvParameters 任务传入参数
 */
void chassis_driver_task(void *pvParameters)
{
    (void)pvParameters; 
    int16_t motor_out_current[4] = {0};
    bool halt_last = chassis_handle.halt;

    while(1) {
        if (chassis_handle.halt != halt_last) {
            /* 模式切换沿触发时清零 PID 内部状态，避免跨模式继承积分和误差历史 */
            for (int i = 0; i < 4; i++) {
                chassis_pid_clear_state(&dji_3508_speed_pid[i]);
                chassis_pid_clear_state(&dji_3508_pos_pid[i]);
            }

            if (chassis_handle.halt) {
                chassis_handle.degree_lock = false;
            }
            halt_last = chassis_handle.halt;
        }

        if(!chassis_handle.halt) {
            chassis_handle.degree_lock = false;             
            omni_wheels_resolve((const chassis_speed_t*)&chassis_handle.chassis_speed, (volatile float*)chassis_handle.chassis_speed.target_rpm);

            for(int i = 0; i < 4; i++) {
                float real_rpm = dji_3508_handle[i].speed_rpm;
                float target_rpm = chassis_handle.chassis_speed.target_rpm[i]; 
                float calc_current = pid_calc(&dji_3508_speed_pid[i], target_rpm, real_rpm);
                motor_out_current[i] = (int16_t)calc_current;
            }
        } else {
            chassis_halt_degree_update();
            for (int i = 0; i < 4; i++) {
                float target_degree = chassis_handle.g_current_rotor_degree[i];
                float measure_degree = dji_3508_handle[i].rotor_degree;
                float target_rpm = pid_calc(&dji_3508_pos_pid[i], target_degree, measure_degree);
                float measure_rpm = dji_3508_handle[i].speed_rpm;
                float calc_current = pid_calc(&dji_3508_speed_pid[i], target_rpm, measure_rpm);
                motor_out_current[i] = (int16_t)calc_current;
            }
        }

        /* 3508 主控电机 PID 计算与 CAN 下发 */
        dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP1, motor_out_current[0], 
                              motor_out_current[1], motor_out_current[2], motor_out_current[3]);

        /* 2006 辅控电机 PID 计算与 CAN 下发 (使用 0x1FF 即 DJI_MOTOR_GROUP2) */
        float real_2006_rpm = dji_2006_handle.speed_rpm;
        int16_t motor_2006_out = (int16_t)pid_calc(&dji_2006_pid, chassis_handle.chassis_speed.target_2006_rpm, real_2006_rpm);
        dji_motor_set_current(CHASSIS_CAN_SELECT, DJI_MOTOR_GROUP2, motor_2006_out, 0, 0, 0);

        /* 达妙伺服电机位置持续高频发包控制（MIT 模式） */
        if (chassis_handle.lift_state == LIFT_STATE_UP) {
            for (int i = 0; i < 2; i++) {
                dm_mit_ctrl(&dm_motor_handle[i], 90.0f, 10.0f, 10.0f, 0.1f, 1.0f); /* 杆子下降至触地(达妙转到90°) */  
            }
        } else if (chassis_handle.lift_state == LIFT_STATE_DOWN) {
            for (int i = 0; i < 2; i++) {
                dm_mit_ctrl(&dm_motor_handle[i], 30.0f, 10.0f, 10.0f, 0.1f, 1.0f); /* 杆子收起/车子下降(达妙转到30°) */ 
            }
        }

        vTaskDelay(5);
    }
}

/* ==================================================== 底盘和抬升按键注册函数 ==================================================== */

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
                chassis_handle.degree_lock = false;
            } else {
                log_message(LOG_INFO, "Set chassis halt. ");
                chassis_handle.halt = true;
                chassis_handle.degree_lock = false;
            }
        } break;

        default:
            break;
    }
}

static void lift_switch_mode(uint8_t key, remote_key_event_t event) {
    UNUSED(event);
    if (chassis_handle.mode == CHASSIS_MODE_LIFT_SEQ) {
        return;                                             /* 已经处于自动序列流程中，屏蔽操作 */ 
    }
    
    switch (key) {
        case LIFT_UP_KEY: {
            log_message(LOG_INFO, "Trigger Lift UP sequence.");
            chassis_handle.mode = CHASSIS_MODE_LIFT_SEQ;
            chassis_handle.lift_fsm.action = 1;
            chassis_handle.lift_fsm.step = 0;
        } break;

        case LIFT_DOWN_KEY: {
            log_message(LOG_INFO, "Trigger Lift DOWN sequence.");
            chassis_handle.mode = CHASSIS_MODE_LIFT_SEQ;
            chassis_handle.lift_fsm.action = 2;
            chassis_handle.lift_fsm.step = 0;
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
    remote_register_key_callback(LIFT_UP_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode);
    remote_register_key_callback(LIFT_DOWN_KEY, REMOTE_KEY_PRESS_UP,
                                 lift_switch_mode); 
                                 
}

/**
 * @brief 底盘底层初始化，配置电机的CAN ID通信、PID参数以及梯形速度规划器参数
 */
static void chassis_bottom_init(void){
    /* DJI 3508电机初始化 */
    for(int i = 0; i < 4; i++) {
        dji_motor_init(&dji_3508_handle[i], DJI_M3508, CAN_Motor1_ID + i, CHASSIS_CAN_SELECT);
    }
    
    /* DJI 2006电机初始化 */
    dji_motor_init(&dji_2006_handle, DJI_M2006, CAN_Motor5_ID, CHASSIS_CAN_SELECT);
    
    /* 达妙4310初始化 */
    for(int i = 0; i < 2; i++) {
        dm_motor_init(&dm_motor_handle[i], 0x11 + i, i + 1, DM_MODE_MIT, DM_J4310,
                      1.0f, 10.0f, 1.0f,  LIFT_CAN_SELECT);
        dm_motor_enable(&dm_motor_handle[i]);                  
    }

    /* DJI电机PID初始化 */
    for(int i = 0; i < 4; i++) {
        pid_init(&dji_3508_speed_pid[i], 16384.0f, 800.0f, 0.0f, 20000.0f, DELTA_PID, 1.48f, 0.2f, 0.0f);
        pid_init(&dji_3508_pos_pid[i], 3000.0f, 800.0f, 0.0f, 360.0f, DELTA_PID, 120.0f, 0.2f, 1.5f);
    }
    pid_init(&dji_2006_pid, 10000.0f, 500.0f, 0.0f, 15000.0f, DELTA_PID, 2.0f, 0.1f, 0.0f);
    
    /* 规划器初始化 */
    chassis_plan_config_t plan_cfg;
    chassis_plan_config_default(&plan_cfg);
        
    plan_cfg.max_speed_xy = 20.0f;   // 20.0 m/s
    plan_cfg.max_speed_w  = 5.0f;   // 5.0 rad/s
        
    plan_cfg.max_accel_xy = 15.0f;   // 15.0 m/s^2
    plan_cfg.max_accel_w  = 5.0f;   // 5.0 rad/s^2
        
    chassis_plan_init(&plan_cfg, CHASSIS_SPEED_PLAN_TRAPEZOID);
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
        xTaskCreate(chassis_driver_task, "chassis_driver_task", 256,
                    NULL, 4, &chassis_driver_task_handle);
    if (task_create_res != pdPASS) {
        return;
    }                

}

/* ==================================================== 其他功能函数 ==================================================== */
/**
 * @brief 更新自锁时的轮子角度，进入自锁时记录当前轮子角度并锁定，退出自锁时解锁
 * @note 该函数仅在底盘进入自锁状态的第一周期被调用一次，确保在自锁过程中目标角度保持不变
 */

static void chassis_halt_degree_update(void) {
    if (chassis_handle.halt && !chassis_handle.degree_lock) {
        for (int i = 0; i < 4; i++) {
            chassis_handle.g_current_rotor_degree[i] = dji_3508_handle[i].rotor_degree;
        }
        chassis_handle.degree_lock = true;
    }
}

static void chassis_pid_clear_state(pid_t *pid) {
    pid->iout = 0.0f;
    pid->pos_out = 0.0f;

#if PID_USE_DELTA_PID
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
    pid->delta_u = 0.0f;
    pid->delta_out = 0.0f;
    pid->delta_lastout = 0.0f;
#else
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
#endif
}