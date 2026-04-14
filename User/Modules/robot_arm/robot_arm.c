/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块
 *
 * @note 核心流程：目标位置 -> 逆向运动学 -> 梯形轨迹规划 -> CAN MIT控制 -> 反馈闭环
 * @version 1.8
 * @date 2026-04-15
 */

#include "includes.h"

#define ARM_1           450.0f     // 大臂长度
#define ARM_2           450.0f     // 小臂长度
#define ARM_3           105.0f     // 吸盘长度
#define DEFAULT_ANGLE_1 0.1645f    // 大臂初始角度（相对于z轴正方向，逆时针为正）
#define DEFAULT_ANGLE_2 0.1747f    // 大臂与小臂夹角
#define DEFAULT_ANGLE_3 0.9155f    // 吸盘初始角度（相对于x轴正方向，逆时针为正）
#define DEFAULT_X       0.0f
#define DEFAULT_Z       0.0f
#define DEFAULT_ARM_1_2 66.5f      // 大臂与小臂连接处距离，即达妙4340长度
#define DEFAULT_ARM3_X  85.72f     // ARM3和吸盘的直线距离
#define DM_SPEED        0.4f

#define ARM_SWITCH_KEY  10         // 遥控器按键编号，按下后切换到下一个预设点位

typedef struct {
    float y;
    float z;
    float pitch;
} arm_target_point_t;

static RobotArm g_robot_arm;
static uint8_t g_arm_target_index;
static TaskHandle_t g_robot_arm_task_handle;

static const arm_target_point_t g_arm_target_points[4] = {
    {139.95f + 20.0f, 102.70f, 0.9155f},  // 预设零点
    {533.142f, 91.5027f, 0.0f},
    {450.0f, 356.0f, 0.0f},
    {-450.0f, 356.0f, 0.0f},
};


/**
 * @brief 机械臂应用层初始化：创建任务、初始化机构、设置初始点位并注册遥控器按键回调
 * @note 任务句柄和入口函数均由机械臂模块内部管理
 */
void robot_arm_init(void) {
    xTaskCreate(robot_arm_task, "arm_ctrl_task", 512, NULL, 2,
                &g_robot_arm_task_handle);
                
    robot_arm_system_init(&g_robot_arm);

    g_arm_target_index = 0;
    robot_arm_apply_target(g_arm_target_index);

    remote_register_key_callback(ARM_SWITCH_KEY, REMOTE_KEY_PRESS_UP,
                                 robot_arm_switch_target);
}


/**
 * @brief 初始化机械臂系统：电机、CAN、轨迹、状态
 *
 * @param arm 机械臂结构体指针
 */
void robot_arm_system_init(RobotArm *arm) {
    dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_POS_SPEED, DM_J8006, 3.14f, 45.0f,
                  20.0f, can3_selected);
    dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_POS_SPEED, DM_J8006, 3.14f, 45.0f,
                  20.0f, can3_selected);
    dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_POS_SPEED, DM_J4340, 3.14f, 45.0f,
                  20.0f, can3_selected);
    dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_POS_SPEED, DM_J4310, 3.14f, 45.0f,
                  20.0f, can3_selected);

    dm_motor_enable(&arm->damiao_1);
    dm_motor_enable(&arm->damiao_2);
    dm_motor_enable(&arm->damiao_3);
    dm_motor_enable(&arm->damiao_4);

    arm->arm_target_y = 0.0f;
    arm->arm_target_z = 0.0f;
    arm->arm_target_pitch = 0.0f;

    arm->arm_motion_active = 0;
    arm->target_mode = ARM_TARGET_CARTESIAN;

    arm->status = ARM_DEFAULT;
}

/**
 * @brief 机械臂周期任务
 * @note 周期执行目标跟踪与电机控制
 *
 * @param pvParameters 任务参数（未使用）
 */
void robot_arm_task(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        robot_arm_update(&g_robot_arm);
        vTaskDelay(10);
    }
}

/**
 * @brief 根据索引应用预设点位
 *
 * @param index 预设点位索引
 */
void robot_arm_apply_target(uint8_t index) {
    robot_arm_set_target(&g_robot_arm, g_arm_target_points[index].y,
                         g_arm_target_points[index].z,
                         g_arm_target_points[index].pitch);
    g_robot_arm.arm_motion_active = 0;
}

/**
 * @brief 遥控器控制：KEY10 抬起后切换到下一个预设点位
 *
 * @param key 按键编号
 * @param event 按键事件
 */
void robot_arm_switch_target(uint8_t key, remote_key_event_t event) {
    UNUSED(key);

    if (event != REMOTE_KEY_PRESS_UP) {
        return;
    }

    g_arm_target_index = (g_arm_target_index + 1) % 4;
    robot_arm_apply_target(g_arm_target_index);
}

/**
 * @brief 设置机械臂末端目标位置
 *
 * @param arm 机械臂结构体指针
 * @param y 目标Y坐标(mm)
 * @param z 目标Z坐标(mm)
 * @param pitch 目标俯仰角(rad)
 */
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch) {
    arm->arm_target_y = y;
    arm->arm_target_z = z;
    arm->arm_target_pitch = pitch;
    arm->target_mode = ARM_TARGET_CARTESIAN;
}

/**
 * @brief 设置机械臂关节角目标（调试使用，正常情况不调用此函数）
 *
 * @param arm 机械臂结构体指针
 * @param joint1 关节1目标角(rad)
 * @param joint2 关节2目标角(rad)
 * @param joint3 关节3目标角(rad)
 */
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2,
                                float joint3) {
    arm->arm_joint_target[0] = joint1;
    arm->arm_joint_target[1] = joint2;
    arm->arm_joint_target[2] = joint3;
    arm->target_mode = ARM_TARGET_JOINT;
}

/**
 * @brief 机械臂统一周期更新函数
 * @note 统一链路：目标值 -> 逆解 -> 轨迹启动/更新 -> MIT控制
 * @note 轨迹输出为 3 关节角，电机1/2共用第1关节角
 *
 * @param arm 机械臂结构体指针
 */
void robot_arm_update(RobotArm *arm) {
    float angle[3];

    if (arm->target_mode == ARM_TARGET_JOINT) {
        angle[0] = arm->arm_joint_target[0];
        angle[1] = arm->arm_joint_target[1];
        angle[2] = arm->arm_joint_target[2];
    } else {
        arm_pos_angle(arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch,
                      angle);
    }

    arm->arm_motion_active = 0;
    arm_apply_ctrl(arm, angle);
}

/**
 * @brief 按机械臂三关节角下发 MIT 控制
 * @note 电机1与电机2共用大臂角度，其余对应小臂和吸盘
 *
 * @param arm 机械臂结构体
 * @param joint_des 三关节目标角：[0]大臂(用于电机1/2), [1]小臂, [2]吸盘
 */
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {

    dm_pos_speed_ctrl(&arm->damiao_1, joint_des[0], 0.1);
    dm_pos_speed_ctrl(&arm->damiao_2, -joint_des[0], 0.1);
    dm_pos_speed_ctrl(&arm->damiao_3, joint_des[1], DM_SPEED);
    dm_pos_speed_ctrl(&arm->damiao_4, joint_des[2], 0.8);
}

/**
 * @brief 输入指定的末端位置和吸盘姿态，计算 3 个电机的关节角度
 * @note 单位：长度(mm)，角度(rad)
 * @note 传入的x1, z1为相对于基座的坐标，基座即为两个8006的连接中心点
 * @param x1 目标末端的水平前向坐标 (沿X轴)
 * @param z1 目标末端的垂直坐标
 * @param pitch_angle 吸盘末端期望的绝对俯仰角
 * @param angle 输出的三个电机角度：angle[0]大臂, angle[1]小臂, angle[2]吸盘
 */
void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]) {
    float x, z;
    float x_w, z_w; // 小臂与吸盘连接处的坐标
    float m_2, m;
    float a, a2, b, b2;
    float angle1_1, angle1_2;
    float angle2_inner; // 大臂与小臂的实际几何内角

    // 1. 坐标平移 
    x = x1 + DEFAULT_X; 
    z = z1 + DEFAULT_Z;

    // 2. 姿态解耦，求腕关节坐标
    x_w = x - ARM_3 * cosf(pitch_angle) - (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch_angle);
    z_w = z - ARM_3 * sinf(pitch_angle);
    m_2 = x_w * x_w + z_w * z_w;
    arm_sqrt_f32(m_2, &m);

    // 3. 求大臂角度
    b = (ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m);
    arm_sqrt_f32(1 - b * b, &b2);
    arm_atan2_f32(b2, b, &angle1_1);
    arm_atan2_f32(z_w, x_w, &angle1_2);
    angle[0] = DEFAULT_ANGLE_1 + PI/2 - angle1_1 - angle1_2;

    // 4. 求小臂角度
    a = (ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2);
    arm_sqrt_f32(1 - a * a, &a2);
    arm_atan2_f32(a2, a, &angle2_inner);
    angle[1] = angle2_inner - DEFAULT_ANGLE_2;

    if (x_w >= 0) {
        // 第一象限：保持原有的几何构型解
        angle[0] = DEFAULT_ANGLE_1 + PI/2 - angle1_1 - angle1_2;
        angle[1] = angle2_inner - DEFAULT_ANGLE_2;
    } else {
        // 第二象限：切换到另一个解
        angle[0] = DEFAULT_ANGLE_1 + PI/2 + angle1_1 - angle1_2;
        angle[1] = 2 * PI + (DEFAULT_ANGLE_2 - angle2_inner); 
    }

    // 5. 求吸盘角度
    angle[2] = - pitch_angle - angle[0] + angle[1] + DEFAULT_ANGLE_3;
    if(angle[2] < -0.1){
        angle[2] += 2*PI;
    }
    if(angle[2] > 2*PI){
        angle[2] -= 2*PI;
    }
}
