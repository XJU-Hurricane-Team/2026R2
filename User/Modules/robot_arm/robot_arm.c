/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块 - 逆解 + 轨迹规划 + MIT控制闭环
 * 
 * @note 核心流程：目标位置 → 逆向运动学 → 梯形轨迹规划 → CAN MIT控制 → 反馈闭环
 * @note 调用链：robot_arm_system_init() | robot_arm_test_update() → arm_pos_angle() 
 *         → arm_start_traj_motion() → robot_arm_update() → arm_apply_ctrl() 
 *         → dm_mit_ctrl() 
 * @version 1.3
 * @date 2026-04-04
 */

#include "robot_arm/robot_arm.h"
#include "arm_math.h"
#include <math.h>

#define ARM_1          100
#define ARM_2          100
#define ARM_3          50.0f //吸盘电机轴心到吸盘作用点的直线距离
#define DEFAULT_Y      0.0f  
#define DEFAULT_Z      0.0f 

#define ARM_TEST_DT    0.003f
#define ARM_TEST_V_MAX 0.5f
#define ARM_TEST_A_MAX 0.2f

#define DM_DEBUG_DT    0.005f
#define DM_DEBUG_V_MAX 0.3f
#define DM_DEBUG_A_MAX 0.1f

/**
 * @brief 重置轨迹组中所有轨迹的状态和时间参数
 * @note 此函数在初始化或新动作开始时调用，将所有轨迹状态恢复到FINISHED
 * 
 * @param traj 轨迹数组指针
 * @param start 起始位置数组
 * @param goal 目标位置数组
 * @param count 轨迹/位置的个数
 */
static void traj_group_reset(Trajectory_Handler_t *traj, float *start, float *goal, int count) {
    for (int i = 0; i < count; i++) {
        start[i] = 0.0f;
        goal[i] = 0.0f;
        traj[i].state = FINISHED;
        traj[i].current_time = 0.0;
        traj[i].total_time = 0.0;
    }
}

/**
 * @brief 批量更新多条梯形轨迹，计算期望位置和速度
 * @note 每周期调用一次，轨迹内部时间自动累加，到达终点后自动停止
 * 
 * @param traj 轨迹数组
 * @param count 轨迹个数
 * @param[out] p_des 期望位置输出数组
 * @param[out] w_des 期望速度输出数组
 * @return 运动状态综合标志 (0:所有轨迹完成, 非0:至少有一条还在运动)
 */
static uint8_t traj_update_all(Trajectory_Handler_t *traj, int count, float *p_des, float *w_des) {
    uint8_t running = 0;

    for (int i = 0; i < count; i++) {
        running |= (uint8_t)t_trajectory_update(&traj[i], &p_des[i], &w_des[i]);
    }

    return running;
}

/**
 * @brief 下发MIT控制指令到4个电机
 * 
 * @param arm 机械臂结构体
 */
static void arm_apply_ctrl(RobotArm *arm, const float p_des[4]) {
    dm_mit_ctrl(&arm->damiao_1, p_des[0], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_2, p_des[1], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_3, p_des[2], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_4, p_des[3], 0.0f, 60.0f, 0.01f, 0.0f);
}

/**
 * @brief 启动机械臂轨迹运动，初始化3个关节的梯形轨迹
 * @note 读取电机当前位置作为起点，计算运动时间
 * 
 * @param arm 机械臂结构体
 * @param goal[3] 三轴目标关节角：[0]大臂, [1]小臂, [2]吸盘
 */
static void arm_start_traj_motion(RobotArm *arm, const float goal[3]) {
    arm->arm_start_pos[0] = arm->damiao_1.position;
    arm->arm_start_pos[1] = arm->damiao_3.position;
    arm->arm_start_pos[2] = arm->damiao_4.position;

     for (int i = 0; i < 3; i++) {
        arm->arm_goal_pos[i] = goal[i];
        t_trajectory_init(&arm->arm_traj[i], arm->arm_start_pos[i], arm->arm_goal_pos[i],
                          ARM_TEST_V_MAX, ARM_TEST_A_MAX, ARM_TEST_DT);
    }

    arm->arm_motion_active = 1;
}

/**
 * @brief 启动4个电机独立轨迹运动（机械臂调试模式）
 * @note 读取电机当前位置作为起点，4个电机按各自目标位置运动
 * 
 * @param arm 机械臂结构体
 * @param goal 目标位置数组
 */
static void dm_start_traj_motion(RobotArm *arm, const float goal[4]) {
    arm->dm_start_pos[0] = arm->damiao_1.position;
    arm->dm_start_pos[1] = arm->damiao_2.position;
    arm->dm_start_pos[2] = arm->damiao_3.position;
    arm->dm_start_pos[3] = arm->damiao_4.position;

    for (int i = 0; i < 4; i++) {
        arm->dm_goal_pos[i] = goal[i];
        t_trajectory_init(&arm->dm_traj[i], arm->dm_start_pos[i], arm->dm_goal_pos[i],
                          DM_DEBUG_V_MAX, DM_DEBUG_A_MAX, DM_DEBUG_DT);
    }

    arm->dm_motion_active = 1;
}

/**
 * @brief 初始化机械臂系统：电机、CAN、轨迹、状态
 * 
 * @param arm 机械臂结构体指针
 */
void robot_arm_system_init(RobotArm *arm) {
    dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_MIT, DM_J8006, 12.5f, 45.0f,
                  10.0f, can1_selected);
    dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_MIT, DM_J8006, 12.5f, 45.0f,
                  10.0f, can1_selected);
    dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_MIT, DM_J4340, 12.5f, 45.0f,
                  10.0f, can1_selected);
    dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_MIT, DM_J4310, 12.5f, 45.0f,
                  10.0f, can1_selected);

    dm_motor_enable(&arm->damiao_1);
    dm_motor_enable(&arm->damiao_2);
    dm_motor_enable(&arm->damiao_3);
    dm_motor_enable(&arm->damiao_4);
    // 初始化运动状态
    arm->arm_target_y = 200.0f;
    arm->arm_target_z = 100.0f;
    arm->arm_target_pitch = 0.0f;
    for (int i = 0; i < 4; i++) {
        arm->dm_debug_target_position[i] = 0.0f;
    }

    traj_group_reset(arm->arm_traj, arm->arm_start_pos, arm->arm_goal_pos, 3);
    arm->arm_motion_active=0;

    traj_group_reset(arm->dm_traj, arm->dm_start_pos, arm->dm_goal_pos, 4);
    arm->dm_motion_active=0;

    // 初始化状态
    arm->status = ARM_DEFAULT;
}

/**
 * @brief 机械臂测试更新函数（按键模式）
 * @note 按键改目标 - 逆解 - 启动轨迹 - 周期更新轨迹 - MIT控制
 * 
 * @param arm 机械臂结构体指针
 */
void robot_arm_test_update(RobotArm *arm) {
    float angle[3];

    arm_pos_angle(arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, angle);

    if (arm->arm_motion_active == 0) {
        arm_start_traj_motion(arm, angle);
    }

    robot_arm_update(arm);
}

/**
 * @brief 电机调试更新函数（独立模式）
 * @note 用于单独调试电机参数，绕过逆解直接下发每个电机独立目标位置
 * 
 * @param arm 机械臂结构体指针
 * @note 目标位置需在外部设置：arm->dm_debug_target_position[motor_index] = value
 * @warning 此函数当前未被主流程调用，预留为扩展接口
 */
void robot_arm_dm_debug_update(RobotArm *arm) {
    float p_des[4], w_des[4];

    if (arm->dm_motion_active == 0) {
        dm_start_traj_motion(arm, arm->dm_debug_target_position);
    }

    arm->dm_motion_active = traj_update_all(arm->dm_traj, 4, p_des, w_des);

    arm_apply_ctrl(arm, p_des);
}

/**
 * @brief 输入指定的末端位置和吸盘姿态，计算3个电机的关节角度
 * @note  单位：长度(mm)，角度(rad)
 * @param y1 目标末端的水平前向坐标 
 * @param z1 目标末端的垂直坐标
 * @param pitch_angle 吸盘末端期望的绝对俯仰角
 * @param angle 输出的三个电机角度：angle[0]大臂, angle[1]小臂, angle[2]吸盘
 */
void arm_pos_angle(float y1, float z1, float pitch_angle, float angle[3]) {
    float y, z;
    float y_w, z_w; // 小臂与吸盘连接处的坐标
    float m_2, m;
    float a, a2, b, b2;
    float angle1_1, angle1_2;
    float angle2_inner;

    y = y1 + DEFAULT_Y;
    z = z1 + DEFAULT_Z;

    y_w = y - ARM_3 * cosf(pitch_angle); 
    z_w = z - ARM_3 * sinf(pitch_angle);
    m_2 = y_w * y_w + z_w * z_w;
    arm_sqrt_f32(m_2, &m);

    b = (ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m);
    arm_sqrt_f32(1 - b * b, &b2);
    arm_atan2_f32(b2, b, &angle1_1); 
    arm_atan2_f32(z_w, y_w, &angle1_2); 
    angle[0] = angle1_2 + angle1_1; 

    a = (ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2);
    arm_sqrt_f32(1 - a * a, &a2);
    arm_atan2_f32(a2, a, &angle2_inner); 

    angle[1] = - PI + angle2_inner;
    angle[2] = pitch_angle - angle[0] - angle[1];
}

/**
 * @brief 设置机械臂末端目标位置（仅更新目标值）
 * @note 此函数仅记录目标，逆解和轨迹启动由test_update函数处理
 * @note 按键/遥控器调用此函数修改目标，触发下一周期轨迹重新规划
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
}

/**
 * @brief 机械臂事件接口：移动到准备位置
 * @note 预留扩展接口，用于后续上位机/状态机完善
 * @note 当前主流程未直接调用，仅在按键测试与序列控制时触发
 * @note 准备位置为（y=0, z=0, pitch=0）末端位置
 * 
 * @param arm 机械臂结构体指针
 * @see robot_arm_event_catch() 机械臂抓取事件
 */
void robot_arm_event_ready(RobotArm *arm) {
    float angle[3];
    
    // 计算准备位置（0,0,0）的关节角
    arm_pos_angle(0.0f, 0.0f, 0.0f, angle);
    
    arm_start_traj_motion(arm, angle);
    arm->status = ARM_MOVING_TO_READY;
}

/**
 * @brief 机械臂事件：抓取位置
 * 
 * @param arm 机械臂结构体指针
 * @param y 目标Y坐标(mm)
 * @param z 目标Z坐标(mm)
 * @see robot_arm_event_ready() 机械臂准备位置事件
 */
void robot_arm_event_catch(RobotArm *arm, float y, float z) {
    float angle[3];
    
    // 计算抓取位置的关节角
    arm_pos_angle(y, z, 0.0f, angle);
    
    arm_start_traj_motion(arm, angle);
    arm->status = ARM_CATCHING;
}

/**
 * @brief 机械臂运动周期更新函数
 * @param arm 机械臂结构体指针
 * @return 无返回值；更新arm->arm_motion_active和arm->arm_traj[x]状态
 * @warning 调用前需确保arm_motion_active=1，通常由robot_arm_test_update或event接口启动
 * @see robot_arm_test_update() 外层测试/控制接口
 * @see arm_apply_ctrl() 电机控制发令体
 */
void robot_arm_update(RobotArm *arm) {
    float p_des[3], w_des[3];
    float p_des_4[4];

    if (arm->arm_motion_active == 0) {
        return;
    }

    arm->arm_motion_active = traj_update_all(arm->arm_traj, 3, p_des, w_des);

    p_des_4[0] = p_des[0];
    p_des_4[1] = p_des[0];
    p_des_4[2] = p_des[1];
    p_des_4[3] = p_des[2];

    arm_apply_ctrl(arm, p_des_4);
}
