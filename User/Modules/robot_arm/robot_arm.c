/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块
 *
 * @note 核心流程：目标位置 -> 逆向运动学 -> 梯形轨迹规划 -> CAN MIT控制 -> 反馈闭环
 * @version 1.4
 * @date 2026-04-06
 */

#include "robot_arm/robot_arm.h"

#define ARM_1          100
#define ARM_2          100
#define ARM_3          50.0f // 吸盘电机轴心到吸盘作用点的直线距离
#define DEFAULT_Y      0.0f
#define DEFAULT_Z      0.0f

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

    arm->arm_target_y = 200.0f;
    arm->arm_target_z = 100.0f;
    arm->arm_target_pitch = 0.0f;

    traj_group_reset(arm->arm_traj, arm->arm_start_pos, arm->arm_goal_pos, 3);
    arm->arm_motion_active = 0;

    arm->status = ARM_DEFAULT;
}

/**
 * @brief 重置轨迹组中所有轨迹的状态和时间参数
 * @note 此函数在初始化或新动作开始时调用，将所有轨迹状态恢复到 FINISHED
 *
 * @param traj 轨迹数组指针
 * @param start 起始位置数组
 * @param goal 目标位置数组
 * @param count 轨迹/位置的个数
 */
void traj_group_reset(Trajectory *traj, float *start, float *goal, int count) {
    for (int i = 0; i < count; i++) {
        start[i] = 0.0f;
        goal[i] = 0.0f;
        traj[i].state = FINISHED;
        traj[i].current_time = 0.0;
        traj[i].total_time = 0.0;
    }
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
}

/**
 * @brief 启动机械臂轨迹运动，初始化 3 个关节的梯形轨迹
 * @note 读取电机当前位置作为起点，计算运动时间
 *
 * @param arm 机械臂结构体
 * @param goal[3] 三轴目标关节角：[0]大臂, [1]小臂, [2]吸盘
 */
void arm_start_traj_motion(RobotArm *arm, const float goal[3]) {
    arm->arm_start_pos[0] = arm->damiao_1.position;
    arm->arm_start_pos[1] = arm->damiao_3.position;
    arm->arm_start_pos[2] = arm->damiao_4.position;

    for (int i = 0; i < 3; i++) {
        arm->arm_goal_pos[i] = goal[i];
        t_trajectory_init(&arm->arm_traj[i], arm->arm_start_pos[i], arm->arm_goal_pos[i],
                          5, 10, 0.003);
    }

    arm->arm_motion_active = 1;
}

/**
 * @brief 批量更新多条梯形轨迹，计算期望位置
 * @note 每周期调用一次，轨迹内部时间自动累加，到达终点后自动停止
 *
 * @param traj 轨迹数组
 * @param count 轨迹个数
 * @param[out] p_des 期望位置输出数组
 * @return 运动状态综合标志 (0:所有轨迹完成, 非0:至少有一条还在运动)
 */
static uint8_t traj_update_all(Trajectory *traj, int count, float *p_des) {
    uint8_t running = 0;
    float w_des;

    for (int i = 0; i < count; i++) {
        running |= (uint8_t)t_trajectory_update(&traj[i], &p_des[i], &w_des);
    }

    return running;
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
    float p_des[3];

    arm_pos_angle(arm->arm_target_y, arm->arm_target_z, arm->arm_target_pitch, angle);

    if (arm->arm_motion_active == 0) {
        arm_start_traj_motion(arm, angle);
    }

    arm->arm_motion_active = traj_update_all(arm->arm_traj, 3, p_des);
    arm_apply_ctrl(arm, p_des);
}

/**
 * @brief 按机械臂三关节角下发 MIT 控制
 * @note 电机1与电机2共用大臂角度，其余对应小臂和吸盘
 *
 * @param arm 机械臂结构体
 * @param joint_des 三关节目标角：[0]大臂(用于电机1/2), [1]小臂, [2]吸盘
 */
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {
    dm_mit_ctrl(&arm->damiao_1, joint_des[0], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_2, joint_des[0], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_3, joint_des[1], 0.0f, 60.0f, 0.01f, 0.0f);
    dm_mit_ctrl(&arm->damiao_4, joint_des[2], 0.0f, 60.0f, 0.01f, 0.0f);
}

/**
 * @brief 输入指定的末端位置和吸盘姿态，计算 3 个电机的关节角度
 * @note 单位：长度(mm)，角度(rad)
 *
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