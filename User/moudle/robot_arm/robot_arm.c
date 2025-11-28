/**
 * @file robot_arm.c
 * @author meiwenhuaqingnian
 * @brief 
 * @version 0.1
 * @date 2025-11-05
 * 
 * 
 */

#include "robot_arm/robot_arm.h"
#include "arm_math.h"

#define ARM_1          490      //单位：mm
#define ARM_2          500      //单位：mm
#define ANGLE2_DEFAULT 0.285744 //16.38°

/**
 * @brief 输入指定的末端位置，计算机械臂的关节角度
 * 
 * @note  单位：mm
 * @param x1 目标末端的x坐标
 * @param y1 目标末端的y坐标
 * @param z1 目标末端的z坐标
 * 
 *                *
 *              * |   *   
 *            *   |  ^    *
 * angle2   *     |angle3   +
 *     ^  +       |         |
 * ****************************************
 * 
 */
void arm_ctrl(float x1, float y1, float z1, float angle[3]) {
    float angle2, angle2_1, angle2_2, angle3;
    float x, y, z;
    float m, m_2, a, a2, b, b2, b3;
    x = x1;
    y = y1 + ARM_2;
    z = ARM_1 + z1;
    // m为辅助线斜边,两个+号连线
    m_2 = x * x + y * y + z * z;
    arm_sqrt_f32(m_2, &m);

    // 云台角度
    arm_atan2_f32(x, y, &angle[0]);

    // 大臂角度
    b = (ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m);
    arm_sqrt_f32(1 - b * b, &b2);
    arm_atan2_f32(b2, b, &angle2_1);
    arm_sqrt_f32(x * x + y * y, &b3);
    arm_atan2_f32(z, b3, &angle2_2);
    angle2 = angle2_1 + angle2_2;
    angle[1] = 3.1415 / 2 - angle2;

    //小臂角度
    a = (ARM_1 * ARM_1 - m_2 + ARM_2 * ARM_2) / (2 * ARM_1 * ARM_2);
    arm_sqrt_f32(1 - a * a, &a2);
    arm_atan2_f32(a2, a, &angle3);
    angle[2] = angle3 - angle[1] - ANGLE2_DEFAULT;
}


int arm_action_set(Trajectory_Handler_t trajectory[3], float current_m1,float current_m2,
                          float current_m3,float target_m1,float target_m2, float target_dm) {

    t_trajectory_init(&trajectory[0], current_m1, target_m1, 10, 20, 0.005);
    t_trajectory_init(&trajectory[1], current_m2, target_m2, 10, 20, 0.005);
    t_trajectory_init(&trajectory[2], current_m3, target_dm, 10, 20, 0.005);

    while (!((trajectory[0].state == FINISHED) && (trajectory[1].state == FINISHED) &&
             (trajectory[2].state == FINISHED))) {}
    return 1;
}
/**
 * @brief 根据关节角度计算末端坐标
 * @param angle 三个关节角度数组 [angle[0]: 云台角度, angle[1]: 大臂角度, angle[2]: 小臂角度]
 * @param x 输出参数：x坐标
 * @param y 输出参数：y坐标
 * @param z 输出参数：z坐标
 * @return 执行状态，0表示成功
 */
uint8_t arm_inverse_solution(float angle[3], float *x, float *y, float *z) {
    // 参数有效性检查
    if (x == NULL || y == NULL || z == NULL) {
        return 1;
    }

    // 提取角度参数
    float theta0 = angle[0] * 6.33; // 云台角度
    float theta1 = angle[2] * 6.33; // 大臂角度
    float theta2 = angle[1] * 6.33; // 小臂角度

    float angle2 = theta1;                           // 大臂与垂直轴的夹角
    float angle3 = theta2 + theta1 + ANGLE2_DEFAULT; // 大臂与小臂的夹角

    // 计算垂直面内的坐标分量
    // 大臂末端在垂直面内的坐标
    float x_arm1 = ARM_1 * arm_sin_f32(angle2);       // 大臂水平分量
    float z_arm1 = ARM_1 * (1 - arm_cos_f32(angle2)); // 大臂垂直分量

    // 小臂相对于垂直轴的角度
    float arm2_angle = angle2 + ANGLE2_DEFAULT;

    float x_arm2 = ARM_2 * arm_cos_f32(arm2_angle);       // 小臂水平分量
    float z_arm2 = ARM_2 * (1 - arm_sin_f32(arm2_angle)); // 小臂垂直分量

    // 垂直面内末端总水平分量
    float b3 = x_arm1 + x_arm2;
    // 垂直面内末端总垂直分量
    float z_total = ARM_2 - z_arm1 - z_arm2;

    // 考虑云台旋转，计算最终的三维坐标
    float y_temp = b3 * arm_cos_f32(theta0);

    // 计算坐标
    *x = b3 * arm_sin_f32(theta0); // x坐标
    *y = y_temp - ARM_2;           // y坐标
    *z = z_total;                  // z坐标

    return 0;
}