/**
 * @file robot_arm.c
 * @author czf,why
 * @brief 
 * @version 1.2
 * @date 2025-11-30
 * 
 * 
 */

#include "robot_arm/robot_arm.h"
#include "arm_math.h"

//Y:-359.0399f, Z:-479.7188
#define ARM_1          490  //单位：mm
#define ARM_2          500 //单位：mm
#define ANGLE2_DEFAULT 0.285744 //16.38°

#define DEFAULT_X          0
#define DEFAULT_Z          245
#define DEFAULT_Y          250

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
void arm_pos_angle(float x1, float y1, float z1, float angle[3]) {
    float angle2, angle2_1, angle2_2, angle3;
    float x, y, z;
    float m, m_2, a, a2, b, b2, b3;
    x = x1;
    y = y1 + DEFAULT_Y;
    z = DEFAULT_Z + z1;
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

/**
 * @brief 机械臂角度驱动：初始化轨迹规划。
 * @param trajectory 轨迹句柄数组
 * @param myarm 机械臂句柄
 * @param target_m1 小臂电机1目标位置
 * @param target_m2 大臂电机2目标位置
 * @param target_dm 云台电机目标位置
 * @return int 总是返回1 (成功启动)
 */
int arm_angle_drive(Trajectory_Handler_t trajectory[3], arm_handle_t *myarm,
                     float target_m1, float target_m2, float target_dm) {

    t_trajectory_init(&trajectory[0], myarm->motor1->Pos, target_m1, 10, 20,
                      0.005);
    t_trajectory_init(&trajectory[1], myarm->motor2->Pos, target_m2, 10, 20,
                      0.005);
    t_trajectory_init(&trajectory[2], myarm->motor3->position, target_dm, 2, 4,
                      0.005);

    // while (!((trajectory[0].state == FINISHED) &&
    //          (trajectory[1].state == FINISHED) &&
    //          (trajectory[2].state == FINISHED)))
    //     ;
    return 1;
}

/**
 * @brief  机械臂末端位置计算：用三个电机反馈的位置计算出机械臂末端位置
 * @param  angle[3]  输入：云台角度、大臂角度、小臂角度（rad）
 * @param  x1        输出：实时x坐标
 * @param  y1        输出：实时y坐标
 * @param  z1        输出：实时z坐标
 */
void arm_angle_pos(float input_angle1, float input_angle2, float input_angle3,
                  float *x1, float *y1, float *z1) {
    float phi = input_angle1;    // 云台角度
    float theta1 = input_angle2; // 大臂角度
    float theta2 = input_angle3; // 小臂角度

    float angle2, angle3;     // 原函数中的中间角度
    float a, b;               // 余弦定理中间值
    float m, m_sq;            // 空间向量模长
    float angle2_1, angle2_2; // 大臂角度分解值
    float r, z;               // xy平面模长、z坐标
    float sin_val, cos_val;   // 三角函数计算
    float sqrt_val;           // 平方根计算

    angle2 = PI / 2.0f - theta1;

    angle3 = theta2 + theta1 + ANGLE2_DEFAULT;

    cos_val = arm_cos_f32(angle3);

    m_sq = ARM_1 * ARM_1 + ARM_2 * ARM_2 - 2 * ARM_1 * ARM_2 * cos_val;
    // 防止根号内为负
    if (m_sq < 0.0f) {
        m_sq = 0.0f;
    }
    arm_sqrt_f32(m_sq, &m);

    if (m < 1e-6f) { // 防止分母为0
        b = 0.0f;
    } else {
        b = (ARM_1 * ARM_1 + m_sq - ARM_2 * ARM_2) / (2 * ARM_1 * m);
    }

    float b_sq = b * b;
    if (b_sq > 1.0f) {
        b_sq = 1.0f; // 防止根号内为负
    }
    arm_sqrt_f32(1.0f - b_sq, &sqrt_val);
    arm_atan2_f32(sqrt_val, b, &angle2_1);

    angle2_2 = angle2 - angle2_1;

    cos_val = arm_cos_f32(angle2_2);
    sin_val = arm_sin_f32(angle2_2);
    r = m * cos_val;
    z = m * sin_val;

    sin_val = arm_sin_f32(phi);
    cos_val = arm_cos_f32(phi);
    float x = r * sin_val;
    float y = r * cos_val;

    *x1 = x;
    *y1 = y;
    *z1 = z;
}

/**
 * @brief 根据关节角度计算末端坐标
 * @param angle 三个关节角度数组 [angle[0]: 云台角度, angle[1]: 大臂角度, angle[2]: 小臂角度]
 * @param x 输出参数：x坐标
 * @param y 输出参数：y坐标
 * @param z 输出参数：z坐标
 * @return 执行状态，0表示成功
 */
uint8_t arm_angle_pos_2(float input_angle1, float input_angle2,
                       float input_angle3, float *x, float *y, float *z) {
    // 参数有效性检查
    if (x == NULL || y == NULL || z == NULL) {
        return 1;
    }

    // 提取角度参数
    float theta0 = input_angle1; // 云台角度
    float theta1 = input_angle2; // 大臂角度
    float theta2 = input_angle3; // 小臂角度

    float angle2 = theta1; // 大臂与垂直轴的夹角

    // 计算垂直面内的坐标分量
    // 大臂末端在垂直面内的坐标
    float x_arm1 = ARM_1 * arm_sin_f32(angle2);       // 大臂水平分量
    float z_arm1 = ARM_1 * (1 - arm_cos_f32(angle2)); // 大臂垂直分量

    // 小臂相对于垂直轴的角度
    float arm2_angle = theta2 + ANGLE2_DEFAULT;

    float x_arm2 = ARM_2 * arm_sin_f32(arm2_angle); // 小臂水平分量
    float z_arm2 = ARM_2 * arm_cos_f32(arm2_angle); // 小臂垂直分量

    // 垂直面内末端总水平分量
    float b3 = x_arm1 + x_arm2;
    // 垂直面内末端总垂直分量
    float z_total = -z_arm1 - z_arm2;

    // 考虑云台旋转，计算最终的三维坐标
    float y_temp = b3 * arm_cos_f32(theta0);

    // 计算坐标
    *x = b3 * arm_sin_f32(theta0); // x坐标
    *y = y_temp;                   // y坐标
    *z = z_total + ARM_1;          // z坐标

    return 0;
}