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
#include "./unitree_motor/unitree_motor.h"
#include "arm_math.h"

#define ARM_1          490
#define ARM_2          500
#define ANGLE2_DEFAULT 0.285744 //16.38°


/**
 * @brief 
 * 
 * @param x1 
 * @param y1 
 * @param z1 
 * 
 *                *
 *              * |   *   
 *            *   |  ^    *
 * angle2   *     |angle3   +
 *     ^  +       |         |
 * ****************************************
 * @return uint8_t 
 */
void arm_ctrl(float x1, float y1, float z1,float angle[3]) {
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
    angle[1] = 3.1415/2 - angle2;

    //小臂角度
    a = (ARM_1 * ARM_1 - m_2 + ARM_2 * ARM_2) / (2 * ARM_1 * ARM_2);
    arm_sqrt_f32(1 - a * a, &a2);
    arm_atan2_f32(a2, a, &angle3);
    angle[2] = angle3 - angle[1] - ANGLE2_DEFAULT;

}

uint8_t arm_forward_kinematics(dm_handle_t *pan_motor, 
                                       unitree_motor_handle_t *arm1_motor,
                                       unitree_motor_handle_t *arm2_motor,
                                       float *x1, float *y1, float *z1) {
    // 1. 入参检查
    if (!pan_motor || !arm1_motor || !arm2_motor || !x1 || !y1 || !z1) {
        return 1;
    }

    // 2. 提取电机反馈角度（单位：rad，与原逆解一致）
    float theta0 = pan_motor->position;   // 云台旋转角
    float theta1 = arm1_motor->Pos;       // 大臂电机角（对应原逆解的angle[1]）
    float theta2 = arm2_motor->Pos;       // 小臂电机角（对应原逆解的angle[2]）

    // 3. 还原原逆解中的几何角度
    float angle2 = PI/2.0f - theta1;      // 大臂与垂直轴的夹角（原逆解的angle2）
    float angle3 = theta2 + theta1 + ANGLE2_DEFAULT; // 大臂与小臂的夹角（原逆解的angle3）

    // 4. 垂直面内二连杆坐标计算（核心修正：小臂角度为“大臂角度 + (π - 夹角angle3)”）
    // 大臂末端在垂直面内的坐标（水平: x_arm，垂直: z_arm）
    float x_arm1 = ARM_1 * arm_sin_f32(angle2);  // 大臂水平分量
    float z_arm1 = ARM_1 * arm_cos_f32(angle2);  // 大臂垂直分量
    // 小臂相对于垂直轴的角度：大臂角度 + (π - 大臂小臂夹角)（三角形内角和为π）
    float arm2_angle = angle2 + (PI - angle3);
    // 小臂末端相对于大臂末端的坐标
    float x_arm2 = ARM_2 * arm_sin_f32(arm2_angle); // 小臂水平分量
    float z_arm2 = ARM_2 * arm_cos_f32(arm2_angle); // 小臂垂直分量

    // 5. 垂直面内末端总坐标
    float b3 = x_arm1 + x_arm2;          // 水平投影长度（对应原逆解的b3）
    float z_total = z_arm1 + z_arm2;     // 垂直总高度（对应原逆解的z）

    // 6. 云台旋转映射到三维坐标（与原逆解完全对齐）
    float x = b3 * arm_sin_f32(theta0);  // 原逆解：angle[0] = atan2(x, y)
    float y = b3 * arm_cos_f32(theta0);  // 原逆解：y = y1 + ARM_2

    // 7. 还原为目标坐标（与原逆解的输入x1/y1/z1一致）
    *x1 = x;
    *y1 = y ;                     // 原逆解：y = y1 + ARM_2 → 反向推导
    *z1 = z_total ;               // 原逆解：z = ARM_1 + z1 → 反向推导

    return 0;
}

/**
 * @brief 根据关节角度计算末端执行器坐标
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
    float theta0 = angle[0]*6.33;  // 云台角度
    float theta1 = angle[1]*6.33;  // 大臂角度
    float theta2 = angle[2]*6.33;  // 小臂角度
    
    // 还原几何角度（与arm_ctrl函数中的计算对应）
    float angle2 = 3.1415/2 - theta1;  // 大臂与垂直轴的夹角
    float angle3 = theta2 + theta1 + ANGLE2_DEFAULT;  // 大臂与小臂的夹角
    
    // 计算垂直面内的坐标分量
    // 大臂末端在垂直面内的坐标
    float x_arm1 = ARM_1 * arm_sin_f32(angle2);  // 大臂水平分量
    float z_arm1 = ARM_1 * arm_cos_f32(angle2);  // 大臂垂直分量
    
    // 小臂相对于垂直轴的角度：大臂角度 + (π - 大臂小臂夹角)
    float arm2_angle = angle2 + (3.1415 - angle3);
    
    // 小臂末端相对于大臂末端的坐标
    float x_arm2 = ARM_2 * arm_cos_f32(arm2_angle);  // 小臂水平分量
    float z_arm2 = ARM_2 * arm_sin_f32(arm2_angle);  // 小臂垂直分量
    
    // 垂直面内末端总水平分量
    float b3 = x_arm1 + x_arm2;
    // 垂直面内末端总垂直分量
    float z_total = z_arm1 + z_arm2;
    
    // 考虑云台旋转，计算最终的三维坐标
    // 应用云台旋转角度，将垂直面内坐标映射到三维空间
    float y_temp = b3 * arm_cos_f32(theta0);
    
    // 根据arm_ctrl函数中的坐标变换关系，反向计算输入坐标
    *x = b3 * arm_sin_f32(theta0);  // x坐标
    *y = y_temp - ARM_2;            // y坐标（考虑ARM_2偏移）
    *z = z_total - ARM_1;           // z坐标（考虑ARM_1偏移）
    
    return 0;
}

/**
 * @brief 封装函数：读取电机反馈并计算末端坐标（简化调用）
 * @param pan_motor     云台电机句柄
 * @param arm1_motor    大臂电机句柄
 * @param arm2_motor    小臂电机句柄
 * @param end_pos       输出：末端坐标数组 [x1, y1, z1]
 * @return 执行状态（同arm_forward_kinematics）
 */
uint8_t arm_get_end_position(dm_handle_t *pan_motor,
                             unitree_motor_handle_t *arm1_motor,
                             unitree_motor_handle_t *arm2_motor,
                             float end_pos[3]) {
    if (end_pos == NULL) return 1;
    return arm_forward_kinematics(pan_motor, arm1_motor, arm2_motor,
                                  &end_pos[0], &end_pos[1], &end_pos[2]);
}