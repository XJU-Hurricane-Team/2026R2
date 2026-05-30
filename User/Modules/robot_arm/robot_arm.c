/**
 * @file robot_arm.c
 * @author xinglu
 * @brief 机械臂驱动模块 (核心算法与动力学层)
 * @version 3.1
 * @date 2026-05-17
 */

#include "robot_arm.h"
#include <math.h>
#include <string.h>

/* ---------------- 静态工具函数 ---------------- */

static float arm_clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float arm_wrap_pi(float angle) {
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

static uint8_t arm_is_place_like(arm_status_t status) {
    return (status == ARM_STATE_PLACE) || (status == ARM_STATE_WAIT_TAKEOUT) ||
           (status == ARM_STATE_TAKEOUT_1) || (status == ARM_STATE_TAKEOUT_2);
}

uint8_t arm_is_ready_state(RobotArm *arm) {
    return (arm->status == ARM_STATE_READY_1) ||
           (arm->status == ARM_STATE_READY_2) || (arm->status == ARM_STATE_READY_3);
}

uint8_t arm_is_takeout_state(RobotArm *arm) {
    return (arm->status == ARM_STATE_TAKEOUT_1) ||
           (arm->status == ARM_STATE_TAKEOUT_2);
}

static int8_t arm_get_ik_quadrant(float y, float z, float pitch) {
    float x_w = y + DEFAULT_X - ARM_3 * cosf(pitch) -
                (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch);
    (void)z;
    return (x_w >= 0.0f) ? 1 : -1;
}

/* ---------------- 目标解算与预处理 ---------------- */

static void arm_get_unwrapped_target(RobotArm *arm, float y, float z,
                                     float pitch, float out_joints[3]) {
    arm_pos_angle(y, z, pitch, out_joints);

    /* 大臂 unwrap */
    float diff0 = out_joints[0] - arm->damiao_1.position;
    while (diff0 > PI) {
        diff0 -= 2.0f * PI;
        out_joints[0] -= 2.0f * PI;
    }
    while (diff0 < -PI) {
        diff0 += 2.0f * PI;
        out_joints[0] += 2.0f * PI;
    }
    if (out_joints[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) {
        out_joints[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;
    }

    /* 小臂 unwrap (damiao_3): 大于 2PI 或小于 -2PI 才需要校准 */
    float diff1 = out_joints[1] - arm->damiao_3.position;
    while (diff1 > 2.0f * PI) {
        diff1 -= 2.0f * PI;
        out_joints[1] -= 2.0f * PI;
    }
    while (diff1 < -2.0f * PI) {
        diff1 += 2.0f * PI;
        out_joints[1] += 2.0f * PI;
    }

    /* 吸盘 unwrap (damiao_4): 大于 2PI 或小于 -2PI 才需要校准 */
    float target2 = out_joints[2];
    float diff2 = target2 - arm->damiao_4.position;
    while (diff2 > 2.0f * PI) {
        if (target2 - 2.0f * PI < -0.1f) {
            break;
        }
        diff2 -= 2.0f * PI;
        target2 -= 2.0f * PI;
    }
    while (diff2 < -2.0f * PI) {
        if (target2 + 2.0f * PI > 3.8f) {
            break;
        }
        diff2 += 2.0f * PI;
        target2 += 2.0f * PI;
    }
    out_joints[2] = arm_clampf(target2, -0.1f, 3.8f);
}

/* ---------------- 状态机转换检测 ---------------- */

static void arm_detect_state_change(RobotArm *arm) {
    if (arm->status == arm->last_status) {
        return;
    }

    arm_status_t prev = arm->last_status;
    arm_status_t curr = arm->status;

    uint32_t now = HAL_GetTick();

    arm->latch_type = ARM_LATCH_NONE;
    arm->flags.suction_wait_locked_inited = 0;
    arm->flags.small_arm_wait_locked_inited = 0;
    arm->flags.big_arm_wait_locked_inited = 0;

    if ((prev == ARM_STATE_CATCH) && arm_is_place_like(curr)) {
        arm->latch_type = ARM_LATCH_SUCTION_WAIT;
        arm->suction_wait_start_tick = now;
    } else if (arm_is_place_like(prev) && arm_is_takeout_state(arm)) {
        /* 如果已经启动了三步序列，不要覆盖其锁存和过冲设置 */
        if (arm->motion_state == ARM_MOTION_STATE_TAKEOUT_SEQ_BIG) {
            /* 三步序列已启动，保持 SEQ_BIG_ARM_ONLY 锁存，不触发额外过冲 */
        } else {
            arm->latch_type = ARM_LATCH_TAKEOUT_WAIT;
            arm->suction_wait_start_tick = now;
            arm->big_arm_overshoot_rad =
                ARM_PLACE_TAKEOUT_BIG_ARM_OVERSHOOT_RAD;
            arm->big_arm_overshoot_armed = 1;
            /* takeout_target_suction_angle 已在 robot_arm_start_takeout_sequence 中设置 */
        }
    } else if (arm_is_place_like(prev) && (curr == ARM_STATE_CATCH)) {
        arm->latch_type = ARM_LATCH_SMALL_ARM_WAIT;
        arm->suction_wait_start_tick = now;
    }

    arm->last_status = curr;
}

/* ---------------- 状态机更新 ---------------- */

static void arm_update_motion_state(RobotArm *arm, float joint_target[3]) {
    float err0 = fabsf(arm_wrap_pi(arm->damiao_1.position - joint_target[0]));
    float err1 = fabsf(arm->damiao_3.position - joint_target[1]);

    switch (arm->motion_state) {
        case ARM_MOTION_STATE_IDLE:
        case ARM_MOTION_STATE_DIRECT_MOVE:
            break;

        case ARM_MOTION_STATE_MULTI_TRANS:
            if (err0 < 0.20f && err1 < 0.20f) {
                arm->multi_trans_index++;
                if (arm->multi_trans_index < arm->multi_trans_count) {
                    arm->arm_target_y = arm->trans_y[arm->multi_trans_index];
                    arm->arm_target_z = arm->trans_z[arm->multi_trans_index];
                    arm->arm_target_pitch =
                        arm->trans_pitch[arm->multi_trans_index];
                } else {
                    /* 三点过渡完成，根据层数决定是否进入过冲序列 */
                    float overshoot_by_layer =
                        ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD -
                        arm->place_layer * 0.30f;
                    if (overshoot_by_layer < 0.0f) {
                        overshoot_by_layer = 0.0f;
                    }
                    arm->big_arm_overshoot_rad = overshoot_by_layer;

                    if (overshoot_by_layer < 0.0f) {
                        /* 无过冲，直接进入小臂吸盘阶段 */
                        arm->motion_state =
                            ARM_MOTION_STATE_PLACE_SMALL_SUCTION;
                        arm->latch_type = ARM_LATCH_PLACE_SMALL_SUCTION;
                        arm->suction_wait_start_tick = HAL_GetTick();
                        arm->flags.small_arm_wait_locked_inited = 0;
                        arm->flags.suction_wait_locked_inited = 0;
                    } else {
                        /* 有过冲，进入过冲阶段 */
                        arm->motion_state = ARM_MOTION_STATE_PLACE_OVERSHOOT;
                        arm->flags.place_exit_safety_active = 0;
                        arm->big_arm_overshoot_armed = 1;
                        /* 设置锁存：大臂过冲期间锁死小臂和吸盘 */
                        arm->latch_type = ARM_LATCH_PLACE_OVERSHOOT_WAIT;
                        arm->suction_wait_start_tick = HAL_GetTick();
                        arm->flags.small_arm_wait_locked_inited = 0;
                        arm->flags.suction_wait_locked_inited = 0;
                    }
                    arm->arm_target_y = arm->final_target_y;
                    arm->arm_target_z = arm->final_target_z;
                    arm->arm_target_pitch = arm->final_target_pitch;
                    /* 重新计算 joint_target 为最终目标的关节角度 */
                    arm_get_unwrapped_target(
                        arm, arm->final_target_y, arm->final_target_z,
                        arm->final_target_pitch, joint_target);
                }
            }
            break;

        case ARM_MOTION_STATE_PLACE_OVERSHOOT:
            /* 过冲阶段：等待过冲完成，然后进入小臂吸盘同步阶段 */
            if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_RECOVER ||
                arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_NONE) {
                /* 过冲恢复完成，切换到小臂吸盘运动阶段 */
                arm->motion_state = ARM_MOTION_STATE_PLACE_SMALL_SUCTION;
                arm->latch_type = ARM_LATCH_PLACE_SMALL_SUCTION;
                arm->suction_wait_start_tick = HAL_GetTick();
                arm->flags.small_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        case ARM_MOTION_STATE_PLACE_SMALL_SUCTION:
            /* 小臂吸盘阶段：只判断小臂到位，吸盘不计入 */
            if (err1 < ARM_SMALL_ARM_READY_ERR_RAD) {
                /* 小臂到位了，进入大臂回位阶段 */
                arm->motion_state = ARM_MOTION_STATE_PLACE_BIG_RECOVER;
                arm->latch_type = ARM_LATCH_PLACE_BIG_RECOVER;
                arm->suction_wait_start_tick = HAL_GetTick();
                arm->flags.big_arm_wait_locked_inited = 0;
            }
            break;

        case ARM_MOTION_STATE_PLACE_BIG_RECOVER:
            /* 大臂回位阶段：等待大臂到位 */
            if (err0 < ARM_BIG_ARM_READY_ERR_RAD) {
                /* 所有到位，完成整个序列 */
                arm->motion_state = ARM_MOTION_STATE_DIRECT_MOVE;
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.small_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
                arm->flags.big_arm_wait_locked_inited = 0;
            }
            break;

        case ARM_MOTION_STATE_SINGLE_TRANS:
            if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_NONE &&
                err0 < 0.12f && err1 < 0.12f) {
                arm->flags.transition_done = 1;
                arm->motion_state = ARM_MOTION_STATE_DIRECT_MOVE;
                arm->arm_target_y = arm->final_target_y;
                arm->arm_target_z = arm->final_target_z;
                arm->arm_target_pitch = arm->final_target_pitch;
                arm->flags.place_exit_safety_active = 0;
            }
            break;

        case ARM_MOTION_STATE_TAKEOUT_SEQ_BIG:
            if (err0 <= ARM_TAKEOUT_SEQ_ERR_TOLERANCE_RAD) {
                arm->motion_state = ARM_MOTION_STATE_TAKEOUT_SEQ_SMALL;
                /* 进入第二步：仅小臂运动，大臂和吸盘锁定 */
                arm->latch_type = ARM_LATCH_SEQ_SMALL_ARM_ONLY;
                arm->suction_wait_start_tick = HAL_GetTick();
                arm->flags.big_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        case ARM_MOTION_STATE_TAKEOUT_SEQ_SMALL:
            if (err1 <= ARM_TAKEOUT_SEQ_ERR_TOLERANCE_RAD) {
                arm->motion_state = ARM_MOTION_STATE_TAKEOUT_SEQ_FINAL;
                /* 进入第三步：清除所有锁存，三关节共同运动 */
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.big_arm_wait_locked_inited = 0;
                arm->flags.small_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        case ARM_MOTION_STATE_TAKEOUT_SEQ_FINAL:
            break;

        default:
            break;
    }
}

/* ---------------- 过冲处理 ---------------- */

static void arm_update_overshoot(RobotArm *arm, float joint_target[3]) {
    if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_NONE &&
        !arm->big_arm_overshoot_armed) {
        return;
    }

    if (arm->big_arm_overshoot_armed) {
        float overshoot_sign = 1.0f;
        arm->big_arm_final_joint = joint_target[0];

        arm->big_arm_overshoot_joint =
            joint_target[0] + overshoot_sign * arm->big_arm_overshoot_rad;
        if (arm->big_arm_overshoot_joint < ARM_BIG_ARM_MIN_ANGLE_RAD) {
            arm->big_arm_overshoot_joint = ARM_BIG_ARM_MIN_ANGLE_RAD;
        }

        arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_OVERSHOOT;
        arm->big_arm_overshoot_armed = 0;
    }

    if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_OVERSHOOT) {
        joint_target[0] = arm->big_arm_overshoot_joint;

        float pos_err =
            fabsf(arm->damiao_1.position - arm->big_arm_overshoot_joint);

        if (pos_err < 0.50f) {
            arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_RECOVER;
        }

        uint32_t elapsed = HAL_GetTick() - arm->suction_wait_start_tick;

        if (arm->latch_type == ARM_LATCH_TAKEOUT_WAIT) {
            if (pos_err < 0.05f) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_RECOVER;
            }
            return;
        }

        float actual_suction = joint_target[2];
        if (arm_is_place_like(arm->status)) {
            actual_suction -= ARM_PLACE_SUCTION_OFFSET_RAD;
        }
        actual_suction = arm_clampf(actual_suction, -0.1f, 3.8f);

        float err_small = fabsf(arm->damiao_3.position - joint_target[1]);
        uint8_t small_ready = (err_small < 0.40f);

        if (small_ready) {
            arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_RECOVER;
        }
    } else if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_RECOVER) {
        joint_target[0] = arm->big_arm_final_joint;
        if (fabsf(arm->damiao_1.position - arm->big_arm_final_joint) < 0.05f) {
            arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_NONE;
            arm->flip_transition_dir = 0;
        }
    }
}

/* ---------------- 锁存处理 ---------------- */

/**
 * @brief 应用锁存逻辑，控制各关节的运动状态
 * @param arm 机械臂结构体指针
 * @param joint_des 期望的三关节角度 [大臂, 小臂, 吸盘]
 * @param out_joint1_cmd 输出的小臂目标位置
 * @param out_joint1_speed 输出的小臂目标速度
 * @param out_joint2_cmd 输出的吸盘目标位置
 * @param out_joint2_speed 输出的吸盘目标速度
 * @param small_speed_base 小臂基础速度
 * @param small_speed_gain 小臂速度增益
 * @param small_speed_max 小臂最大速度限制
 * @note 根据当前的锁存类型(latch_type)决定各关节的运动策略
 */
static void arm_apply_latch(RobotArm *arm, float joint_des[3],
                            float *out_joint1_cmd, float *out_joint1_speed,
                            float *out_joint2_cmd, float *out_joint2_speed,
                            float small_speed_base, float small_speed_gain,
                            float small_speed_max) {
    uint32_t elapsed = HAL_GetTick() - arm->suction_wait_start_tick;
    uint8_t timeout = (elapsed >= ARM_SUCTION_WAIT_TIMEOUT_MS);

    uint8_t small_arm_ready = (fabsf(arm->damiao_3.position - joint_des[1]) <=
                               ARM_SMALL_ARM_READY_ERR_RAD);
    uint8_t suction_ready = (fabsf(arm->damiao_4.position - joint_des[2]) <=
                             ARM_SMALL_ARM_READY_ERR_RAD);

    /* 计算各关节误差，用于新锁存类型的到位判断 */
    float err0 = fabsf(arm_wrap_pi(arm->damiao_1.position - joint_des[0]));
    float err1 = fabsf(arm->damiao_3.position - joint_des[1]);

    switch (arm->latch_type) {
        case ARM_LATCH_SUCTION_WAIT:
            if (small_arm_ready) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        case ARM_LATCH_SMALL_ARM_WAIT:
            if (suction_ready || timeout) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.small_arm_wait_locked_inited = 0;
            }
            break;

        case ARM_LATCH_TAKEOUT_WAIT:
            if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_RECOVER ||
                arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_NONE ||
                timeout) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.small_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        /* 放置态三点过渡后过冲阶段：大臂过冲，小臂和吸盘锁死 */
        case ARM_LATCH_PLACE_OVERSHOOT_WAIT: {
            uint8_t overshoot_done =
                (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_RECOVER ||
                 arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_NONE);
            /* 固定锁死，直到过冲恢复完成 */
            if (overshoot_done) {
                /* 过冲完成，但保持锁存，等待 arm_update_motion_state 切换状态 */
            }
        } break;

        /* 放置态小臂吸盘运动阶段：小臂和吸盘运动，大臂锁死 */
        case ARM_LATCH_PLACE_SMALL_SUCTION: {
            /* 只判断小臂到位，吸盘不计入 */
            if (err1 < ARM_SMALL_ARM_READY_ERR_RAD || timeout) {
                /* 到位，但保持锁存，等待 arm_update_motion_state 切换状态 */
            }
        } break;

        /* 放置态大臂回位阶段：大臂回位，小臂和吸盘锁死 */
        case ARM_LATCH_PLACE_BIG_RECOVER: {
            uint8_t big_ready = (err0 < ARM_BIG_ARM_READY_ERR_RAD);
            if (big_ready || timeout) {
                /* 到位，但保持锁存，等待 arm_update_motion_state 切换状态 */
            }
        } break;

        /* 取出序列专用锁存：仅大臂运动，小臂和吸盘锁定 */
        case ARM_LATCH_SEQ_BIG_ARM_ONLY:
            if (fabsf(arm_wrap_pi(arm->damiao_1.position -
                                  arm->takeout_seq_big_arm_target)) <=
                ARM_TAKEOUT_SEQ_ERR_TOLERANCE_RAD) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.small_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        /* 取出序列专用锁存：仅小臂运动，大臂和吸盘锁定 */
        case ARM_LATCH_SEQ_SMALL_ARM_ONLY:
            if (fabsf(arm->damiao_3.position -
                      arm->takeout_seq_small_arm_target) <=
                ARM_TAKEOUT_SEQ_ERR_TOLERANCE_RAD) {
                arm->latch_type = ARM_LATCH_NONE;
                arm->flags.big_arm_wait_locked_inited = 0;
                arm->flags.suction_wait_locked_inited = 0;
            }
            break;

        default:
            break;
    }

    /* 小臂锁存判断：原有的 SMALL_ARM_WAIT 或 TAKEOUT_WAIT，以及新增的 SEQ_SMALL_ARM_ONLY */
    if (arm->latch_type == ARM_LATCH_SMALL_ARM_WAIT ||
        arm->latch_type == ARM_LATCH_TAKEOUT_WAIT) {
        if (!arm->flags.small_arm_wait_locked_inited) {
            arm->small_arm_wait_locked_pos = arm->damiao_3.position;
            arm->flags.small_arm_wait_locked_inited = 1;
        }
        *out_joint1_cmd = arm->small_arm_wait_locked_pos;
        *out_joint1_speed = ARM_SMALL_WAIT_HOLD_SPEED;
    } else if (arm->latch_type == ARM_LATCH_SEQ_SMALL_ARM_ONLY) {
        /* SEQ_SMALL 阶段：小臂动，所以这里不锁存（走 else 分支） */
        *out_joint1_cmd = joint_des[1];
        *out_joint1_speed = arm_clampf(
            small_speed_base +
                small_speed_gain * fabsf(joint_des[1] - arm->damiao_3.position),
            0.04f, small_speed_max);
    } else if (arm->latch_type == ARM_LATCH_SEQ_BIG_ARM_ONLY) {
        /* SEQ_BIG 阶段：小臂锁定 */
        if (!arm->flags.small_arm_wait_locked_inited) {
            arm->small_arm_wait_locked_pos = arm->damiao_3.position;
            arm->flags.small_arm_wait_locked_inited = 1;
        }
        *out_joint1_cmd = arm->small_arm_wait_locked_pos;
        *out_joint1_speed = ARM_SMALL_WAIT_HOLD_SPEED;
    } else if (arm->latch_type == ARM_LATCH_PLACE_BIG_RECOVER) {
        /* 放置态大臂回位阶段：小臂锁定 */
        if (!arm->flags.small_arm_wait_locked_inited) {
            arm->small_arm_wait_locked_pos = arm->damiao_3.position;
            arm->flags.small_arm_wait_locked_inited = 1;
        }
        *out_joint1_cmd = arm->small_arm_wait_locked_pos;
        *out_joint1_speed = ARM_SMALL_WAIT_HOLD_SPEED;
    } else if (arm->latch_type == ARM_LATCH_PLACE_OVERSHOOT_WAIT ||
               arm->latch_type == ARM_LATCH_PLACE_SMALL_SUCTION) {
        /* 放置态小臂吸盘阶段：小臂运动 */
        *out_joint1_cmd = joint_des[1];
        *out_joint1_speed = arm_clampf(
            small_speed_base +
                small_speed_gain * fabsf(joint_des[1] - arm->damiao_3.position),
            0.04f, small_speed_max);
    } else {
        *out_joint1_cmd = joint_des[1];
        *out_joint1_speed = arm_clampf(
            small_speed_base +
                small_speed_gain * fabsf(joint_des[1] - arm->damiao_3.position),
            0.04f, small_speed_max);
    }

    /* 吸盘锁存判断 */
    if (arm->latch_type == ARM_LATCH_SUCTION_WAIT ||
        arm->latch_type == ARM_LATCH_TAKEOUT_WAIT ||
        arm->latch_type == ARM_LATCH_SEQ_BIG_ARM_ONLY ||
        arm->latch_type == ARM_LATCH_SEQ_SMALL_ARM_ONLY ||
        arm->latch_type == ARM_LATCH_PLACE_BIG_RECOVER) {
        if (!arm->flags.suction_wait_locked_inited) {
            /* TAKEOUT_WAIT 使用待取出态解算的目标角度，其他使用电机反馈角度 */
            if (arm->latch_type == ARM_LATCH_TAKEOUT_WAIT) {
                arm->suction_wait_locked_pos =
                    arm->takeout_target_suction_angle;
            } else {
                arm->suction_wait_locked_pos = arm->damiao_4.position;
            }
            arm->flags.suction_wait_locked_inited = 1;
        }
        *out_joint2_cmd = arm->suction_wait_locked_pos;
        *out_joint2_speed = ARM_SUCTION_WAIT_HOLD_SPEED;
    } else if (arm->latch_type == ARM_LATCH_PLACE_SMALL_SUCTION) {
        /* 放置态小臂吸盘阶段：吸盘运动 */
        *out_joint2_cmd = joint_des[2];
        *out_joint2_speed = ARM_SUCTION_SPEED_MAX;
    } else {
        *out_joint2_cmd = joint_des[2];
        *out_joint2_speed = ARM_SUCTION_SPEED_MAX;
    }

    /* 大臂锁存判断（仅用于 SEQ_SMALL_ARM_ONLY 阶段） */
    if (arm->latch_type == ARM_LATCH_SEQ_SMALL_ARM_ONLY) {
        if (!arm->flags.big_arm_wait_locked_inited) {
            arm->big_arm_wait_locked_pos = arm->damiao_1.position;
            arm->flags.big_arm_wait_locked_inited = 1;
        }
        /* 注意：大臂锁存通过修改 joint_des[0] 实现，在 arm_apply_ctrl 中处理 */
    }
}

/* ---------------- 驱动层主接口 ---------------- */

/**
 * @brief 系统初始化
 * @param arm 机械臂结构体指针
 */
void robot_arm_system_init(RobotArm *arm) {
    memset(arm, 0, sizeof(RobotArm));

    dm_motor_init(&arm->damiao_1, 0x11, 0x01, DM_MODE_PVT, DM_J8006, 3.14f,
                  10.0f, 10.0f, can3_selected);
    dm_motor_init(&arm->damiao_2, 0x12, 0x02, DM_MODE_PVT, DM_J8006, 3.14f,
                  10.0f, 10.0f, can3_selected);
    dm_motor_init(&arm->damiao_3, 0x13, 0x03, DM_MODE_PVT, DM_J4340, 6.28f,
                  10.0f, 10.0f, can3_selected);
    dm_motor_init(&arm->damiao_4, 0x14, 0x04, DM_MODE_PVT, DM_J4310, 6.28f,
                  10.0f, 10.0f, can3_selected);

    dm_motor_enable(&arm->damiao_1);
    dm_motor_enable(&arm->damiao_2);
    dm_motor_enable(&arm->damiao_3);
    dm_motor_enable(&arm->damiao_4);

    arm->target_mode = ARM_TARGET_CARTESIAN;
    arm->status = ARM_STATE_INIT;
    arm->last_status = ARM_STATE_INIT;
    arm->motion_state = ARM_MOTION_STATE_IDLE;
    arm->ctrl_dt = ARM_CTRL_DT_DEFAULT;

    arm->joint_cmd_prev[0] = DEFAULT_ANGLE_1;
    arm->joint_cmd_prev[1] = DEFAULT_ANGLE_2;
    arm->joint_cmd_prev[2] = DEFAULT_ANGLE_3;
    arm->big_arm_cmd_filtered = DEFAULT_ANGLE_1;
}

/**
 * @brief 设定底层计算控制周期
 * @param arm 机械臂结构体指针
 * @param dt_s 周期时长 (s)
 */
void robot_arm_set_ctrl_dt(RobotArm *arm, float dt_s) {
    arm->ctrl_dt = arm_clampf(dt_s, 0.001f, 0.2f);
}

/**
 * @brief 设置笛卡尔空间目标
 * @param arm 机械臂结构体指针
 * @param y Y轴坐标
 * @param z Z轴坐标
 * @param pitch 姿态角
 */
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch) {
    arm->target_mode = ARM_TARGET_CARTESIAN;
    arm->flags.place_exit_safety_active = 0;
    arm->flags.place_entry_sync_active = 0;

    int8_t target_quadrant = arm_get_ik_quadrant(y, z, pitch);
    arm->flip_transition_dir = 0;
    arm->big_arm_overshoot_armed = 0;
    arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_NONE;

    if (arm->last_target_quadrant != 0) {
        if ((arm->last_target_quadrant < 0) && (target_quadrant > 0)) {
            arm->flip_transition_dir = 1;
        } else if ((arm->last_target_quadrant > 0) && (target_quadrant < 0)) {
            arm->flip_transition_dir = -1;
        }
    }

    uint8_t delay_overshoot = 0;

    /* 进入 PLACE/WAIT_TAKEOUT 时重置象限跟踪，避免与多点过渡冲突 */
    if (arm_is_place_like(arm->status)) {
        arm->last_target_quadrant = 0;
        arm->flip_transition_dir = 0;
    }

    if (arm_is_place_like(arm->last_status) &&
        !arm_is_place_like(arm->status) && !arm_is_takeout_state(arm) &&
        arm->status != ARM_STATE_INIT) {
        arm->motion_state = ARM_MOTION_STATE_SINGLE_TRANS;
        arm->flags.has_transition = 1;
        arm->flags.transition_done = 0;

        arm->final_target_y = y;
        arm->final_target_z = z;
        arm->final_target_pitch = pitch;
        arm->arm_target_y = y + 100.0f;
        arm->arm_target_z = z + 100.0f;
        arm->arm_target_pitch = pitch;

        arm->flags.place_exit_safety_active = 1;
    } else if (arm->status == ARM_STATE_CATCH) {
        /* CATCH: 添加过渡点 (y-200, z+200) */
        arm->motion_state = ARM_MOTION_STATE_SINGLE_TRANS;
        arm->flags.has_transition = 1;
        arm->flags.transition_done = 0;

        arm->final_target_y = y;
        arm->final_target_z = z;
        arm->final_target_pitch = pitch;
        arm->arm_target_y = y - 20.0f;
        arm->arm_target_z = z + 20.0f;
        arm->arm_target_pitch = pitch;
    } else if (arm->last_status != arm->status &&
               arm_is_place_like(arm->status)) {
        arm->motion_state = ARM_MOTION_STATE_MULTI_TRANS;
        arm->multi_trans_index = 0;
        arm->multi_trans_count = 3;

        arm->trans_y[0] = 468.158f;
        arm->trans_z[0] = 183.510f;
        arm->trans_pitch[0] = 0.0f;
        arm->trans_y[1] = 422.826f;
        arm->trans_z[1] = 775.271f;
        arm->trans_pitch[1] = PI / 2.0f;
        /* 根据层数选择第三个过渡点: 0=底层/顶层(一三层), 1=中层(二层) */
        if (arm->place_layer == 1) {
            /* 第二层: 使用更靠后的过渡点 */
            arm->trans_y[2] = 100.000f;
            arm->trans_z[2] = 835.030f;
            arm->trans_pitch[2] = PI / 2.0f;
        } else {
            /* 第一层/第三层: 使用更靠前的过渡点 */
            arm->trans_y[2] = 280.000f;
            arm->trans_z[2] = 785.030f;
            arm->trans_pitch[2] = PI / 2.0f;
        }

        arm->final_target_y = y;
        arm->final_target_z = z;
        arm->final_target_pitch = pitch;
        arm->arm_target_y = arm->trans_y[0];
        arm->arm_target_z = arm->trans_z[0];
        arm->arm_target_pitch = arm->trans_pitch[0];

        arm->flags.has_transition = 0;
        delay_overshoot = 1;
        arm->flags.place_entry_sync_active = 1;
    } else if (arm->flip_transition_dir == 1) {
        arm->motion_state = ARM_MOTION_STATE_SINGLE_TRANS;
        arm->flags.has_transition = 1;
        arm->flags.transition_done = 0;
        arm->final_target_y = y;
        arm->final_target_z = z;
        arm->final_target_pitch = pitch;

        float offset_y = ARM_TRANS_Y_OFFSET;
        arm->arm_target_y = (y > offset_y) ? (y - offset_y) : 0.0f;
        arm->arm_target_z = z + ARM_TRANS_Z_OFFSET;
        arm->arm_target_pitch = pitch;
    } else {
        arm->motion_state = ARM_MOTION_STATE_DIRECT_MOVE;
        arm->flags.has_transition = 0;
        arm->flags.transition_done = 1;
        arm->final_target_y = y;
        arm->final_target_z = z;
        arm->final_target_pitch = pitch;
        arm->arm_target_y = y;
        arm->arm_target_z = z;
        arm->arm_target_pitch = pitch;
    }

    if (!delay_overshoot) {
        if (arm_is_place_like(arm->status)) {
            arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
            arm->big_arm_overshoot_armed = 1;
        } else if (arm_is_takeout_state(arm)) {
            arm->big_arm_overshoot_rad =
                ARM_UPPER_TAKEOUT_BIG_ARM_OVERSHOOT_RAD;
            arm->big_arm_overshoot_armed = 1;
        } else if (arm->flip_transition_dir == 1 && z < 800.0f) {
            arm->big_arm_overshoot_rad = ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD;
            arm->big_arm_overshoot_armed = 1;
        }
    }
    arm->last_target_quadrant = target_quadrant;
}

/**
 * @brief 设置关节空间目标
 * @param arm 机械臂结构体指针
 * @param joint1 大臂角度
 * @param joint2 小臂角度
 * @param joint3 吸盘角度
 */
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2,
                                float joint3) {
    arm->arm_joint_target[0] = (joint1 < ARM_BIG_ARM_MIN_ANGLE_RAD)
                                   ? ARM_BIG_ARM_MIN_ANGLE_RAD
                                   : joint1;
    arm->arm_joint_target[1] = joint2;
    arm->arm_joint_target[2] = joint3;
    arm->target_mode = ARM_TARGET_JOINT;
    arm->big_arm_overshoot_armed = 0;
    arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_NONE;
}

/**
 * @brief 设置强制大臂过冲量
 * @param arm 机械臂结构体指针
 * @param overshoot_rad 过冲角度 (rad)
 */
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad) {
    if (overshoot_rad <= 0.0f) {
        arm->big_arm_overshoot_rad = 0.0f;
        arm->big_arm_overshoot_armed = 0;
        arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_NONE;
        return;
    }
    arm->big_arm_overshoot_rad = overshoot_rad;
    arm->big_arm_overshoot_armed = 1;
}

/**
 * @brief 启动取出动作序列
 * @param arm 机械臂结构体指针
 * @param y 目标Y坐标
 * @param z 目标Z坐标
 * @param pitch 目标姿态角
 * @param wait_takeout_suction_angle 待取出态的吸盘目标角度（用于锁定）
 * @return 1: 序列已启动, 0: 无需序列 (直接运动)
 */
uint8_t robot_arm_start_takeout_sequence(RobotArm *arm, float y, float z,
                                         float pitch,
                                         float wait_takeout_suction_angle) {
    float final_joint[3];
    arm_pos_angle(y, z, pitch, final_joint);

    float dir0 = 1.0f;
    float dir1 = -1.0f;

    arm->final_target_y = y;
    arm->final_target_z = z;
    arm->final_target_pitch = pitch;

    arm->takeout_seq_big_arm_target =
        arm->damiao_1.position + dir0 * ARM_TAKEOUT_SEQ_BIG_ARM_STEP_RAD;
    if (arm->takeout_seq_big_arm_target < ARM_BIG_ARM_MIN_ANGLE_RAD) {
        arm->takeout_seq_big_arm_target = ARM_BIG_ARM_MIN_ANGLE_RAD;
    }
    arm->takeout_seq_small_arm_target =
        arm->damiao_3.position + dir1 * ARM_TAKEOUT_SEQ_SMALL_ARM_STEP_RAD;

    arm->motion_state = ARM_MOTION_STATE_TAKEOUT_SEQ_BIG;
    arm->target_mode = ARM_TARGET_JOINT;
    arm->big_arm_overshoot_armed = 0;
    arm->big_arm_overshoot_phase = ARM_OVERSHOOT_PHASE_NONE;

    /* 设置第一步：仅大臂运动，小臂和吸盘锁定 */
    arm->latch_type = ARM_LATCH_SEQ_BIG_ARM_ONLY;
    arm->suction_wait_start_tick = HAL_GetTick();
    arm->flags.small_arm_wait_locked_inited = 0;
    arm->flags.suction_wait_locked_inited = 0;

    /* 保存待取出态的吸盘目标角度（用于后续TAKEOUT状态切换时锁定） */
    arm->takeout_target_suction_angle = wait_takeout_suction_angle;

    arm->arm_joint_target[0] = arm->takeout_seq_big_arm_target;
    arm->arm_joint_target[1] = arm->damiao_3.position;
    arm->arm_joint_target[2] = arm->damiao_4.position;

    return 1;
}

/**
 * @brief 检查是否处于取出序列执行中
 * @param arm 机械臂结构体指针
 * @return 1: 序列执行中, 0: 无序列或已完成
 */
uint8_t robot_arm_is_takeout_sequence_active(RobotArm *arm) {
    return (arm->motion_state >= ARM_MOTION_STATE_TAKEOUT_SEQ_BIG) &&
           (arm->motion_state <= ARM_MOTION_STATE_TAKEOUT_SEQ_FINAL);
}

/**
 * @brief 周期控制更新任务
 * @param arm 机械臂结构体指针
 */
void robot_arm_update(RobotArm *arm) {
    float joint_target[3];
    float joint_cmd[3];

    if (!arm->flags.joint_cmd_inited) {
        arm->flags.joint_cmd_inited = 1;
        arm->joint_cmd_prev[0] = arm->damiao_1.position;
        arm->joint_cmd_prev[1] = arm->damiao_3.position;
        arm->joint_cmd_prev[2] = arm->damiao_4.position;
    }

    arm_detect_state_change(arm);

    if (arm->target_mode == ARM_TARGET_JOINT) {
        joint_target[0] = arm->arm_joint_target[0];
        joint_target[1] = arm->arm_joint_target[1];
        joint_target[2] = arm_clampf(arm->arm_joint_target[2], -0.1f, 3.8f);
    } else {
        arm_get_unwrapped_target(arm, arm->arm_target_y, arm->arm_target_z,
                                 arm->arm_target_pitch, joint_target);
    }

    arm_update_motion_state(arm, joint_target);

    if (arm->motion_state == ARM_MOTION_STATE_TAKEOUT_SEQ_SMALL) {
        arm->arm_joint_target[0] = arm->takeout_seq_big_arm_target;
        arm->arm_joint_target[1] = arm->takeout_seq_small_arm_target;
        joint_target[0] = arm->takeout_seq_big_arm_target;
        joint_target[1] = arm->takeout_seq_small_arm_target;
    } else if (arm->motion_state == ARM_MOTION_STATE_TAKEOUT_SEQ_FINAL) {
        arm->target_mode = ARM_TARGET_CARTESIAN;
        /* 同步更新目标坐标，避免后续周期使用旧值 */
        arm->arm_target_y = arm->final_target_y;
        arm->arm_target_z = arm->final_target_z;
        arm->arm_target_pitch = arm->final_target_pitch;
        arm_get_unwrapped_target(arm, arm->final_target_y, arm->final_target_z,
                                 arm->final_target_pitch, joint_target);
        arm->motion_state = ARM_MOTION_STATE_DIRECT_MOVE;
    }

    arm_update_overshoot(arm, joint_target);

    if (joint_target[0] < ARM_BIG_ARM_MIN_ANGLE_RAD) {
        joint_target[0] = ARM_BIG_ARM_MIN_ANGLE_RAD;
    }

    /* 取出序列 SEQ_SMALL 阶段：锁定大臂位置（吸盘已在 arm_apply_latch 中处理） */
    if (arm->latch_type == ARM_LATCH_SEQ_SMALL_ARM_ONLY) {
        if (!arm->flags.big_arm_wait_locked_inited) {
            arm->big_arm_wait_locked_pos = arm->damiao_1.position;
            arm->flags.big_arm_wait_locked_inited = 1;
        }
        joint_target[0] = arm->big_arm_wait_locked_pos;
    }

    /* 放置态小臂吸盘运动阶段：锁定大臂位置 */
    if (arm->latch_type == ARM_LATCH_PLACE_SMALL_SUCTION) {
        if (!arm->flags.big_arm_wait_locked_inited) {
            arm->big_arm_wait_locked_pos = arm->big_arm_overshoot_joint;
            arm->flags.big_arm_wait_locked_inited = 1;
        }
        joint_target[0] = arm->big_arm_wait_locked_pos;
    }

    joint_cmd[0] = joint_target[0];
    joint_cmd[1] = joint_target[1];
    joint_cmd[2] = joint_target[2];

    arm->joint_cmd_prev[0] = joint_cmd[0];
    arm->joint_cmd_prev[1] = joint_cmd[1];
    arm->joint_cmd_prev[2] = joint_cmd[2];

    arm_apply_ctrl(arm, joint_cmd);

    arm->arm_motion_active =
        (fabsf(arm->damiao_1.position - joint_cmd[0]) > 0.03f) ||
        (fabsf(arm->damiao_3.position - joint_cmd[1]) > 0.03f) ||
        (fabsf(arm->damiao_4.position - joint_cmd[2]) > 0.03f);

    robot_arm_fk(arm->damiao_1.position, arm->damiao_3.position,
                 arm->damiao_4.position, &arm->current_y, &arm->current_z,
                 &arm->current_pitch);
}

/**
 * @brief 底层电机力矩控制分发
 * @param arm 机械臂结构体指针
 * @param joint_des 期望的三关节角度
 */
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]) {
    float joint0_cmd = (joint_des[0] < ARM_BIG_ARM_MIN_ANGLE_RAD)
                           ? ARM_BIG_ARM_MIN_ANGLE_RAD
                           : joint_des[0];
    float joint0_err = joint0_cmd - arm->damiao_1.position;
    float err_abs = fabsf(joint0_err);
    float cmd_speed, max_step, delta;

    float joint1_final_cmd, joint1_final_speed;
    float joint2_final_cmd, joint2_final_speed;

    float small_speed_base, small_speed_gain, small_speed_max;
    float place_speed_min, suction_speed_max;

    if (!arm->flags.big_arm_filter_inited) {
        arm->flags.big_arm_filter_inited = 1;
        arm->big_arm_cmd_filtered = arm->damiao_1.position;
    }

    suction_speed_max = ARM_SUCTION_SPEED_MAX;

    if (arm->status == ARM_STATE_CATCH) {
        small_speed_base = ARM_SMALL_CATCH_SPEED_BASE;
        small_speed_gain = ARM_SMALL_CATCH_SPEED_GAIN;
        small_speed_max = ARM_SMALL_CATCH_SPEED_MAX;
    } else if (arm->status == ARM_STATE_OVERLOOK) {
        // 小臂速度
        small_speed_base = ARM_SMALL_SPEED_MAX;
        small_speed_gain = 2.0f;
        small_speed_max = ARM_OVERLOOK_SMALL_SPEED;
        // 吸盘速度
        suction_speed_max = ARM_OVERLOOK_SUCTION_SPEED;
    } else if (arm->flags.place_exit_safety_active) {
        small_speed_base = ARM_SMALL_PLACE_EXIT_SPEED_BASE;
        small_speed_gain = ARM_SMALL_PLACE_EXIT_SPEED_GAIN;
        small_speed_max = ARM_SMALL_PLACE_EXIT_SPEED_MAX;
    } else {
        small_speed_base = ARM_SMALL_SPEED_BASE;
        small_speed_gain = ARM_SMALL_SPEED_GAIN;
        small_speed_max = ARM_SMALL_SPEED_MAX;
    }

    max_step = ARM_J8006_CMD_RATE_LIMIT * arm->ctrl_dt;
    delta = arm_clampf(ARM_J8006_FILTER_ALPHA *
                           (joint0_cmd - arm->big_arm_cmd_filtered),
                       -max_step, max_step);
    arm->big_arm_cmd_filtered += delta;
    if (arm->big_arm_cmd_filtered < ARM_BIG_ARM_MIN_ANGLE_RAD) {
        arm->big_arm_cmd_filtered = ARM_BIG_ARM_MIN_ANGLE_RAD;
    }

    float est_ff = delta / arm->ctrl_dt;
    arm->big_arm_speed_filtered =
        0.6f * fabsf(est_ff) + 0.4f * arm->big_arm_speed_filtered;
    cmd_speed = ARM_J8006_CMD_SPEED_BASE + arm->big_arm_speed_filtered;

    if (err_abs > ARM_J8006_NEAR_ERR_RAD) {
        cmd_speed = fminf(cmd_speed, ARM_J8006_CMD_SPEED_MAX);
    } else if (err_abs > ARM_J8006_FINE_ERR_RAD) {
        float blend = (err_abs - ARM_J8006_FINE_ERR_RAD) /
                      (ARM_J8006_NEAR_ERR_RAD - ARM_J8006_FINE_ERR_RAD);
        cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX +
                                         blend * (ARM_J8006_NEAR_SPEED_MAX -
                                                  ARM_J8006_FINE_SPEED_MAX));
    } else {
        cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
    }

    if ((joint0_err * arm->damiao_1.speed) < 0.0f) {
        cmd_speed = fminf(cmd_speed, ARM_J8006_FINE_SPEED_MAX);
    }

    if (arm->big_arm_overshoot_phase == ARM_OVERSHOOT_PHASE_RECOVER) {
        float phase2_limit = (arm->flip_transition_dir != 0)
                                 ? ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX
                                 : ARM_J8006_NEAR_SPEED_MAX;
        cmd_speed = fminf(cmd_speed, phase2_limit);
    }

    if (arm->flags.place_exit_safety_active) {
        cmd_speed = fminf(cmd_speed, ARM_BIG_PLACE_EXIT_SPEED_MAX);
    }
    cmd_speed = arm_clampf(cmd_speed, 0.06f, ARM_J8006_CMD_SPEED_MAX);

    float joint2_cmd = joint_des[2];
    if (arm_is_place_like(arm->status)) {
        joint2_cmd -= ARM_PLACE_SUCTION_OFFSET_RAD;
    }

    arm_apply_latch(arm, joint_des, &joint1_final_cmd, &joint1_final_speed,
                    &joint2_final_cmd, &joint2_final_speed, small_speed_base,
                    small_speed_gain, small_speed_max);

    uint8_t place_like_mode =
        arm_is_place_like(arm->status) || arm_is_takeout_state(arm);

    if (arm->latch_type == ARM_LATCH_NONE) {
        if (place_like_mode) {
            joint2_final_cmd = joint2_cmd;
            float joint2_err = joint2_cmd - arm->damiao_4.position;
            joint2_final_speed =
                ARM_SUCTION_PLACE_SPEED_BASE +
                ARM_SUCTION_PLACE_SPEED_GAIN * fabsf(joint2_err);
            place_speed_min =
                (fabsf(joint2_err) < ARM_SUCTION_PLACE_FINE_ERR_RAD)
                    ? ARM_SUCTION_PLACE_FINE_SPEED_MIN
                    : ARM_SUCTION_PLACE_SPEED_MIN;

            if ((arm->flip_transition_dir != 0) ||
                (arm->big_arm_overshoot_phase != ARM_OVERSHOOT_PHASE_NONE)) {
                joint2_final_speed =
                    arm_clampf(joint2_final_speed, place_speed_min,
                               ARM_SUCTION_FLIP_SLOW_SPEED_MAX);
            } else {
                joint2_final_speed = arm_clampf(
                    joint2_final_speed, place_speed_min, suction_speed_max);
            }
            if ((joint2_err * arm->damiao_4.speed) < 0.0f) {
                joint2_final_speed = fminf(joint2_final_speed,
                                           ARM_SUCTION_PLACE_BRAKE_SPEED_MAX);
            }
        } else {
            joint2_final_cmd = joint2_cmd;
            joint2_final_speed = ARM_SUCTION_SPEED_MAX;
        }
    }

    joint2_final_cmd = arm_clampf(joint2_final_cmd, -0.1f, 3.8f);

    dm_pvt_ctrl(&arm->damiao_1, arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
    dm_pvt_ctrl(&arm->damiao_2, -arm->big_arm_cmd_filtered, cmd_speed, 0.65f);
    dm_pvt_ctrl(&arm->damiao_3, joint1_final_cmd, joint1_final_speed, 0.90f);
    dm_pvt_ctrl(&arm->damiao_4, joint2_final_cmd, joint2_final_speed, 0.90f);
}

/**
 * @brief 逆运动学解算
 * @param x1 目标X轴坐标
 * @param z1 目标Z轴坐标
 * @param pitch_angle 期望姿态角
 * @param angle 输出的三关节角度
 */
void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]) {
    // 1. 坐标平移
    float x = x1 + DEFAULT_X;
    float z = z1 + DEFAULT_Z;

    if (x < 0) {
        x += ARM_3;
        z += ARM_3;
    }

    // 2. 姿态解耦，求腕关节坐标
    float x_w = x - ARM_3 * cosf(pitch_angle) -
                (DEFAULT_ARM3_X - DEFAULT_ARM_1_2) * cosf(pitch_angle);
    float z_w = z - ARM_3 * sinf(pitch_angle);
    float m_2 = x_w * x_w + z_w * z_w;
    float m, b, b2, angle1_1, angle1_2, a, a2, angle2_inner;
    arm_sqrt_f32(m_2, &m);

    // 3. 求大臂角度
    b = arm_clampf((ARM_1 * ARM_1 + m_2 - ARM_2 * ARM_2) / (2 * ARM_1 * m),
                   -1.0f, 1.0f);
    arm_sqrt_f32(1 - b * b, &b2);
    arm_atan2_f32(b2, b, &angle1_1);
    arm_atan2_f32(z_w, x_w, &angle1_2);

    // 4. 求小臂角度
    a = arm_clampf((ARM_1 * ARM_1 + ARM_2 * ARM_2 - m_2) / (2 * ARM_1 * ARM_2),
                   -1.0f, 1.0f);
    arm_sqrt_f32(1 - a * a, &a2);
    arm_atan2_f32(a2, a, &angle2_inner);

    // 根据目标所在的象限分别计算输出角度
    if (x1 >= 0) {
        // 第一象限：保持原有的几何构型解
        angle[0] = DEFAULT_ANGLE_1 + PI / 2 - angle1_1 - angle1_2;
        angle[1] = angle2_inner - DEFAULT_ANGLE_2;
    } else {
        // 第二象限：切换到另一个解
        angle[0] = DEFAULT_ANGLE_1 + PI / 2 + angle1_1 - angle1_2;
        angle[1] = 2 * PI + (DEFAULT_ANGLE_2 - angle2_inner);
    }

    // 保证大臂不往后倒撞到已放置的箱子
    if (angle[0] < 0.0f) {
        angle[0] = 0;
    }

    // 5. 求吸盘角度
    angle[2] = -pitch_angle - angle[0] + angle[1] + DEFAULT_ANGLE_3;

    // 防止吸盘反转
    if (angle[2] < -0.1f) {
        angle[2] += 2 * PI;
    }
    if (angle[2] > 2 * PI) {
        angle[2] -= 2 * PI;
    }
}

/**
 * @brief 正运动学解算(由于吸盘关节电机回传数据有问题，暂时无法使用)
 * @param joint1 大臂角度
 * @param joint2 小臂角度
 * @param joint3 吸盘角度
 * @param y_out 输出的Y轴坐标
 * @param z_out 输出的Z轴坐标
 * @param pitch_out 输出的姿态角
 */
void robot_arm_fk(float joint1, float joint2, float joint3, float *y_out,
                  float *z_out, float *pitch_out) {
    float joint2_norm = joint2;
    while (joint2_norm < 0.0f) {
        joint2_norm += 2.0f * PI;
    }
    while (joint2_norm >= 2.0f * PI) {
        joint2_norm -= 2.0f * PI;
    }

    float pitch = -joint3 - joint1 + joint2_norm + DEFAULT_ANGLE_3;
    float theta_L1 = DEFAULT_ANGLE_1 + (PI / 2.0f) - joint1;
    float theta_L2, angle2_inner;

    if (joint2_norm < PI) {
        angle2_inner = joint2_norm + DEFAULT_ANGLE_2;
        theta_L2 = theta_L1 - (PI - angle2_inner);
    } else {
        angle2_inner = 2.0f * PI + DEFAULT_ANGLE_2 - joint2_norm;
        theta_L2 = theta_L1 + (PI - angle2_inner);
    }

    float x_w = ARM_1 * cosf(theta_L1) + ARM_2 * cosf(theta_L2);
    float z_w = ARM_1 * sinf(theta_L1) + ARM_2 * sinf(theta_L2);
    float eff_L3_x = ARM_3 + (DEFAULT_ARM3_X - DEFAULT_ARM_1_2);
    float x = x_w + eff_L3_x * cosf(pitch);
    float z = z_w + ARM_3 * sinf(pitch);

    *y_out = x - DEFAULT_X;
    *z_out = z - DEFAULT_Z;
    *pitch_out = pitch;
}
