/**
 * @file robot_arm.h
 * @author xinglu
 * @brief 机械臂底层驱动模块 (解算与运动学参数定义)
 * @version 3.1
 * @date 2026-05-17
 */

#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <cubemx.h>
#include "./Damiao-Motor/damiao.h"
#include "arm_math.h"

/* ================== 机械结构物理参数 ================== */
#define ARM_1 450.0f /**< 大臂长度 (mm) */
#define ARM_2 450.0f /**< 小臂长度 (mm) */
#define ARM_3 105.0f /**< 吸盘长度 (mm) */
#define DEFAULT_ANGLE_1                                                        \
    0.1645f /**< 大臂零点（初始态）与z轴夹角 (相对于z轴正方向，逆时针为正，rad) */
#define DEFAULT_ANGLE_2 0.1747f /**< 大臂与小臂夹角 (rad) */
#define DEFAULT_ANGLE_3                                                        \
    0.9155f /**< 吸盘初始角度 (相对于x轴正方向，逆时针为正，rad) */
#define DEFAULT_X       0.0f   /**< 机械臂基座系 X 轴偏置 (mm) */
#define DEFAULT_Z       0.0f   /**< 机械臂基座系 Z 轴偏置 (mm) */
#define DEFAULT_ARM_1_2 66.5f  /**< 大臂与小臂连接处距离 (即达妙4340长度，mm) */
#define DEFAULT_ARM3_X  85.72f /**< ARM3和吸盘的直线距离 (mm) */

/* ================== 控制周期与过渡参数 ================== */
#define ARM_CTRL_DT_DEFAULT      0.01f  /**< 控制周期 (10ms) */
#define ARM_TRANS_Z_OFFSET       100.0f /**< 过渡阶段 Z 轴抬高避障距离 (mm) */
#define ARM_TRANS_Y_OFFSET       80.0f  /**< 过渡阶段 Y 轴缩回避障距离 (mm) */

/* ================== 大臂电机 (J8006) 控制参数 ================== */
// #define ARM_J8006_CMD_SPEED_BASE 0.24f /**< 基础运动速度 (rad/s) */
// #define ARM_J8006_CMD_SPEED_MAX  0.55f /**< 绝对最大允许速度 (rad/s) */
#define ARM_J8006_CMD_SPEED_BASE 0.60f /**< 基础运动速度 (rad/s) */
#define ARM_J8006_CMD_SPEED_MAX  1.6f /**< 绝对最大允许速度 (rad/s) */
#define ARM_J8006_FILTER_ALPHA                                                 \
    0.99f /**< 低通滤波平滑系数 (越小越平滑但延迟大) */
#define ARM_J8006_CMD_RATE_LIMIT                                               \
    1.80f /**< 指令变化率限制 (rad/s)，防止阶跃信号 */
#define ARM_J8006_NEAR_ERR_RAD   0.08f /**< 接近目标时的减速触发阈值 (rad) */
#define ARM_J8006_FINE_ERR_RAD   0.03f /**< 微调阶段的误差阈值 (rad) */
#define ARM_J8006_NEAR_SPEED_MAX 0.90f /**< 接近阶段的最大速度限制 (rad/s) */
#define ARM_J8006_FINE_SPEED_MAX 0.80f /**< 微调阶段的最大速度限制 (rad/s) */
#define ARM_J8006_PHASE2_Q2TOQ1_SPEED_MAX                                      \
    0.6f /**< 象限翻转阶段的限速 (rad/s) */

#define ARM_BIG_ARM_FLIP_OVERSHOOT_FORCE_RAD                                   \
    0.43f /**< 跨越象限运动时的强制过冲量 (rad) */
#define ARM_PLACE_TAKEOUT_BIG_ARM_OVERSHOOT_RAD                                \
    0.4f /**< PLACE->TAKEOUT 时的过冲量 (rad) */
#define ARM_UPPER_TAKEOUT_BIG_ARM_OVERSHOOT_RAD                                \
    0.4f /**< 上中层取出态独立的过冲量 (rad) */
#define ARM_BIG_ARM_MIN_ANGLE_RAD                                              \
    0.12f /**< 大臂物理下限，禁止反向小于0.12 rad */

/* ================== 小臂与吸盘速度参数 ================== */
/* 速度模式：speed = Kp × |error|，下限 0，上限 _MAX */
#define ARM_SMALL_SPEED_KP              2.00f /**< 小臂比例增益 (rad/s/rad) */
#define ARM_SMALL_SPEED_MAX             1.60f /**< 小臂最大速度 (rad/s) */
#define ARM_SMALL_PLACE_EXIT_SPEED_KP   1.50f /**< 退出放置态小臂比例增益 */
#define ARM_SMALL_PLACE_EXIT_SPEED_MAX  1.60f /**< 退出放置态小臂最大速度 */
#define ARM_SMALL_CATCH_SPEED_KP        2.00f /**< 抓取动作时小臂比例增益 */
#define ARM_SMALL_CATCH_SPEED_MAX       1.60f /**< 抓取动作时小臂最大速度 */
#define ARM_BIG_PLACE_EXIT_SPEED_MAX    0.60f /**< 退出放置态大臂限速 */

#define ARM_SUCTION_SPEED_MAX           2.80f /**< 吸盘最大速度 */
#define ARM_SUCTION_FLIP_SLOW_SPEED_MAX 1.80f /**< 翻转时吸盘限速 */
#define ARM_BIG_ARM_READY_ERR_RAD                                              \
    0.20f /**< 等待大臂到达指定位置的允许误差 (rad) */
#define ARM_SMALL_ARM_READY_ERR_RAD                                            \
    0.07f /**< 等待小臂到达指定位置的允许误差 (rad) */
#define ARM_SUCTION_PLACE_SPEED_BASE      0.55f /**< 放置态吸盘基础速度 */
#define ARM_SUCTION_PLACE_SPEED_GAIN      1.10f /**< 放置态吸盘速度增益 */
#define ARM_SUCTION_PLACE_SPEED_MIN       0.70f /**< 放置态吸盘最小速度 */
#define ARM_SUCTION_PLACE_SPEED_MAX       1.50f /**< 放置态吸盘最大速度 */
#define ARM_SUCTION_PLACE_FINE_ERR_RAD    0.03f /**< 放置态吸盘微调误差 */
#define ARM_SUCTION_PLACE_FINE_SPEED_MIN  0.08f /**< 放置态吸盘微调最小速度 */
#define ARM_SUCTION_PLACE_BRAKE_SPEED_MAX 0.16f /**< 刹车阶段最大速度 */
#define ARM_PLACE_SUCTION_OFFSET_RAD      -0.0f /**< 放置下压预紧力 */

#define ARM_SUCTION_WAIT_HOLD_SPEED       1.20f  /**< 吸盘锁死保持速度 */
#define ARM_SUCTION_WAIT_TIMEOUT_MS       1000U /**< 吸盘等待超时时间 (ms) */
#define ARM_TAKEOUT_WAIT_TIMEOUT_MS       4000U /**< 取出动作超时时间 (ms) */
#define ARM_SMALL_WAIT_HOLD_SPEED         1.80f /**< 小臂锁死保持速度 */
#define ARM_SMALL_SPEED_DEADBAND_RAD      0.05f /**< 速度模式下小臂到位死区 (rad) */

/* ================== 俯瞰态速度参数 ================== */
#define ARM_OVERLOOK_BIG_SPEED            1.00f /**< 俯瞰态大臂速度 (rad/s) */
#define ARM_OVERLOOK_SMALL_SPEED          3.00f /**< 俯瞰态小臂速度 (rad/s) */
#define ARM_OVERLOOK_SUCTION_SPEED        2.20f /**< 俯瞰态吸盘速度 (rad/s) */

/* ================== 取出动作序列参数 ================== */
#define ARM_TAKEOUT_SEQ_BIG_ARM_STEP_RAD                                       \
    0.3746f /**< 取出时大臂预移动步长 (rad) */
#define ARM_TAKEOUT_SEQ_SMALL_ARM_STEP_RAD                                     \
    0.80f /**< 取出底层时小臂预移动步长 (rad) */
#define ARM_TAKEOUT_SEQ_SMALL_ARM_STEP_RAD_LAYER2                              \
    0.10f /**< 取出二层时小臂预移动步长 (rad) */
#define ARM_TAKEOUT_SEQ_LAYER0_SMALL_LOCK_RAD                                  \
    4.0f /**< 取出第一层时小臂中间锁定角度 (rad)，到达后等大臂到位再继续 */
#define ARM_TAKEOUT_START_LAYER            0    /**< 取出序列起始层数 (0~2)，逐层递减 */
#define ARM_TAKEOUT_SEQ_ERR_TOLERANCE_RAD                                      \
    0.12f /**< 序列步骤到位误差阈值 (rad) */


/**
 * @brief 机械臂工作状态枚举
 */
typedef enum {
    ARM_STATE_INIT = 0,    /**< 初始状态 */
    ARM_STATE_READY_1 = 1, /**< 就绪态1 (从下往上看) */
    ARM_STATE_READY_2 = 2, /**< 就绪态2 (从上往下看) */
    ARM_STATE_READY_3 = 3, /* 就绪态3 (向上40cm处，准备抓取) */
    ARM_STATE_READY_4 = 4, /**< 就绪态4 (从平面看，准备放置) */
    ARM_STATE_CATCH = 5,        /**< 抓取动作执行状态 */
    ARM_STATE_PLACE = 6,        /**< 放置动作执行状态 */
    ARM_STATE_WAIT_TAKEOUT = 7, /**< 待取出动作执行状态 */
    ARM_STATE_TAKEOUT_1 = 8,    /**< 取出动作执行状态 (放置二层) */
    ARM_STATE_TAKEOUT_2 = 9,    /**< 取出动作执行状态 (放置三层)*/
    ARM_STATE_OVERLOOK = 10,     /**< 俯瞰态 */
    ARM_STATE_CLOSE_PUMP = 11   /**< 关闭气泵状态 (特殊状态，用于反馈任务) */
} arm_status_t;

/**
 * @brief 机械臂目标控制模式枚举
 */
typedef enum {
    ARM_TARGET_CARTESIAN = 0, /**< 笛卡尔坐标系模式 */
    ARM_TARGET_JOINT = 1      /**< 关节空间模式 */
} arm_target_mode_t;

/**
 * @brief 机械臂运动状态机枚举 (集中管理所有运动状态)
 */
typedef enum {
    ARM_MOTION_STATE_IDLE = 0,            /**< 空闲状态 */
    ARM_MOTION_STATE_DIRECT_MOVE,         /**< 直接运动 (无过渡) */
    ARM_MOTION_STATE_MULTI_TRANS,         /**< 多点过渡运动 */
    ARM_MOTION_STATE_SINGLE_TRANS,        /**< 单点过渡运动 */
    ARM_MOTION_STATE_OVERSHOOT_PHASE1,    /**< 过冲阶段1 (执行过冲) */
    ARM_MOTION_STATE_OVERSHOOT_PHASE2,    /**< 过冲阶段2 (恢复位置) */
    ARM_MOTION_STATE_PLACE_OVERSHOOT,     /**< 放置态：三点过渡后过冲 */
    ARM_MOTION_STATE_PLACE_SMALL_SUCTION, /**< 放置态：小臂吸盘运动阶段 */
    ARM_MOTION_STATE_PLACE_BIG_RECOVER,   /**< 放置态：大臂回位阶段 */
    ARM_MOTION_STATE_TAKEOUT_SEQ_BIG,     /**< 取出序列: 大臂过冲 */
    ARM_MOTION_STATE_TAKEOUT_SEQ_SMALL,   /**< 取出序列: 小臂旋转 */
    ARM_MOTION_STATE_TAKEOUT_SEQ_FINAL,   /**< 取出序列: 大臂回位 */
    ARM_MOTION_STATE_TAKEOUT_SEQ_SUCTION, /**< 取出序列: 小臂吸盘协同 */
} arm_motion_state_t;

/**
 * @brief 锁存等待类型枚举
 */
typedef enum {
    ARM_LATCH_NONE = 0,             /**< 无锁存 */
    ARM_LATCH_SUCTION_WAIT,         /**< 吸盘等待小臂到位 */
    ARM_LATCH_SMALL_ARM_WAIT,       /**< 小臂等待吸盘到位 */
    ARM_LATCH_TAKEOUT_WAIT,         /**< 取出等待大臂抬起 */
    ARM_LATCH_PLACE_OVERSHOOT_WAIT, /**< 放置态过冲阶段：大臂过冲，小臂吸盘不锁死 */
    ARM_LATCH_PLACE_SMALL_SUCTION, /**< 放置态小臂吸盘阶段：小臂吸盘运动，大臂锁死 */
    ARM_LATCH_PLACE_BIG_RECOVER, /**< 放置态大臂回位阶段：大臂回位，小臂吸盘锁死 */
    ARM_LATCH_SEQ_BIG_ARM_ONLY,   /**< 取出序列：仅大臂运动，小臂和吸盘锁定 */
    ARM_LATCH_SEQ_SMALL_ARM_ONLY, /**< 取出序列：仅小臂运动，大臂和吸盘锁定 */
    ARM_LATCH_SEQ_BIG_LOCKED,     /**< 取出序列Step3：大臂锁定，小臂+吸盘协同运动 */
} arm_latch_type_t;

/**
 * @brief 大臂过冲阶段枚举
 */
typedef enum {
    ARM_OVERSHOOT_PHASE_NONE = 0,  /**< 无过冲 */
    ARM_OVERSHOOT_PHASE_OVERSHOOT, /**< 过冲执行中 */
    ARM_OVERSHOOT_PHASE_RECOVER,   /**< 过冲恢复中 */
} arm_overshoot_phase_t;

/**
 * @brief 机械臂核心控制结构体
 * 包含电机句柄、目标点位、实时状态、状态机变量及运行上下文。
 */
typedef struct {
    /* 电机驱动句柄定义 */
    dm_handle_t damiao_1; /**< 大臂关节电机1 (双电机同轴驱动) */
    dm_handle_t
        damiao_2; /**< 大臂关节电机2 (双电机同轴驱动，与电机1互为反向) */
    dm_handle_t damiao_3; /**< 小臂关节电机 */
    dm_handle_t damiao_4; /**< 末端吸盘/夹爪电机 */

    /* 目标点位变量 (笛卡尔空间/关节空间) */
    float arm_target_y;        /**< 当前正在追踪的过渡/中间目标 Y 坐标 */
    float arm_target_z;        /**< 当前正在追踪的过渡/中间目标 Z 坐标 */
    float arm_target_pitch;    /**< 当前正在追踪的过渡/中间目标 Pitch 倾角 */
    float arm_joint_target[3]; /**< 关节模式下的目标角度数组 */

    /* 最终目标点位暂存 (用于过渡动作完成后恢复最终目标) */
    float final_target_y;
    float final_target_z;
    float final_target_pitch;

    /* 实时姿态反馈 (通过正向运动学 FK 解算得出) */
    float current_y;
    float current_z;
    float current_pitch;

    /* 多点过渡控制参数 (专门针对 CATCH -> PLACE 这种需要避障的长距离运动) */
    uint8_t multi_trans_index; /**< 当前执行到的过渡点位索引 */
    uint8_t multi_trans_count; /**< 总共设定的过渡点数量 */
    float trans_y[4];          /**< 过渡点 Y 坐标数组 */
    float trans_z[4];          /**< 过渡点 Z 坐标数组 */
    float trans_pitch[4];      /**< 过渡点 Pitch 姿态数组 */

    /* 运行状态反馈与模式 */
    uint8_t
        arm_motion_active; /**< 机械臂运动活跃标志 (1:正在移动, 0:已到达目标并静止) */
    arm_target_mode_t
        target_mode;     /**< 当前采用的控制模式 (坐标系解算/关节直驱) */
    arm_status_t status; /**< 当前应用层设定的目标状态 */
    arm_status_t
        last_status; /**< 上一时刻的状态机状态 (用于检测状态切换边沿) */
    arm_motion_state_t motion_state; /**< 集中式运动状态机 */

    /* 取出动作序列控制 */
    float takeout_seq_big_arm_target;   /**< 取出序列大臂目标（Step3/4: 最终角度） */
    float takeout_seq_small_arm_target; /**< 取出序列小臂目标（Step4: 最终角度） */
    float takeout_seq_suction_target;   /**< 取出序列吸盘目标（Step4: 最终角度） */
    uint32_t takeout_wait_start_tick;   /**< 取出等待开始时间戳 */
    float
        takeout_target_suction_angle; /**< 待取出态目标吸盘角度 (解算值，用于TAKEOUT切换时锁定) */

    /* 大臂过冲控制 (用于跨越奇点或特定姿态时的力矩补偿) */
    float big_arm_overshoot_rad;     /**< 设定的过冲角度大小 (弧度) */
    float big_arm_final_joint;       /**< 过冲结束后的最终目标角度 */
    float big_arm_overshoot_joint;   /**< 叠加过冲量后的临时目标角度 */
    uint8_t big_arm_overshoot_armed; /**< 过冲动作使能标志 (1:准备执行过冲) */
    arm_overshoot_phase_t big_arm_overshoot_phase; /**< 过冲所处阶段 */

    /* 内部运行上下文与滤波缓冲 */
    float ctrl_dt; /**< 控制周期时间 (秒)，用于速度和积分计算 */
    float
        joint_cmd_prev[3]; /**< 上一控制周期的指令下发值 (用于计算导数或平滑) */

    float big_arm_cmd_filtered;   /**< 大臂指令位置的低通滤波结果 */
    float big_arm_speed_filtered; /**< 大臂指令速度的低通滤波结果 */

    /* 锁存等待逻辑参数 (用于等待某一个关节到达指定位置再继续后续动作) */
    arm_latch_type_t latch_type;     /**< 当前锁存类型 */
    float big_arm_wait_locked_pos;   /**< 大臂等待期间的锁定角度 */
    float suction_wait_locked_pos;   /**< 吸盘等待期间的锁定角度 */
    float small_arm_wait_locked_pos; /**< 小臂等待期间的锁定角度 */
    uint32_t
        suction_wait_start_tick; /**< 吸盘等待动作的开始时间戳 (用于超时强制退出) */

    /* 象限跟踪 */
    int8_t
        last_target_quadrant; /**< 上一次目标所处的象限 (1: 正向象限, -1: 反向象限) */
    int8_t flip_transition_dir; /**< 机械臂象限翻转的方向指示 */

    /* 放置层数 (0=底层, 1=中层, 2=顶层) */
    uint8_t place_layer;   /**< 当前放置目标层数，用于选择过渡点 */
    uint8_t takeout_layer; /**< 当前取出层数 (0~2)，逐层递减，独立于放置 */

    /* 位域压缩标志位 (节省内存，优化布尔变量存储) */
    struct {
        uint16_t joint_cmd_inited : 1;      /**< 初始关节指令是否已初始化 */
        uint16_t big_arm_filter_inited : 1; /**< 大臂滤波器是否已初始化 */
        uint16_t has_transition : 1;        /**< 当前动作是否需要执行单点过渡 */
        uint16_t transition_done : 1;       /**< 单点过渡动作是否已执行完成 */
        uint16_t place_exit_safety_active
            : 1; /**< 从放置态退出时，启用安全过渡点与专用速度策略 */
        uint16_t place_entry_sync_active
            : 1; /**< 进入放置态/待取出态时，启用小臂与吸盘的同步等待逻辑，确保先到位后动作 */
        uint16_t small_arm_wait_locked_inited
            : 1; /**< 小臂等待锁存位置是否已初始化记录 */
        uint16_t suction_wait_locked_inited
            : 1; /**< 吸盘等待锁存位置是否已初始化记录 */
        uint16_t big_arm_wait_locked_inited
            : 1; /**< 大臂等待锁存位置是否已初始化记录 */
    } flags;

} RobotArm;

/* ================== API 声明 ================== */

void robot_arm_system_init(RobotArm *arm);
void robot_arm_update(RobotArm *arm);
void arm_apply_ctrl(RobotArm *arm, const float joint_des[3]);
void robot_arm_set_target(RobotArm *arm, float y, float z, float pitch);
void robot_arm_set_joint_target(RobotArm *arm, float joint1, float joint2,
                                float joint3);
void robot_arm_set_big_arm_overshoot(RobotArm *arm, float overshoot_rad);
void robot_arm_set_ctrl_dt(RobotArm *arm, float dt_s);
uint8_t robot_arm_start_takeout_sequence(RobotArm *arm, float y, float z,
                                         float pitch,
                                         float wait_takeout_suction_angle);
uint8_t robot_arm_is_takeout_sequence_active(RobotArm *arm);
void robot_arm_fk(float joint1, float joint2, float joint3, float *y_out,
                  float *z_out, float *pitch_out);
void arm_pos_angle(float x1, float z1, float pitch_angle, float angle[3]);
uint8_t arm_is_ready_state(arm_status_t status);
uint8_t arm_is_takeout_state(arm_status_t status);

#endif /* ROBOT_ARM_H */
