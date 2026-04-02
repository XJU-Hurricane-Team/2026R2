/**
 * @file    chassis_calculations.c
 * @author  Jackrainman
 * @brief   底盘速度规划模块（梯形/余弦/多项式）
 * @version 2.1
 * @date    2026-03-02
 */

#include "chassis_calculations.h"

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#ifndef PI
#define PI 3.14159265358979f          /* 圆周率常量 */
#endif

#define AXIS_NUM                 3     /* 规划轴数量：X/Y/W */
#define AXIS_X                   0     /* X 轴索引 */
#define AXIS_Y                   1     /* Y 轴索引 */
#define AXIS_W                   2     /* 角速度轴(W)索引 */

#define DEFAULT_ACCEL_XY         40.0f /* 默认平面合加速度上限(mm/s^2) */
#define DEFAULT_ACCEL_W          20.0f /* 默认角加速度上限(rad/s^2) */
#define DEFAULT_SPEED_XY         5000.0f /* 默认平面速度模长上限(mm/s) */
#define DEFAULT_SPEED_W          10.0f /* 默认角速度上限(rad/s) */

#define MIN_ACCEL                0.001f /* 加速度最小保护值，防止除零或极小值不稳定 */
#define SPEED_EPS                0.001f /* 速度比较的近似零阈值 */
#define MAX_DT_SEC               0.1f   /* 单步最大有效时间间隔(秒)，超过视为异常步长 */
#define POLY_DSHAPE_MAX          1.875f /* 五次多项式速度曲线导数峰值系数 */

typedef struct {    /* 轴规划状态结构体 */
    float start;    /* 起点速度 */
    float target;   /* 终点速度 */
    float duration; /* 规划时长 */
    float elapsed;  /* 当前已规划时长 */
    uint8_t active; /* 是否在跑轨迹，给每个州保存 “非梯形模式” 的轨迹进度状态 */
} axis_profile_state_t;

typedef struct {
    chassis_plan_config_t cfg;      /* 当前配置 */
    chassis_speed_plan_type_t mode;  /* 当前模式 */
    float speed[AXIS_NUM];      /* 当前速度（每个轴） */
    float last_vx;              /* 上一次的 X 轴速度 */
    float last_vy;              /* 上一次的 Y 轴速度 */
    axis_profile_state_t profile[AXIS_NUM]; /* 轴规划状态 */
    uint32_t last_tick;         /* 上一次规划的节拍 */
    uint8_t first_update;       /* 是否首次更新 */
} chassis_plan_ctx_t;

static chassis_plan_ctx_t g_plan = {    /* 默认配置，全局实例 */
    .cfg = {
        .max_accel_xy = DEFAULT_ACCEL_XY,
        .max_accel_w = DEFAULT_ACCEL_W,
        .max_speed_xy = DEFAULT_SPEED_XY,
        .max_speed_w = DEFAULT_SPEED_W
    },
    .mode = CHASSIS_SPEED_PLAN_COSINE,
    .first_update = 1
};

/* ========================================================================== */
/*                              基础数学工具函数                          */
/* ========================================================================== */

/* 计算浮点绝对值。 */
static float absf(float v) {
    return (v >= 0.0f) ? v : -v;
}

/* 将浮点值限制在 [low, high] 区间内。 */
static float clampf(float v, float low, float high) {     /* 上/下限保护，避免越界值进入后续控制 */
    if (v < low) return low;
    if (v > high) return high;
    return v;
}

/* 对单轴速度做对称限幅，vmax<=0 表示不限制。 */
static float limit_axis_speed(float v, float vmax) {
    if (vmax <= 0.0f) {
        return v;
    }
    return clampf(v, -vmax, vmax);
}

/* ========================================================================== */
/*                              配置与时间基函数                          */
/* ========================================================================== */

/* 清洗配置参数，保证加速度和速度上限处于有效范围。 */
static void sanitize_plan_config(chassis_plan_config_t *cfg) {
    if (cfg->max_accel_xy < MIN_ACCEL) cfg->max_accel_xy = MIN_ACCEL;
    if (cfg->max_accel_w < MIN_ACCEL) cfg->max_accel_w = MIN_ACCEL;
    if (cfg->max_speed_xy < 0.0f) cfg->max_speed_xy = 0.0f;
    if (cfg->max_speed_w < 0.0f) cfg->max_speed_w = 0.0f;
}

/* 根据 RTOS tick 计算本次控制周期 dt(秒)，并屏蔽异常步长。 */
static float compute_dt_sec(uint32_t now_tick) {
    uint32_t delta_tick;
    float dt;

    if (g_plan.first_update) {
        g_plan.last_tick = now_tick;
        g_plan.first_update = 0u;
        return 0.0f;
    }

    delta_tick = now_tick - g_plan.last_tick;
    g_plan.last_tick = now_tick;
    /* FreeRTOS 系统节拍固定为 1ms/tick， dt 直接按毫秒换算为秒 */
    dt = (float)delta_tick * 0.001f;
    if (dt <= 0.0f || dt > MAX_DT_SEC) {
        return 0.0f;
    }
    return dt;
}

/* ========================================================================== */
/*                           轨迹模型参数与形状函数                       */
/* ========================================================================== */

/* 按余弦轨迹模型计算完成 dv 所需时长。 */
static float compute_cosine_duration(float dv, float amax) {
    if (dv <= SPEED_EPS || amax < MIN_ACCEL) {
        return 0.0f;
    }
    return (PI * dv) / (2.0f * amax);
}

/* 按五次多项式轨迹模型计算完成 dv 所需时长。 */
static float compute_poly_duration(float dv, float amax) {
    if (dv <= SPEED_EPS || amax < MIN_ACCEL) {
        return 0.0f;
    }
    return (POLY_DSHAPE_MAX * dv) / amax;
}

/* 余弦轨迹形状函数：输入归一化进度 p，输出插值比例。 */
static float eval_cosine_shape(float p) {
    return 0.5f * (1.0f - cosf(PI * p));
}

/* 五次多项式轨迹形状函数：输入归一化进度 p，输出插值比例。 */
static float eval_poly_shape(float p) {
    float p2 = p * p;
    float p3 = p2 * p;
    return p3 * (10.0f + p * (-15.0f + 6.0f * p));
}

/* ========================================================================== */
/*                              单轴速度步进函数                          */
/* ========================================================================== */

/* 梯形模式下的单轴一步更新：按 max_accel*dt 限制每步速度变化量。 */
static float step_trapezoid_axis(float current, float target, float max_accel, float dt) {
    float dv = target - current;
    float max_dv = max_accel * dt;
    if (dv > max_dv) return current + max_dv;
    if (dv < -max_dv) return current - max_dv;
    return target;
}

/* 通用轨迹一步更新：用于余弦/多项式模式，维护单轴轨迹状态并输出当前速度。 */
static float step_profile_axis(axis_profile_state_t *state, float current, float target,
                               float max_accel, float dt,
                               float (*compute_duration)(float, float),
                               float (*eval_shape)(float)) {
    if (absf(target - state->target) > SPEED_EPS || !state->active) {
        float duration = compute_duration(absf(target - current), max_accel);
        state->start = current;
        state->target = target;
        state->duration = duration;
        state->elapsed = 0.0f;
        state->active = (duration > 0.0f) ? 1u : 0u;
    }

    if (!state->active || state->duration <= 0.0f) {
        return target;
    }

    state->elapsed += dt;
    if (state->elapsed >= state->duration) {
        state->active = 0u;
        return state->target;
    }

    {
        float p = clampf(state->elapsed / state->duration, 0.0f, 1.0f);
        return state->start + (state->target - state->start) * eval_shape(p);
    }
}

/* ========================================================================== */
/*                              整车输出约束函数                          */
/* ========================================================================== */

/* 对输出速度做限幅：XY 按模长限速，W 按单轴限速。 */
static void limit_output_speed(void) {
    float vmax_xy = g_plan.cfg.max_speed_xy;
    if (vmax_xy > 0.0f) {
        float vx = g_plan.speed[AXIS_X];
        float vy = g_plan.speed[AXIS_Y];
        float v2 = vx * vx + vy * vy;
        float vmax2 = vmax_xy * vmax_xy;
        if (v2 > vmax2 && v2 > 0.0f) {
            float scale = vmax_xy / sqrtf(v2);
            g_plan.speed[AXIS_X] = vx * scale;
            g_plan.speed[AXIS_Y] = vy * scale;
        }
    }
    g_plan.speed[AXIS_W] = limit_axis_speed(g_plan.speed[AXIS_W], g_plan.cfg.max_speed_w);
}

/* 平面合加速度限幅：若 sqrt(ax^2 + ay^2) 超限，则按同方向缩放到上限 */
static void limit_planar_accel(float dt) {
    float ax;
    float ay;
    float acc;
    float limit = g_plan.cfg.max_accel_xy;

    if (dt <= 0.0f) {
        return;
    }

    if (limit <= 0.0f) {
        g_plan.last_vx = g_plan.speed[AXIS_X];
        g_plan.last_vy = g_plan.speed[AXIS_Y];
        return;
    }

    ax = (g_plan.speed[AXIS_X] - g_plan.last_vx) / dt;
    ay = (g_plan.speed[AXIS_Y] - g_plan.last_vy) / dt;
    acc = sqrtf(ax * ax + ay * ay);
    if (acc > limit && acc > 0.0f) {
        float scale = limit / acc;
        ax *= scale;
        ay *= scale;
        g_plan.speed[AXIS_X] = g_plan.last_vx + ax * dt;
        g_plan.speed[AXIS_Y] = g_plan.last_vy + ay * dt;
    }

    g_plan.last_vx = g_plan.speed[AXIS_X];
    g_plan.last_vy = g_plan.speed[AXIS_Y];
}

/* ========================================================================== */
/*                           对外接口：初始化与配置                       */
/* ========================================================================== */

/* 填充规划器默认配置。 */
void chassis_plan_config_default(chassis_plan_config_t *cfg) {
    if (cfg == NULL) {
        return;
    }

    cfg->max_accel_xy = DEFAULT_ACCEL_XY;
    cfg->max_accel_w = DEFAULT_ACCEL_W;
    cfg->max_speed_xy = DEFAULT_SPEED_XY;
    cfg->max_speed_w = DEFAULT_SPEED_W;
}

/* 重置运行时状态与轨迹状态，通常在初始化或模式重建时调用。 */
void chassis_plan_reset_state(float speed_x, float speed_y, float speed_w) {
    g_plan.speed[AXIS_X] = speed_x;
    g_plan.speed[AXIS_Y] = speed_y;
    g_plan.speed[AXIS_W] = speed_w;
    g_plan.last_vx = speed_x;
    g_plan.last_vy = speed_y;
    g_plan.last_tick = 0u;
    g_plan.first_update = 1u;
    memset(g_plan.profile, 0, sizeof(g_plan.profile));
}

/* 创建/初始化规划器上下文，应用配置并设置初始模式。 */
void chassis_plan_init(const chassis_plan_config_t *cfg, chassis_speed_plan_type_t mode) {
    if (cfg != NULL) {
        g_plan.cfg = *cfg;
    } else {
        chassis_plan_config_default(&g_plan.cfg);
    }
    sanitize_plan_config(&g_plan.cfg);

    if (mode >= CHASSIS_SPEED_PLAN_NUM) {
        mode = CHASSIS_SPEED_PLAN_COSINE;
    }
    g_plan.mode = mode;
    chassis_plan_reset_state(0.0f, 0.0f, 0.0f);
}

/* ========================================================================== */
/*                          对外接口：模式与限制设置                      */
/* ========================================================================== */

/* 设置速度规划模式，模式变化时清空轨迹缓存。 */
void chassis_plan_mode_set(chassis_speed_plan_type_t mode) {
    if (mode >= CHASSIS_SPEED_PLAN_NUM || mode == g_plan.mode) {
        return;
    }
    g_plan.mode = mode;
    memset(g_plan.profile, 0, sizeof(g_plan.profile));
}

/* 获取当前速度规划模式。 */
chassis_speed_plan_type_t chassis_plan_mode_get(void) {
    return g_plan.mode;
}

/* 设置加速度约束（XY 与 W）。 */
void chassis_plan_limit_accel_set(float accel_xy, float accel_w) {
    g_plan.cfg.max_accel_xy = (accel_xy < MIN_ACCEL) ? MIN_ACCEL : accel_xy;
    g_plan.cfg.max_accel_w = (accel_w < MIN_ACCEL) ? MIN_ACCEL : accel_w;
}

/* 设置速度约束（XY 与 W）。 */
void chassis_plan_limit_speed_set(float speed_xy, float speed_w) {
    g_plan.cfg.max_speed_xy = (speed_xy < 0.0f) ? 0.0f : speed_xy;
    g_plan.cfg.max_speed_w = (speed_w < 0.0f) ? 0.0f : speed_w;
}

/* ========================================================================== */
/*                             对外接口：主执行入口                       */
/* ========================================================================== */

/* 执行一次速度规划主流程并输出本周期结果。 */
void chassis_plan_step(float target_x, float target_y, float target_yaw,
                       float *output_x, float *output_y, float *output_yaw) {
    float dt = compute_dt_sec(xTaskGetTickCount());

    if (dt > 0.0f) {
        float target[AXIS_NUM] = {target_x, target_y, target_yaw};
        float accel[AXIS_NUM] = {g_plan.cfg.max_accel_xy, g_plan.cfg.max_accel_xy, g_plan.cfg.max_accel_w};
        uint8_t i;

        for (i = 0u; i < AXIS_NUM; i++) {
            if (g_plan.mode == CHASSIS_SPEED_PLAN_TRAPEZOID) {
                g_plan.speed[i] = step_trapezoid_axis(g_plan.speed[i], target[i], accel[i], dt);
            } else if (g_plan.mode == CHASSIS_SPEED_PLAN_COSINE) {
                g_plan.speed[i] = step_profile_axis(&g_plan.profile[i], g_plan.speed[i], target[i],
                                                    accel[i], dt, compute_cosine_duration, eval_cosine_shape);
            } else {
                g_plan.speed[i] = step_profile_axis(&g_plan.profile[i], g_plan.speed[i], target[i],
                                                    accel[i], dt, compute_poly_duration, eval_poly_shape);
            }
        }

        limit_output_speed();
        limit_planar_accel(dt);
    }

    if (output_x != NULL) *output_x = g_plan.speed[AXIS_X];
    if (output_y != NULL) *output_y = g_plan.speed[AXIS_Y];
    if (output_yaw != NULL) *output_yaw = g_plan.speed[AXIS_W];
}
