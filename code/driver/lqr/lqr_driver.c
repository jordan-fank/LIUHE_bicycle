/*
 * lqr_driver.c
 *
 * LQR平衡控制器驱动层实现
 *
 * Created on: 2025
 * Author: suiyungui
 */

#include "lqr_driver.h"

// ==================== 辅助函数 ====================

/**
 * @brief  浮点数限幅
 */
static float lqr_constrain_float(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// ==================== 默认LQR增益表（离线生成） ====================
// 生成条件：
// 1. 物理参数初值使用 lqr_driver.h 中的默认宏：
//    LQR_DEFAULT_COM_HEIGHT_M / LQR_DEFAULT_WHEELBASE_M / LQR_DEFAULT_GRAVITY_M_S2
// 2. LQR权重：Q=diag(100.0, 15.0, 30.0), R=2.0
// 3. 状态: x = [phi, phi_dot, delta], 输入: u = delta_dot
// 4. K = [K_phi, K_phi_dot, K_delta]
//
// 注意：
// 1. 这张表不是“物理真理”，只是当前项目接入时使用的默认表
// 2. 如果你的实车 h / L / 速度范围明显不同，这张表需要重算，而不是只改应用层参数

static const LQR_GainEntry_t default_gain_entries[] = {
    {0.5f,  {-214.7704f, -23.9102f, 22.1226f}},  // 低速
    {1.0f,  {-64.0971f, -7.5628f, 23.9397f}},
    {1.4f,  {-36.5193f, -4.8185f, 28.2038f}},
    {1.9f,  {-26.8197f, -3.9618f, 33.7869f}},
    {2.4f,  {-22.0014f, -3.5798f, 39.9194f}},
    {2.8f,  {-19.1137f, -3.3706f, 46.3089f}},
    {3.3f,  {-17.1817f, -3.2408f, 52.8370f}},
    {3.8f,  {-15.7938f, -3.1531f, 59.4484f}},
    {4.2f,  {-14.7464f, -3.0904f, 66.1137f}},
    {4.7f,  {-13.9266f, -3.0435f, 72.8160f}},
    {5.1f,  {-13.2669f, -3.0072f, 79.5449f}},
    {5.6f,  {-12.7242f, -2.9783f, 86.2936f}},
    {6.1f,  {-12.2697f, -2.9549f, 93.0573f}},
    {6.5f,  {-11.8834f, -2.9355f, 99.8327f}},
    {7.0f,  {-11.5509f, -2.9192f, 106.6175f}},  // 高速
};

#define DEFAULT_GAIN_COUNT  15

// ==================== 函数实现 ====================

/**
 * @brief  初始化LQR控制器
 */
void lqr_init(LQR_Controller_t *ctrl)
{
    uint8_t i;

    // 默认物理参数：与默认增益表的生成条件保持一致
    ctrl->params.h = LQR_DEFAULT_COM_HEIGHT_M;
    ctrl->params.L = LQR_DEFAULT_WHEELBASE_M;
    ctrl->params.b = LQR_DEFAULT_TRAIL_M;
    ctrl->params.g = LQR_DEFAULT_GRAVITY_M_S2;

    // 默认限幅
    ctrl->delta_max = LQR_DEFAULT_DELTA_MAX_RAD;
    ctrl->delta_dot_max = LQR_DEFAULT_DELTA_DOT_MAX_RAD_S;
    ctrl->v_min = LQR_DEFAULT_V_MIN_M_S;

    // 初始化状态
    ctrl->state.phi = 0.0f;
    ctrl->state.phi_dot = 0.0f;
    ctrl->state.delta = 0.0f;

    ctrl->delta_cmd = 0.0f;
    ctrl->delta_dot = 0.0f;
    ctrl->target_delta = 0.0f;
    ctrl->u_raw = 0.0f;
    ctrl->enabled = 0;
    ctrl->gain_valid = 0;
    ctrl->current_velocity = 0.0f;

    // 初始化当前增益
    for (i = 0; i < LQR_STATE_DIM; i++) {
        ctrl->K[i] = 0.0f;
    }

    // 加载默认增益表
    ctrl->gain_table.size = (uint8_t)DEFAULT_GAIN_COUNT;
    for (i = 0; i < DEFAULT_GAIN_COUNT && i < LQR_GAIN_TABLE_SIZE; i++) {
        ctrl->gain_table.entries[i] = default_gain_entries[i];
    }
}

/**
 * @brief  设置物理参数
 */
void lqr_set_physical_params(LQR_Controller_t *ctrl, float h, float L, float b)
{
    ctrl->params.h = h;
    ctrl->params.L = L;
    ctrl->params.b = b;
}

/**
 * @brief  设置增益表
 */
void lqr_set_gain_table(LQR_Controller_t *ctrl, const LQR_GainTable_t *table)
{
    uint8_t i;
    ctrl->gain_table.size = table->size;
    for (i = 0; i < table->size && i < LQR_GAIN_TABLE_SIZE; i++) {
        ctrl->gain_table.entries[i] = table->entries[i];
    }
}

/**
 * @brief  设置限幅参数
 */
void lqr_set_limits(LQR_Controller_t *ctrl, float delta_max, float delta_dot_max, float v_min)
{
    ctrl->delta_max = delta_max;
    ctrl->delta_dot_max = delta_dot_max;
    ctrl->v_min = v_min;
}

/**
 * @brief  根据速度更新增益（线性插值）
 * @return 0=成功，-1=速度过低
 */
int8_t lqr_update_gain(LQR_Controller_t *ctrl, float velocity)
{
    LQR_GainTable_t *table = &ctrl->gain_table;
    uint8_t i, idx;
    float v1, v2, t;
    float k1, k2;

    ctrl->current_velocity = velocity;

    // 低速保护
    if (velocity < ctrl->v_min) {
        ctrl->gain_valid = 0;
        return -1;
    }

    // 边界处理：速度低于最小点
    if (velocity <= table->entries[0].velocity) {
        for (i = 0; i < LQR_STATE_DIM; i++) {
            ctrl->K[i] = table->entries[0].K[i];
        }
        ctrl->gain_valid = 1;
        return 0;
    }

    // 边界处理：速度高于最大点
    if (velocity >= table->entries[table->size - 1].velocity) {
        for (i = 0; i < LQR_STATE_DIM; i++) {
            ctrl->K[i] = table->entries[table->size - 1].K[i];
        }
        ctrl->gain_valid = 1;
        return 0;
    }

    // 查找插值区间
    idx = 0;
    for (i = 0; i < table->size - 1; i++) {
        if (velocity >= table->entries[i].velocity &&
            velocity < table->entries[i + 1].velocity) {
            idx = i;
            break;
        }
    }

    // 线性插值
    v1 = table->entries[idx].velocity;
    v2 = table->entries[idx + 1].velocity;
    t = (velocity - v1) / (v2 - v1);  // 插值因子 [0, 1]

    for (i = 0; i < LQR_STATE_DIM; i++) {
        k1 = table->entries[idx].K[i];
        k2 = table->entries[idx + 1].K[i];
        ctrl->K[i] = k1 + t * (k2 - k1);
    }

    ctrl->gain_valid = 1;
    return 0;
}

/**
 * @brief  执行LQR状态反馈计算
 *
 * 增广状态LQR模型：
 *   状态: x = [phi, phi_dot, delta]
 *   输入: u = delta_dot
 *   控制律: u = -K * x
 *
 * @param  phi: 侧倾角（rad）
 * @param  phi_dot: 侧倾角速度（rad/s）
 * @return 转向角命令delta_cmd（rad）
 */
float lqr_compute(LQR_Controller_t *ctrl, float phi, float phi_dot)
{
    float u;

    if (!ctrl->enabled || !ctrl->gain_valid) {
        return ctrl->delta_cmd;  // 保持当前值
    }

    // 更新状态
    ctrl->state.phi = phi;
    ctrl->state.phi_dot = phi_dot;
    // delta从上一次的积分值获取
    ctrl->state.delta = ctrl->delta_cmd;

    // LQR状态反馈：u = -K * (x - x_target)
    // u 是转向角速度 delta_dot
    // phi和phi_dot的目标已在应用层处理，这里处理delta的目标
    // error_delta = current_delta - target_delta
    float error_delta = ctrl->state.delta - ctrl->target_delta;
    u = -(ctrl->K[0] * phi +
          ctrl->K[1] * phi_dot +
          ctrl->K[2] * error_delta);

    ctrl->u_raw = u;

    // 转向角速度限幅
    u = lqr_constrain_float(u, -ctrl->delta_dot_max, ctrl->delta_dot_max);
    ctrl->delta_dot = u;

    // 积分得到转向角命令
    ctrl->delta_cmd += u * LQR_DT;

    // 转向角限幅
    ctrl->delta_cmd = lqr_constrain_float(ctrl->delta_cmd, -ctrl->delta_max, ctrl->delta_max);

    return ctrl->delta_cmd;
}

/**
 * @brief  重置LQR控制器
 */
void lqr_reset(LQR_Controller_t *ctrl)
{
    ctrl->state.phi = 0.0f;
    ctrl->state.phi_dot = 0.0f;
    ctrl->state.delta = 0.0f;
    ctrl->delta_cmd = 0.0f;
    ctrl->delta_dot = 0.0f;
    ctrl->target_delta = 0.0f;
    ctrl->u_raw = 0.0f;
}

/**
 * @brief  使能/禁用LQR控制器
 */
void lqr_enable(LQR_Controller_t *ctrl, uint8_t enable)
{
    if (enable && !ctrl->enabled) {
        lqr_reset(ctrl);  // 使能时重置
    }
    ctrl->enabled = enable;
}

/**
 * @brief  获取当前增益值（调试用）
 */
void lqr_get_current_gain(LQR_Controller_t *ctrl, float *k_out)
{
    uint8_t i;
    for (i = 0; i < LQR_STATE_DIM; i++) {
        k_out[i] = ctrl->K[i];
    }
}
