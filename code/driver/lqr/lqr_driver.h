/*
 * lqr_driver.h
 *
 * LQR平衡控制器驱动层
 * 基于Cornell点质量模型（增广状态LQR）
 *
 * 状态向量: x = [phi, phi_dot, delta]
 * 控制输入: u = delta_dot（转向角速度）
 *
 * Created on: 2025
 * Author: suiyungui
 */

#ifndef CODE_DRIVER_LQR_LQR_DRIVER_H_
#define CODE_DRIVER_LQR_LQR_DRIVER_H_

#include "stdint.h"

// ==================== 配置参数 ====================
#define LQR_GAIN_TABLE_SIZE     15       // 增益查表点数（最大）
#define LQR_STATE_DIM           3       // 状态维度
#define LQR_DT                  0.005f  // 控制周期（5ms）

// ==================== 物理参数结构体 ====================
typedef struct {
    float h;            // 质心高度（m）
    float L;            // 轴距（m）
    float b;            // 拖曳距（m）
    float g;            // 重力加速度（m/s^2）
} LQR_PhysicalParams_t;

// ==================== 增益表条目 ====================
typedef struct {
    float velocity;                 // 速度点（m/s）
    float K[LQR_STATE_DIM];         // 对应增益 [K_phi, K_phi_dot, K_delta]
} LQR_GainEntry_t;

// ==================== LQR增益表 ====================
typedef struct {
    uint8_t size;                                   // 有效条目数
    LQR_GainEntry_t entries[LQR_GAIN_TABLE_SIZE];   // 按速度升序排列
} LQR_GainTable_t;

// ==================== LQR状态结构体 ====================
typedef struct {
    float phi;          // 侧倾角（rad）
    float phi_dot;      // 侧倾角速度（rad/s）
    float delta;        // 转向角（rad）
} LQR_State_t;

// ==================== LQR控制器结构体 ====================
typedef struct {
    // 物理参数（可配置）
    LQR_PhysicalParams_t params;

    // 增益表
    LQR_GainTable_t gain_table;

    // 当前状态
    LQR_State_t state;

    // 当前增益（插值后）
    float K[LQR_STATE_DIM];

    // 控制输出
    float delta_dot;        // 转向角速度（rad/s）
    float delta_cmd;        // 积分后的转向角命令（rad）
    float target_delta;     // 目标转向角（rad），用于压弯控制

    // 限幅参数
    float delta_max;        // 最大转向角（rad）
    float delta_dot_max;    // 最大转向角速度（rad/s）
    float v_min;            // 最小有效速度（m/s）

    // 状态标志
    uint8_t enabled;        // 控制器使能
    uint8_t gain_valid;     // 增益有效标志

    // 调试信息
    float current_velocity; // 当前速度
    float u_raw;            // 原始控制输出（限幅前）
} LQR_Controller_t;

// ==================== 函数声明 ====================

/**
 * @brief  初始化LQR控制器
 * @param  ctrl: LQR控制器指针
 */
void lqr_init(LQR_Controller_t *ctrl);

/**
 * @brief  设置物理参数
 * @param  ctrl: LQR控制器指针
 * @param  h: 质心高度（m）
 * @param  L: 轴距（m）
 * @param  b: 拖曳距（m）
 */
void lqr_set_physical_params(LQR_Controller_t *ctrl, float h, float L, float b);

/**
 * @brief  设置增益表（从离线计算导入）
 * @param  ctrl: LQR控制器指针
 * @param  table: 增益表指针
 */
void lqr_set_gain_table(LQR_Controller_t *ctrl, const LQR_GainTable_t *table);

/**
 * @brief  根据速度更新增益（线性插值）
 * @param  ctrl: LQR控制器指针
 * @param  velocity: 当前速度（m/s）
 * @return 0=成功，-1=速度过低
 */
int8_t lqr_update_gain(LQR_Controller_t *ctrl, float velocity);

/**
 * @brief  执行LQR状态反馈计算
 * @param  ctrl: LQR控制器指针
 * @param  phi: 侧倾角（rad）
 * @param  phi_dot: 侧倾角速度（rad/s）
 * @return 转向角命令（rad）
 */
float lqr_compute(LQR_Controller_t *ctrl, float phi, float phi_dot);

/**
 * @brief  重置LQR控制器
 * @param  ctrl: LQR控制器指针
 */
void lqr_reset(LQR_Controller_t *ctrl);

/**
 * @brief  使能/禁用LQR控制器
 * @param  ctrl: LQR控制器指针
 * @param  enable: 1=使能，0=禁用
 */
void lqr_enable(LQR_Controller_t *ctrl, uint8_t enable);

/**
 * @brief  设置限幅参数
 * @param  ctrl: LQR控制器指针
 * @param  delta_max: 最大转向角（rad）
 * @param  delta_dot_max: 最大转向角速度（rad/s）
 * @param  v_min: 最小有效速度（m/s）
 */
void lqr_set_limits(LQR_Controller_t *ctrl, float delta_max, float delta_dot_max, float v_min);

/**
 * @brief  获取当前增益值（调试用）
 * @param  ctrl: LQR控制器指针
 * @param  k_out: 输出增益数组（长度3）
 */
void lqr_get_current_gain(LQR_Controller_t *ctrl, float *k_out);

#endif /* CODE_DRIVER_LQR_LQR_DRIVER_H_ */
