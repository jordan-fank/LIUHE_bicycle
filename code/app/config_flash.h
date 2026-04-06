/*
 * 文件: config_flash.h
 * 功能: 比赛参数 Flash 持久化接口
 *
 * 存储内容（全部写入 DFlash 第 11 页）：
 *   - 电机 PID 参数（kp/ki/kd/limit/target_rpm）
 *   - 舵机外环 PID 参数（kp/ki/kd/output_limit/integral_limit）
 *   - 舵机内环 PID 参数（inner_kp/inner_ki，串级模式）
 *   - 舵机机械限位（left/mid/right duty）
 *   - 控制模式（SIMPLE_PD / CASCADE / LOW_SPEED）
 *   - 导航航向增益（g_nav_heading_gain）
 *   - 科目1/2/3 IPS 运行时可调参数
 *   - GPS 勘线数据（参考起点 + 路点数组）
 *
 * 触发保存：
 *   - KEY1 双击 → config_flash_save()（IPS / WiFi / 科目参数调参后手动保存）
 *   - subject1_survey() 步骤2完成后自动调用（勘线完毕立即保存 GPS 路点）
 *
 * 加载：
 *   - cpu0_main.c 中 nav_app_init() 之后调用 config_flash_load()
 *   - 有效则覆盖编译期默认值，无效则使用默认值（首次上电或 erase 后）
 *
 * 作者: 闫锦
 * 日期: 2026-04-05
 */

#ifndef CODE_APP_CONFIG_FLASH_H_
#define CODE_APP_CONFIG_FLASH_H_

#include "zf_common_headfile.h"
#include "nav_app.h"   /* nav_waypoint_t, NAV_MAX_WAYPOINTS */

/* ================================================================
 * Flash 存储配置
 * ================================================================ */

/* 使用 DFlash 第 11 页（共 0~11 页，使用最后一页与程序区隔离）*/
#define CONFIG_FLASH_SECTOR     0u
#define CONFIG_FLASH_PAGE       11u

/* 魔数：用于判断 Flash 是否已保存过有效配置 */
#define CONFIG_FLASH_MAGIC      0xBEEF5A5Au


/* ================================================================
 * 持久化数据结构
 * 注意：double 类型（GPS 坐标）通过 memcpy 以 uint32_t[2] 形式存储，
 *       避免结构体内 double 的对齐/endian 问题，flash 读写全程操作 uint32。
 * ================================================================ */
typedef struct
{
    uint32_t magic;                    /* 魔数，有效性标识 */

    /* ---- 电机 PID ---- */
    float motor_kp;
    float motor_ki;
    float motor_kd;
    float motor_output_limit;
    float motor_target_rpm;            /* target_motor_rpm 的保存值 */

    /* ---- 舵机外环 PID ---- */
    float balance_kp;
    float balance_ki;
    float balance_kd;
    float balance_output_limit;
    float balance_integral_limit;

    /* ---- 舵机内环 PID（CASCADE 模式）---- */
    float balance_inner_kp;
    float balance_inner_ki;

    /* ---- 舵机机械限位 ---- */
    uint32_t servo_left_limit;
    uint32_t servo_mid_duty;
    uint32_t servo_right_limit;

    /* ---- 控制模式与导航增益 ---- */
    uint8_t  servo_control_mode;
    uint8_t  _pad1[3];                 /* 对齐填充 */
    float    nav_heading_gain;

    /* ---- 科目1/2/3 运行时参数 ---- */
    float    s1_high_rpm;
    float    s1_mid_rpm;
    float    s1_turn_rpm;
    float    s1_pre_brake_dist;
    float    s1_finish_brake_dist;
    float    s1_resume_thresh;
    float    s1_accel_step;

    float    s2_rpm;
    float    s2_turn_radius;

    float    s3_go_rpm;
    float    s3_mid_rpm;
    float    s3_turn_rpm;
    float    s3_pre_brake_dist;
    float    s3_finish_brake_dist;
    float    s3_resume_thresh;
    float    s3_accel_step;

    /* ---- GPS 参考起点（double 拆成 2×uint32 存储）---- */
    uint8_t  gps_has_ref;
    uint8_t  _pad2[3];                 /* 对齐填充 */
    uint32_t ref_lat_raw[2];           /* memcpy(ref_lat_raw, &lat, 8) */
    uint32_t ref_lon_raw[2];           /* memcpy(ref_lon_raw, &lon, 8) */

    /* ---- GPS 路点数组 ---- */
    uint8_t  wp_count;
    uint8_t  _pad3[3];                 /* 对齐填充 */
    struct
    {
        uint32_t lat_raw[2];           /* double lat → 2×uint32 */
        uint32_t lon_raw[2];           /* double lon → 2×uint32 */
        float    target_speed;
        float    accept_radius;
    } waypoints[NAV_MAX_WAYPOINTS];    /* 最多 20 个路点，每个 24 字节 */

    uint32_t checksum;                 /* 所有字段（除magic和checksum）的uint32累加和 */

} config_flash_t;

/* 静态断言：确保结构体大小为 4 的整数倍（flash 按 uint32 读写）
   若编译报错说明结构体有隐式 padding，需检查字段顺序或补充 _pad */
typedef char _static_assert_config_size_[(sizeof(config_flash_t) % 4u == 0u) ? 1 : -1];


/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief  保存全部配置到 Flash（DFlash page 11）
 * @note   内部执行：擦除页 → 组装结构体 → 写入
 *         耗时约 10~50ms，请勿在 ISR 中调用
 */
void    config_flash_save  (void);

/**
 * @brief  从 Flash 加载配置并应用到各模块全局变量
 * @return 1 = 加载成功（magic+checksum 均匹配）
 *         0 = 无有效配置（首次上电或已擦除），使用编译期默认值
 * @note   须在 nav_app_init() 之后、scheduler_init() 之前调用
 */
uint8_t config_flash_load  (void);

/**
 * @brief  擦除 Flash 配置页（恢复到"无配置"状态，下次上电使用默认值）
 */
void    config_flash_erase (void);


#endif /* CODE_APP_CONFIG_FLASH_H_ */
