/*
 * 文件: config_flash.c
 * 功能: 比赛参数 Flash 持久化实现
 *
 * Flash 布局（逐飞库 DFlash）：
 *   DFlash 基地址: 0xAF000000，共 12 页，每页 8KB（逻辑容量 4096 字节可用）
 *   本模块使用第 11 页（最后一页），与程序区完全隔离
 *
 * 数据结构布局见 config_flash.h 中 config_flash_t。
 * 校验采用简单累加和（所有 uint32 字段除 magic 和 checksum 本身）。
 *
 * 使用逐飞库 flash_union_buffer（1024×uint32 = 4096 字节缓冲区）作为中转：
 *   写入：clear buffer → memcpy struct → erase page → write_from_buffer
 *   读取：read_to_buffer → memcpy struct → validate
 *
 * 作者: 闫锦
 * 日期: 2026-04-05
 */

#include "config_flash.h"
#include "motor_pid.h"        /* g_motor_kp/ki/kd/limit, target_motor_rpm, motor_pid_init() */
#include "servo_pid.h"        /* g_balance_kp/ki/kd/... , g_servo_control_mode, balance_pid */
#include "servo_app.h"        /* g_servo_left/mid/right_limit */
#include "nav_app.h"          /* nav_export/import_gps_config, g_nav_heading_gain */
#include "subject1_app.h"
#include "subject2_app.h"
#include "subject3_app.h"

#include <string.h>           /* memcpy */

/* ================================================================
 * 内部工具函数
 * ================================================================ */

/**
 * @brief  计算 config_flash_t 的校验和
 * @details 对结构体中除 magic（index 0）和 checksum（最后 index）之外的
 *          所有 uint32 字段做累加，用于检测 Flash 数据是否损坏。
 */
static uint32_t compute_checksum(const config_flash_t *cfg)
{
    const uint32_t *p = (const uint32_t *)(const void *)cfg;
    uint32_t n   = sizeof(config_flash_t) / sizeof(uint32_t);
    uint32_t sum = 0u;
    uint32_t i;

    /* 跳过首个 uint32（magic）和末尾 uint32（checksum 本身）*/
    for (i = 1u; i < n - 1u; i++)
    {
        sum += p[i];
    }
    return sum;
}


/* ================================================================
 * 公开 API 实现
 * ================================================================ */

/**
 * @brief  保存全部配置到 DFlash 第 CONFIG_FLASH_PAGE 页
 */
void config_flash_save(void)
{
    config_flash_t cfg;
    nav_waypoint_t wps[NAV_MAX_WAYPOINTS];
    double         ref_lat = 0.0, ref_lon = 0.0;
    uint8_t        has_ref = 0u, wp_count = 0u;
    uint8_t        i;

    /* ---- 填充魔数 ---- */
    cfg.magic = CONFIG_FLASH_MAGIC;

    /* ---- 电机 PID ---- */
    cfg.motor_kp           = (float)g_motor_kp;
    cfg.motor_ki           = (float)g_motor_ki;
    cfg.motor_kd           = (float)g_motor_kd;
    cfg.motor_output_limit = (float)g_motor_output_limit;
    cfg.motor_target_rpm   = (float)target_motor_rpm;

    /* ---- 舵机外环 PID ---- */
    cfg.balance_kp             = (float)g_balance_kp;
    cfg.balance_ki             = (float)g_balance_ki;
    cfg.balance_kd             = (float)g_balance_kd;
    cfg.balance_output_limit   = (float)g_balance_output_limit;
    cfg.balance_integral_limit = (float)g_balance_integral_limit;

    /* ---- 舵机内环 PID ---- */
    cfg.balance_inner_kp = (float)g_balance_inner_kp;
    cfg.balance_inner_ki = (float)g_balance_inner_ki;

    /* ---- 舵机机械限位 ---- */
    cfg.servo_left_limit  = g_servo_left_limit;
    cfg.servo_mid_duty    = g_servo_mid_duty;
    cfg.servo_right_limit = g_servo_right_limit;

    /* ---- 控制模式与导航增益 ---- */
    cfg.servo_control_mode = g_servo_control_mode;
    cfg._pad1[0] = 0u; cfg._pad1[1] = 0u; cfg._pad1[2] = 0u;
    cfg.nav_heading_gain   = (float)g_nav_heading_gain;

    cfg.s1_high_rpm          = (float)g_s1_high_rpm;
    cfg.s1_mid_rpm           = (float)g_s1_mid_rpm;
    cfg.s1_turn_rpm          = (float)g_s1_turn_rpm;
    cfg.s1_pre_brake_dist    = (float)g_s1_pre_brake_dist;
    cfg.s1_finish_brake_dist = (float)g_s1_finish_brake_dist;
    cfg.s1_resume_thresh     = (float)g_s1_resume_thresh;
    cfg.s1_accel_step        = (float)g_s1_accel_step;

    cfg.s2_rpm               = (float)g_s2_rpm;
    cfg.s2_turn_radius       = (float)g_s2_turn_radius;

    cfg.s3_go_rpm            = (float)g_s3_go_rpm;
    cfg.s3_mid_rpm           = (float)g_s3_mid_rpm;
    cfg.s3_turn_rpm          = (float)g_s3_turn_rpm;
    cfg.s3_pre_brake_dist    = (float)g_s3_pre_brake_dist;
    cfg.s3_finish_brake_dist = (float)g_s3_finish_brake_dist;
    cfg.s3_resume_thresh     = (float)g_s3_resume_thresh;
    cfg.s3_accel_step        = (float)g_s3_accel_step;

    /* ---- GPS 私有状态（通过 nav_app 接口导出）---- */
    nav_export_gps_config(&has_ref, &ref_lat, &ref_lon, &wp_count, wps);

    cfg.gps_has_ref = has_ref;
    cfg._pad2[0] = 0u; cfg._pad2[1] = 0u; cfg._pad2[2] = 0u;
    memcpy(cfg.ref_lat_raw, &ref_lat, sizeof(double));
    memcpy(cfg.ref_lon_raw, &ref_lon, sizeof(double));

    cfg.wp_count = wp_count;
    cfg._pad3[0] = 0u; cfg._pad3[1] = 0u; cfg._pad3[2] = 0u;

    for (i = 0u; i < wp_count && i < NAV_MAX_WAYPOINTS; i++)
    {
        memcpy(cfg.waypoints[i].lat_raw, &wps[i].lat, sizeof(double));
        memcpy(cfg.waypoints[i].lon_raw, &wps[i].lon, sizeof(double));
        cfg.waypoints[i].target_speed  = wps[i].target_speed;
        cfg.waypoints[i].accept_radius = wps[i].accept_radius;
    }
    /* 清零未使用的路点槽（防止 checksum 受脏数据影响）*/
    for (i = wp_count; i < NAV_MAX_WAYPOINTS; i++)
    {
        cfg.waypoints[i].lat_raw[0]    = 0u;
        cfg.waypoints[i].lat_raw[1]    = 0u;
        cfg.waypoints[i].lon_raw[0]    = 0u;
        cfg.waypoints[i].lon_raw[1]    = 0u;
        cfg.waypoints[i].target_speed  = 0.0f;
        cfg.waypoints[i].accept_radius = 0.0f;
    }

    /* ---- 计算校验和 ---- */
    cfg.checksum = compute_checksum(&cfg);

    /* ---- 写入 Flash ----
     * 使用逐飞库的 flash_union_buffer（4096 字节缓冲区）作为中转。
     * flash_buffer_clear() 先清零，然后 memcpy 填充，最后整页写入。 */
    flash_buffer_clear();
    memcpy(flash_union_buffer, &cfg, sizeof(config_flash_t));
    flash_erase_page(CONFIG_FLASH_SECTOR, CONFIG_FLASH_PAGE);
    flash_write_page_from_buffer(CONFIG_FLASH_SECTOR, CONFIG_FLASH_PAGE);

    printf("[FLASH] Saved: motor_kp=%.3f, bal_kp=%.3f, wp_count=%u, size=%u bytes\r\n",
           cfg.motor_kp, cfg.balance_kp, (unsigned)wp_count,
           (unsigned)sizeof(config_flash_t));
}


/**
 * @brief  从 Flash 加载配置，应用到各模块全局变量
 * @return 1 = 成功，0 = 无效（使用默认值）
 */
uint8_t config_flash_load(void)
{
    config_flash_t cfg;
    nav_waypoint_t wps[NAV_MAX_WAYPOINTS];
    double         ref_lat, ref_lon;
    uint8_t        i;

    /* ---- 从 Flash 读到库缓冲区，再 memcpy 到本地结构体 ---- */
    flash_read_page_to_buffer(CONFIG_FLASH_SECTOR, CONFIG_FLASH_PAGE);
    memcpy(&cfg, flash_union_buffer, sizeof(config_flash_t));

    /* ---- 魔数校验 ---- */
    if (cfg.magic != CONFIG_FLASH_MAGIC)
    {
        printf("[FLASH] No valid config (magic=0x%08X), using defaults\r\n",
               (unsigned)cfg.magic);
        return 0u;
    }

    /* ---- 数据完整性校验 ---- */
    if (cfg.checksum != compute_checksum(&cfg))
    {
        printf("[FLASH] Checksum mismatch, config corrupted, using defaults\r\n");
        return 0u;
    }

    /* ---- 应用电机 PID 参数到全局变量 ---- */
    g_motor_kp           = cfg.motor_kp;
    g_motor_ki           = cfg.motor_ki;
    g_motor_kd           = cfg.motor_kd;
    g_motor_output_limit = cfg.motor_output_limit;
    target_motor_rpm     = cfg.motor_target_rpm;
    /* 重新初始化电机 PID 结构体（motor_control 读 struct 而非 globals）*/
    motor_pid_init();

    /* ---- 应用舵机 PID 参数（servo_pid.c 每次调用直接读 volatile globals）---- */
    g_balance_kp             = cfg.balance_kp;
    g_balance_ki             = cfg.balance_ki;
    g_balance_kd             = cfg.balance_kd;
    g_balance_output_limit   = cfg.balance_output_limit;
    g_balance_integral_limit = cfg.balance_integral_limit;
    g_balance_inner_kp       = cfg.balance_inner_kp;
    g_balance_inner_ki       = cfg.balance_inner_ki;

    /* ---- 应用舵机机械限位 ---- */
    g_servo_left_limit  = cfg.servo_left_limit;
    g_servo_mid_duty    = cfg.servo_mid_duty;
    g_servo_right_limit = cfg.servo_right_limit;

    /* ---- 应用控制模式与导航增益 ---- */
    g_servo_control_mode = cfg.servo_control_mode;
    g_nav_heading_gain   = cfg.nav_heading_gain;

    g_s1_high_rpm          = cfg.s1_high_rpm;
    g_s1_mid_rpm           = cfg.s1_mid_rpm;
    g_s1_turn_rpm          = cfg.s1_turn_rpm;
    g_s1_pre_brake_dist    = cfg.s1_pre_brake_dist;
    g_s1_finish_brake_dist = cfg.s1_finish_brake_dist;
    g_s1_resume_thresh     = cfg.s1_resume_thresh;
    g_s1_accel_step        = cfg.s1_accel_step;

    g_s2_rpm               = cfg.s2_rpm;
    g_s2_turn_radius       = cfg.s2_turn_radius;

    g_s3_go_rpm            = cfg.s3_go_rpm;
    g_s3_mid_rpm           = cfg.s3_mid_rpm;
    g_s3_turn_rpm          = cfg.s3_turn_rpm;
    g_s3_pre_brake_dist    = cfg.s3_pre_brake_dist;
    g_s3_finish_brake_dist = cfg.s3_finish_brake_dist;
    g_s3_resume_thresh     = cfg.s3_resume_thresh;
    g_s3_accel_step        = cfg.s3_accel_step;

    /* ---- 恢复 GPS 勘线数据 ---- */
    memcpy(&ref_lat, cfg.ref_lat_raw, sizeof(double));
    memcpy(&ref_lon, cfg.ref_lon_raw, sizeof(double));

    for (i = 0u; i < cfg.wp_count && i < NAV_MAX_WAYPOINTS; i++)
    {
        memcpy(&wps[i].lat, cfg.waypoints[i].lat_raw, sizeof(double));
        memcpy(&wps[i].lon, cfg.waypoints[i].lon_raw, sizeof(double));
        wps[i].target_speed  = cfg.waypoints[i].target_speed;
        wps[i].accept_radius = cfg.waypoints[i].accept_radius;
    }
    nav_import_gps_config(cfg.gps_has_ref, ref_lat, ref_lon, cfg.wp_count, wps);

    printf("[FLASH] Loaded OK: motor_kp=%.3f, bal_kp=%.3f, has_ref=%u, wp=%u\r\n",
           cfg.motor_kp, cfg.balance_kp,
           (unsigned)cfg.gps_has_ref, (unsigned)cfg.wp_count);
    return 1u;
}


/**
 * @brief  擦除 Flash 配置页（下次上电恢复编译期默认值）
 */
void config_flash_erase(void)
{
    flash_erase_page(CONFIG_FLASH_SECTOR, CONFIG_FLASH_PAGE);
    printf("[FLASH] Config page erased, defaults will be used on next boot\r\n");
}
