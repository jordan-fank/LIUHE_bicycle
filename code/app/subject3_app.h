/*
 * 文件: subject3_app.h
 * 功能: 第21届全国智能车大赛 · 科目3（颠簸路段往返）应用层接口
 *
 * 与科目1的区别：
 *   - 多路点GPS导航（坡道/草地/锥桶间隙各设一个路点引导）
 *   - 回程反向经过相同路点，避免撞障碍
 *   - 路点数据独立存储在 Flash page 10，与科目1（page 11）互不干扰
 *
 * 导航策略：
 *   去程 → 多路点GPS导航（WP0→WP1→...→WPN-1掉头点）
 *   掉头 → GPS引导弧形转弯（低速，bearing自然指向回程首路点）
 *   回程 → 反向多路点GPS导航（WPN-2→...→WP0→ref_start）
 *
 * 速度策略（与科目1结构对称）：
 *   GO       ─ 全速直行 SUBJ3_GO_RPM
 *   GO_BRAKE ─ 接近掉头点时线性减速 MID→TURN
 *   UTURN    ─ 低速掉头 SUBJ3_TURN_RPM
 *   RETURN   ─ 渐进加速，终点制动 MID
 *   DONE     ─ 停车
 *
 * 勘线操作（KEY2双击切换到科目3后）：
 *   KEY4 长按第1次 → 记录 ref_start（发车区）
 *   KEY4 长按第2..N次 → 依次记录路径路点（坡道/草地/锥桶间隙/掉头区）
 *   每次记录路点后自动保存到 Flash page 10
 *
 * 作者: 闫锦
 * 日期: 2026-04-06
 */

#ifndef CODE_APP_SUBJECT3_APP_H_
#define CODE_APP_SUBJECT3_APP_H_

#include "zf_common_headfile.h"
#include "nav_app.h"   /* nav_waypoint_t, NAV_MAX_WAYPOINTS */

/* ================================================================
 * 赛场参数（赛前按实测调整）
 * ================================================================ */

/* --- 速度参数（RPM）--- */
#define SUBJ3_GO_RPM              500.0f  /* 去程/回程常速（RPM），略低于科目1因有颠簸 */
#define SUBJ3_MID_RPM             350.0f  /* 中速过渡（进弯/出弯/终点制动）*/
#define SUBJ3_TURN_RPM            250.0f  /* 掉头低速（RPM）*/

/* --- 距离参数（m）--- */
#define SUBJ3_PRE_BRAKE_DIST_M    5.0f    /* 掉头路点前开始减速的距离 */
#define SUBJ3_FINISH_BRAKE_DIST_M 4.0f    /* 终点制动距离 */

/* --- 角度参数（°）--- */
#define SUBJ3_RESUME_THRESH_DEG   30.0f   /* 出弯加速阈值（略大于科目1，因颠簸更需稳定）*/

/* --- 加速参数（RPM/100ms）--- */
#define SUBJ3_ACCEL_STEP_RPM      40.0f   /* 出弯加速步进（略慢于科目1）*/

/* --- 路点参数（m）--- */
#define SUBJ3_WP_RADIUS           2.0f    /* 到达判定半径（略小于科目1以提高精度）*/

/* --- Flash 存储 --- */
#define SUBJ3_FLASH_SECTOR        0u
#define SUBJ3_FLASH_PAGE          10u     /* Flash page 10（与科目1 page 11 隔离）*/
#define SUBJ3_FLASH_MAGIC         0xCAFE3333u


/* ================================================================
 * 状态枚举（与科目1结构对称）
 * ================================================================ */
typedef enum
{
    SUBJ3_STATE_IDLE     = 0,  /* 空闲 */
    SUBJ3_STATE_GO       = 1,  /* 多路点GPS去程，全速 */
    SUBJ3_STATE_GO_BRAKE = 2,  /* 接近掉头点减速 */
    SUBJ3_STATE_UTURN    = 3,  /* 低速掉头 */
    SUBJ3_STATE_RETURN   = 4,  /* 反向多路点GPS回程，渐进加速 */
    SUBJ3_STATE_DONE     = 5,  /* 完成 */
} subject3_state_t;

extern subject3_state_t g_subject3_state;

/* 运行时可调参数 */
extern volatile float g_s3_go_rpm;
extern volatile float g_s3_mid_rpm;
extern volatile float g_s3_turn_rpm;
extern volatile float g_s3_pre_brake_dist;
extern volatile float g_s3_finish_brake_dist;
extern volatile float g_s3_resume_thresh;
extern volatile float g_s3_accel_step;


/* ================================================================
 * API
 * ================================================================ */

void subject3_app_init(void);   /* 初始化（cpu0_main调用，会从Flash加载路点）*/
void subject3_task(void);       /* 调度任务 100ms */
void subject3_start(void);      /* KEY3长按（科目3模式下）：启动 */
void subject3_stop(void);       /* KEY3长按（运行中）：紧急停止 */
void subject3_survey(void);     /* KEY4长按（科目3模式下）：勘线（多路点）*/


#endif /* CODE_APP_SUBJECT3_APP_H_ */
