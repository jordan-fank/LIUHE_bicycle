/*
 * 文件: subject1_app.h
 * 功能: 第21届全国智能车大赛 · 科目1（高速直线往返）应用层接口
 *
 * 导航策略：
 *   全程 GPS 导航（去程+回程），配合分段速度控制实现"快而稳"。
 *
 * 速度策略（5段）：
 *   GO       ─ 全速直行 HIGH_RPM
 *   GO_BRAKE ─ 距掉头点 PRE_BRAKE_DIST 内线性减速 MID→TURN
 *   UTURN    ─ 低速掉头 TURN_RPM（GPS 自然引导弧线）
 *   RETURN   ─ 出弯后每 100ms 加速 +ACCEL_STEP，达终点制动区降 MID
 *   DONE     ─ 停车
 *
 * 操作流程：
 *   [勘线] 车在发车区   → KEY4 长按（第1次）→ nav_set_ref_start()
 *   [勘线] 推车到掉头区 → KEY4 长按（第2次）→ nav_record_waypoint() + 自动 Flash 保存
 *   [重勘] 已有旧路线时 → KEY4 长按（第1次）→ 清旧路线并记录新的 ref_start
 *   [重勘] 推车到掉头区 → KEY4 长按（第2次）→ 记录新路点并覆盖保存
 *   [比赛] 车归发车区   → KEY3 长按 → 启动
 *   [急停] 运行中        → KEY3 长按 → 立即停车
 *
 * 作者: 闫锦
 * 日期: 2026-04-06（升级：分段速度控制）
 */

#ifndef CODE_APP_SUBJECT1_APP_H_
#define CODE_APP_SUBJECT1_APP_H_

#include "zf_common_headfile.h"

/* ================================================================
 * 赛场参数（赛前按实测调整，单位在注释中标明）
 * ================================================================ */

/* --- 速度参数（RPM）--- */
#define SUBJ1_HIGH_RPM            600.0f  /* 全速直行转速（RPM）*/
#define SUBJ1_MID_RPM             400.0f  /* 中速过渡转速（RPM）：减速进弯 / 出弯初段 / 终点制动 */
#define SUBJ1_TURN_RPM            280.0f  /* 掉头低速转速（RPM）：控制掉头半径≤1.5m */

/* --- 距离参数（m）--- */
/* 距掉头路点多远开始线性减速（m）
   初始推荐 6m；若仍过冲则增大，若减速过早则减小 */
#define SUBJ1_PRE_BRAKE_DIST_M    6.0f

/* 回程中距起点多远开始终点制动（m）
   初始推荐 4m；若停车过冲则增大 */
#define SUBJ1_FINISH_BRAKE_DIST_M 4.0f

/* --- 角度参数（°）--- */
/* 掉头完成判定：heading_error 小于此值时开始出弯加速
   初始推荐 25°；若出弯后翻车则增大（等更对准再加速）*/
#define SUBJ1_RESUME_THRESH_DEG   25.0f

/* --- 加速参数（RPM/100ms）--- */
/* 出弯后每 100ms 增加的转速步进
   初始推荐 50 RPM：约 1.2s 从 TURN 加速到 HIGH
   若出弯失稳则减小此值（加速更慢） */
#define SUBJ1_ACCEL_STEP_RPM      50.0f

/* --- 路点参数（m）--- */
/* GPS 路点到达判定半径：dist_to_wp < 此值视为到达
   去程和回程共用同一个值 */
#define SUBJ1_WP_RADIUS           2.5f


/* ================================================================
 * 状态枚举（5个活跃状态 + IDLE + DONE）
 * ================================================================ */
typedef enum
{
    SUBJ1_STATE_IDLE     = 0,  /* 空闲，等待 KEY3 启动 */
    SUBJ1_STATE_GO       = 1,  /* 高速直行（GPS 导航，HIGH_RPM）*/
    SUBJ1_STATE_GO_BRAKE = 2,  /* 接近掉头点线性减速（GPS 导航，MID→TURN）*/
    SUBJ1_STATE_UTURN    = 3,  /* 低速掉头（GPS 导航至起点，TURN_RPM）*/
    SUBJ1_STATE_RETURN   = 4,  /* 回程渐进加速（GPS 导航，TURN→HIGH，终点制动 MID）*/
    SUBJ1_STATE_DONE     = 5,  /* 完成，停车 */
} subject1_state_t;

/* 当前科目1状态（供 key_app / IPS 等外部模块读取）*/
extern subject1_state_t g_subject1_state;

/* 运行时可调参数（默认值由上面的宏给出，可通过 IPS / Flash 覆盖）*/
extern volatile float g_s1_high_rpm;
extern volatile float g_s1_mid_rpm;
extern volatile float g_s1_turn_rpm;
extern volatile float g_s1_pre_brake_dist;
extern volatile float g_s1_finish_brake_dist;
extern volatile float g_s1_resume_thresh;
extern volatile float g_s1_accel_step;

extern uint8_t s_survey_step;             //堪线路点数，科目1堪线==2即堪线完成



/* ================================================================
 * API
 * ================================================================ */

/* 初始化：清零内部状态，在 cpu0_main.c 的 nav_app_init()+config_flash_load() 之后调用 */
void subject1_app_init(void);

/* 调度任务：100ms 周期，注册到 scheduler.c */
void subject1_task(void);

/* KEY3 长按触发：
 *   IDLE/DONE 状态 → 启动科目1（需已完成勘线）
 *   GO/GO_BRAKE/UTURN/RETURN 状态 → 紧急停止 */
void subject1_start(void);
void subject1_stop(void);

/* KEY4 长按触发（勘线/重勘）：
 *   无历史路线时：
 *     第1次：nav_set_ref_start()，记录发车区 GPS 参考起点
 *     第2次：nav_record_waypoint() + 自动 Flash 保存
 *   已有历史路线时：
 *     第1次：清空旧路线并记录新的 ref_start
 *     第2次：记录新 waypoint 并覆盖保存 */
void subject1_survey(void);


#endif /* CODE_APP_SUBJECT1_APP_H_ */
