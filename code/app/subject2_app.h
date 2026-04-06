/*
 * 文件: subject2_app.h
 * 功能: 第21届全国智能车大赛 · 科目2（低速八字绕桩）应用层接口
 *
 * 赛题：在半径 2m 的圆内完成锥桶八字绕桩，两锥桶间距 2m，单向绕行一周。
 * 导航：纯 IMU 惯导（GPS 精度 ~1m，无法用于 2m 半径圆），无需 GPS 勘线。
 * 控制：自动切换 LOW_SPEED 模式（纯 P+I），适合低速大舵角平衡。
 *
 * 八字轨迹分解（每圈 3 段 × 120°，共 8 段）：
 *
 *   段0: heading=0°,    dist=APPROACH → 从发车区直行到交叉点
 *   段1: heading=-120°, dist=ARC      → 绕 A 锥桶左转 120°
 *   段2: heading=-240°, dist=ARC      → 继续左转 120°
 *   段3: heading=-360°, dist=ARC      → 完成 A 整圈，回到交叉点
 *   段4: heading=-240°, dist=ARC      → 绕 B 锥桶右转 120°
 *   段5: heading=-120°, dist=ARC      → 继续右转 120°
 *   段6: heading=0°,    dist=ARC      → 完成 B 整圈，回到交叉点
 *   段7: heading=0°,    dist=RETURN   → 直行回发车区
 *
 * 操作：
 *   KEY2 双击切换到科目2 → KEY3 长按启动 → 自动完成八字 → 停车
 *   无需勘线（段序列由参数自动计算）
 *
 * 作者: 闫锦
 * 日期: 2026-04-06
 */

#ifndef CODE_APP_SUBJECT2_APP_H_
#define CODE_APP_SUBJECT2_APP_H_

#include "zf_common_headfile.h"

/* ================================================================
 * 赛场参数（赛前实测调整）
 * ================================================================ */

/* 低速转速（RPM）：越低越稳但越难维持平衡，建议 150~300
   200 RPM ≈ 0.67 m/s（轮径 64mm），八字全程约 12m，用时 ~18s */
#define SUBJ2_RPM               200.0f

/* 绕桩半径（m）：两锥桶间距 2m，圆半径 2m
   每个锥桶绕行半径 = 锥桶间距 / 2 = 1m，但留 0.2m 余量防出圈
   建议 0.7~0.9m，越小弧线越紧、航向变化越快 */
#define SUBJ2_TURN_RADIUS_M     0.8f

/* 从发车区到八字交叉点的直行距离（m）
   发车区宽 1m，交叉点在两锥桶中间，距发车区约 0.3~1.0m */
#define SUBJ2_APPROACH_DIST_M   0.5f

/* 八字结束后回发车区的直行距离（m） */
#define SUBJ2_RETURN_DIST_M     0.5f


/* ================================================================
 * 自动计算的弧长（勿手动修改，由参数自动推导）
 * ================================================================ */

/* 每 120° 弧段的弧长 = 2πR/3 */
#define SUBJ2_ARC_DIST_M  (2.0f * 3.14159f * SUBJ2_TURN_RADIUS_M / 3.0f)

/* 八字段数：approach(1) + circleA(3) + circleB(3) + return(1) = 8 */
#define SUBJ2_TOTAL_SEGMENTS    8u


/* ================================================================
 * 状态枚举（极简：IDLE → RUNNING → DONE）
 * ================================================================ */
typedef enum
{
    SUBJ2_STATE_IDLE    = 0,  /* 空闲，等待启动 */
    SUBJ2_STATE_RUNNING = 1,  /* 八字绕桩中（IMU导航 + LOW_SPEED 模式）*/
    SUBJ2_STATE_DONE    = 2,  /* 完成 */
} subject2_state_t;

extern subject2_state_t g_subject2_state;

/* 运行时可调参数 */
extern volatile float g_s2_rpm;
extern volatile float g_s2_turn_radius;


/* ================================================================
 * API
 * ================================================================ */

void subject2_app_init(void);   /* 初始化（cpu0_main 调用）*/
void subject2_task(void);       /* 调度任务 100ms */
void subject2_start(void);      /* KEY3 长按（科目2模式下）：启动 */
void subject2_stop(void);       /* KEY3 长按（运行中）：紧急停止 */


#endif /* CODE_APP_SUBJECT2_APP_H_ */
