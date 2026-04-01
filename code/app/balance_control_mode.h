/*
 * 文件: balance_control_mode.h
 * 功能: 平衡控制方案选择开关，统一切换 PID 或 LQR
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#ifndef CODE_APP_BALANCE_CONTROL_MODE_H_
#define CODE_APP_BALANCE_CONTROL_MODE_H_

/* ==================== 平衡控制方案选择 ====================
   只需要修改 BALANCE_CONTROL_MODE 这一行，就可以在 PID 和 LQR 之间切换。

   建议：
   1. 台架联调、无速度测试时，使用 PID
   2. 实车前进平衡测试时，使用 LQR
*/
#define BALANCE_CONTROL_MODE_PID   (0U)
#define BALANCE_CONTROL_MODE_LQR   (1U)

/* ==================== 一键切换开关 ====================
   当前默认使用 PID。
   如需切回原来的 PID，只把这一行改成 BALANCE_CONTROL_MODE_PID 即可。
   如需切到 LQR，只把这一行改成 BALANCE_CONTROL_MODE_LQR 即可。
*/
#define BALANCE_CONTROL_MODE       (BALANCE_CONTROL_MODE_PID)

#endif /* CODE_APP_BALANCE_CONTROL_MODE_H_ */
