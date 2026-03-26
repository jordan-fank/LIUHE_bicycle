/**
 * @file ips_driver.h
 * @brief IPS200 驱动层 - 封装逐飞库底层接口
 *
 * 本层只提供基础绘图原语，不含业务逻辑。
 * 统一使用 8x16 字体（每字符宽8像素、高16像素）。
 * 用户无需修改本文件。
 */

#ifndef IPS_DRIVER_H
#define IPS_DRIVER_H

#include "zf_common_headfile.h"

// 屏幕物理尺寸
#define IPS_W       240
#define IPS_H       320

// 8x16 字体的单字符尺寸
#define CHAR_W      8
#define CHAR_H      16

// 初始化显示屏（内部调用 ips200_init，清屏为白色）
void ips_driver_init(void);

// 全屏填充指定颜色
void ips_driver_clear(uint16 color);

// 在 (x, y) 处显示字符串，超出屏幕右边界自动截断
// fg: 字体颜色  bg: 背景颜色
void ips_driver_show_str(uint16 x, uint16 y, const char *str, uint16 fg, uint16 bg);

// 用指定颜色填充一个矩形区域（用于局部刷新前擦除旧内容）
void ips_driver_fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);

// 画水平线
void ips_driver_hline(uint16 x, uint16 y, uint16 len, uint16 color);

#endif /* IPS_DRIVER_H */
