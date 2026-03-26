/**
 * @file ips_driver.c
 * @brief IPS200 驱动层实现
 *
 * 封装逐飞 ips200 库，提供简洁的绘图原语。
 * 统一使用 IPS200_8X16_FONT（每字符宽8px高16px）。
 * 用户无需修改本文件。
 */

#include "ips_driver.h"

// 逐飞库屏幕类型（SPI接口）
#define IPS200_TYPE     (IPS200_TYPE_SPI)

void ips_driver_init(void)
{
    ips200_init(IPS200_TYPE);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_full(RGB565_WHITE);
}

void ips_driver_clear(uint16 color)
{
    ips200_full(color);
}

void ips_driver_show_str(uint16 x, uint16 y, const char *str, uint16 fg, uint16 bg)
{
    if (str == NULL) return;
    ips200_set_color(fg, bg);
    ips200_set_font(IPS200_8X16_FONT);

    while (*str != '\0')
    {
        // 超出右边界停止，不换行
        if (x + CHAR_W > IPS_W) break;
        ips200_show_char(x, y, *str);
        x += CHAR_W;
        str++;
    }
}

void ips_driver_fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    if (x >= IPS_W || y >= IPS_H || w == 0 || h == 0) return;

    // fg=bg=fill_color，空格字符的所有像素都渲染为该颜色
    // 利用 ips200_show_char 内部的批量传输（1次set_region + 128像素），
    // 比逐像素 draw_line 快约 70 倍
    ips200_set_color(color, color);

    for (uint16 cy = y; cy < y + h; cy += CHAR_H)
        for (uint16 cx = x; cx < x + w; cx += CHAR_W)
            ips200_show_char(cx, cy, ' ');

    // 恢复默认颜色
    ips200_set_color(CLR_FG, CLR_BG);
}

void ips_driver_hline(uint16 x, uint16 y, uint16 len, uint16 color)
{
    if (x >= IPS_W || y >= IPS_H) return;
    if (x + len > IPS_W) len = IPS_W - x;
    ips200_draw_line(x, y, x + len - 1, y, color);
}
