/**
 * 
 * 添加页面操作：
 * 0.需要显示的数据变量
 * 1.注册页面                  ->  ips_app——config.h
 * 2.extern外部需要显示的变量   ->  ips_app——config.h
 * 3.注册参数表                ->  ips_app——config.h
 * 4.注册参数列表的数组         ->  ips_app.c
 * 5.注册参数列表的长度         ->  ips_app.c
 * 6.注册标题                  ->  ips_app.c
 */


#include "ips_app.h"
#include "ips_driver.h"




//==============================================================================
// 全局变量定义   第0步找到需要显示的数据变量
//==============================================================================

// 电机参数
float g_motor_kp = 15.0f;
float g_motor_ki = 0.05f;
float g_motor_kd = 2.0f;


// 电池 / 状态（由外部赋值）
float g_bat_voltage  = 12.6f;
float g_vehicle_speed = 0.0f;


//传感器
float speed  = 25.6f;
int32_t temp  = 12;
float hum = 21.0f;






//==============================================================================
// 内部状态
//==============================================================================

static page_e g_page     = PAGE_IMU;      //当前选中的页面--电机界面
static uint8  g_sel      = 0;               // 当前选中行索引--第一行






//==============================================================================
// 页面参数表（静态，只初始化一次）     --第四步
//==============================================================================

static param_t s_motor_params[] = { MOTOR_PARAM_LIST };
static param_t s_servo_params[] = { SERVO_PARAM_LIST };
static param_t s_battery_params[]= { BATTERY_PARAM_LIST };
static param_t s_imu_params[]= { IMU_PARAM_LIST };        //类别注册新的参数列表数组
static param_t s_test_params[]= { TEST_PARAM_LIST };        //类别注册新的参数列表数组





#define ARRAY_LEN(arr)  (sizeof(arr) / sizeof((arr)[0]))




//==============================================================================
// 返回当前页面的参数数组和长度     --第五步
//==============================================================================
static param_t *get_params(uint8 *count)
{
    switch (g_page)
    {
        case PAGE_MOTOR:
            *count = ARRAY_LEN(s_motor_params);
            return s_motor_params;


        case PAGE_SERVO:
            *count = ARRAY_LEN(s_servo_params);
            return s_servo_params;


        case PAGE_BATTERY:
            *count = ARRAY_LEN(s_battery_params);
            return s_battery_params;


        case PAGE_IMU:
            *count = ARRAY_LEN(s_imu_params);
            return s_imu_params;


        case PAGE_TEST:
            *count = ARRAY_LEN(s_test_params);
            return s_test_params;


        default:
            *count = 0;
            return NULL;
    }
}


//==============================================================================
// 返回当前页面标题     --第六步
//==============================================================================

static const char *get_title(void)
{
    switch (g_page)
    {
        case PAGE_MOTOR:   return "Motor PID";
        case PAGE_SERVO:   return "Servo PID";
        case PAGE_BATTERY: return "Battery";
        case PAGE_IMU: return "IMU";
        case PAGE_TEST: return "TEST";
        default:           return "---";
    }
}










/*---------------------------------以下代码是辅助函数，无需修改---------------------------------------------*/


//==============================================================================
// 绘图辅助（局部刷新，不清全屏）
//==============================================================================

// 标题居中显示，下方画分隔线
static void draw_title(void)
{
    const char *title = get_title();
    uint8 len = 0;
    const char *p = title;
    while (*p++) len++;
    uint16 tx = (SCREEN_W - (uint16)len * 8) / 2;
    if (tx > SCREEN_W) tx = 0;  // 防止上溢

    // 清标题区域
    ips_driver_fill_rect(0, 0, SCREEN_W, TITLE_H, CLR_BG);
    ips_driver_show_str(tx, 0, title, CLR_FG, CLR_BG);
    // 分隔线
    ips_driver_hline(0, TITLE_H, SCREEN_W, CLR_FG);
}

// 绘制单行参数
//   row_idx : 参数在数组中的下标
//   p       : 指向该参数的 param_t
//   selected: 是否为当前选中行
static void draw_row(uint8 row_idx, const param_t *p, uint8 selected)
{
    uint16 y = (uint16)ROW_START_Y + (uint16)row_idx * ROW_H;

    // 用背景色擦除整行
    ips_driver_fill_rect(0, y, SCREEN_W, ROW_H, CLR_BG);

    // 选中指示符 "->" 或空格
    if (selected)
        ips_driver_show_str(ROW_LEFT, y + 2, "->", CLR_FG, CLR_BG);
    else
        ips_driver_show_str(ROW_LEFT, y + 2, "  ", CLR_FG, CLR_BG);

    // 标签（"->" 之后空一格）
    ips_driver_show_str(ROW_LEFT + INDICATOR_W + 2, y + 2, p->label, CLR_FG, CLR_BG);

    // 数值（右对齐，留右边距）
    char buf[16];
    if (p->is_int)
    {
        sprintf(buf, "%d", *(int32_t*)(p->value));
    }
    else
    {
        float v = *(float*)(p->value);
        if (p->dec == 1)
            sprintf(buf, "%.1f", (double)v);
        else
            sprintf(buf, "%.2f", (double)v);
    }

    // 计算右对齐起始x
    uint8 vlen = 0;
    const char *bp = buf;
    while (*bp++) vlen++;
    uint16 vx = SCREEN_W - ROW_RIGHT - (uint16)vlen * 8;
    if (vx > SCREEN_W) vx = 0;

    ips_driver_show_str(vx, y + 2, buf, CLR_FG, CLR_BG);
}

// 只刷新行右侧的数值区域，不碰指示符和标签，用于只读行的静默更新（无闪烁）
static void draw_value_only(uint8 row_idx, const param_t *p)
{
    uint16 y = (uint16)ROW_START_Y + (uint16)row_idx * ROW_H;

    // 格式化新数值
    char buf[16];
    if (p->is_int)
        sprintf(buf, "%d", *(int32_t*)(p->value));
    else if (p->dec == 1)
        sprintf(buf, "%.1f", (double)*(float*)(p->value));
    else
        sprintf(buf, "%.2f", (double)*(float*)(p->value));

    // 计算右对齐起始x（与 draw_row 保持一致）
    uint8 vlen = 0;
    const char *bp = buf;
    while (*bp++) vlen++;
    uint16 vx = SCREEN_W - ROW_RIGHT - (uint16)vlen * 8;
    if (vx > SCREEN_W) vx = 0;

    // 只擦除数值区域（右侧固定宽度），再写入新值
    // 数值最多7字符（如"-99.99"）= 56px，留足擦除宽度
    uint16 erase_x = SCREEN_W - ROW_RIGHT - 7 * 8;
    ips_driver_fill_rect(erase_x, y, SCREEN_W - ROW_RIGHT - erase_x, ROW_H, CLR_BG);
    ips_driver_show_str(vx, y + 2, buf, CLR_FG, CLR_BG);
}

// 重绘当前页面所有内容（仅切页时调用）
static void draw_full_page(void)
{
    ips_driver_clear(CLR_BG);
    draw_title();

    uint8 count = 0;
    param_t *params = get_params(&count);
    for (uint8 i = 0; i < count; i++)
        draw_row(i, &params[i], (i == g_sel));
}











//==============================================================================
// 公开 API
//==============================================================================

void ips_app_init(void)
{
    ips_driver_init();
    g_page = PAGE_IMU;
    g_sel  = 0;
    draw_full_page();
}


static float s_rdonly_cache[MAX_RDONLY_ROWS];

// 定时任务：只有只读行的数值发生变化时才重绘，避免闪烁
void ips_app_task(void)
{
    uint8 count = 0;
    param_t *params = get_params(&count);
    if (params == NULL) return;

    for (uint8 i = 0; i < count && i < MAX_RDONLY_ROWS; i++)
    {
        if (!params[i].rdonly) continue;

        // 读取当前值（统一转成 float 做缓存比较）
        float cur = params[i].is_int ? (float)*(int32_t*)(params[i].value)
                                     : *(float*)(params[i].value);
        if (cur != s_rdonly_cache[i])
        {
            s_rdonly_cache[i] = cur;
            draw_value_only(i, &params[i]);  // 只刷新数值，不刷整行，无闪烁
        }
    }
}

// 切换到下一页
void ips_app_next_page(void)
{
    g_page = (page_e)((g_page + 1) % PAGE_COUNT);
    g_sel  = 0;
    for (uint8 i = 0; i < MAX_RDONLY_ROWS; i++) s_rdonly_cache[i] = 0.0f;
    draw_full_page();
}

// 切换到上一页
void ips_app_prev_page(void)
{
    g_page = (page_e)((g_page + PAGE_COUNT - 1) % PAGE_COUNT);
    g_sel  = 0;
    for (uint8 i = 0; i < MAX_RDONLY_ROWS; i++) s_rdonly_cache[i] = 0.0f;
    draw_full_page();
}

// 向上选择（循环），立即重绘两行
void ips_app_sel_up(void)
{
    uint8 count = 0;
    param_t *params = get_params(&count);
    if (count == 0) return;

    uint8 old_sel = g_sel;
    g_sel = (g_sel + count - 1) % count;
    draw_row(old_sel, &params[old_sel], 0);
    draw_row(g_sel,   &params[g_sel],   1);
}

// 向下选择（循环），立即重绘两行
void ips_app_sel_down(void)
{
    uint8 count = 0;
    param_t *params = get_params(&count);
    if (count == 0) return;

    uint8 old_sel = g_sel;
    g_sel = (g_sel + 1) % count;
    draw_row(old_sel, &params[old_sel], 0);
    draw_row(g_sel,   &params[g_sel],   1);
}

// 增减参数值，立即重绘当前行
void ips_app_adjust(int8 direction)
{
    uint8 count = 0;
    param_t *params = get_params(&count);
    if (params == NULL || g_sel >= count) return;

    param_t *p = &params[g_sel];
    if (p->rdonly) return;

    if (p->is_int)
    {
        int32_t v = *(int32_t*)(p->value) + (int32_t)((float)direction * p->step);
        if (v < (int32_t)p->min) v = (int32_t)p->min;
        if (v > (int32_t)p->max) v = (int32_t)p->max;
        *(int32_t*)(p->value) = v;
    }
    else
    {
        float v = *(float*)(p->value) + (float)direction * p->step;
        if (v < p->min) v = p->min;
        if (v > p->max) v = p->max;
        *(float*)(p->value) = v;
    }

    draw_row(g_sel, p, 1);
}
