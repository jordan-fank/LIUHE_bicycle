/*
 * 文件: wireless_debug_app.c
 * 功能: 无线调试应用层实现
 *       通过高速 WiFi SPI 模块 + 逐飞助手 PC 软件实现无线调参和波形显示。
 *       本文件是独立新增模块，不修改任何现有控制逻辑。
 *
 * 依赖库（均已在 LIUHE_bicycle1/libraries 中）：
 *   - libraries/zf_device/zf_device_wifi_spi.c/h
 *   - libraries/zf_components/seekfree_assistant.c/h
 *   - libraries/zf_components/seekfree_assistant_interface.c/h
 *
 * 作者: 闫锦
 * 日期: 2026-04-01
 */

#include "wireless_debug_app.h"
#include "motor_pid.h"
#include "servo_pid.h"
#include "motor_app.h"
#include "imu_app.h"

/* ===========================================================
 * 模块内部状态
 * =========================================================== */
static uint8_t g_wifi_connected = 0;   // 0=未连接，1=已连接并可用

/* ===========================================================
 * 逐飞助手参数通道映射
 * 与 PC 端通道号一一对应（库内部 channel 从 1 开始，数组下标从 0 开始）
 * =========================================================== */
typedef enum
{
    PARAM_CH_BALANCE_KP   = 0,   // CH1: 舵机平衡 Kp
    PARAM_CH_BALANCE_KI   = 1,   // CH2: 舵机平衡 Ki
    PARAM_CH_BALANCE_KD   = 2,   // CH3: 舵机平衡 Kd
    PARAM_CH_MOTOR_KP     = 3,   // CH4: 电机速度 Kp
    PARAM_CH_MOTOR_KI     = 4,   // CH5: 电机速度 Ki
    PARAM_CH_MOTOR_KD     = 5,   // CH6: 电机速度 Kd
    PARAM_CH_MOTOR_SPEED  = 6,   // CH7: 目标电机转速 (RPM)
    PARAM_CH_RESERVED     = 7,   // CH8: 预留，不处理
} param_channel_enum;

/* ===========================================================
 * 逐飞助手波形通道映射
 * =========================================================== */
typedef enum
{
    OSC_CH_ROLL_ANGLE     = 0,   // CH1: 补偿后横滚角（°）
    OSC_CH_SERVO_OUTPUT   = 1,   // CH2: 舵机 PID 输出值
    OSC_CH_MOTOR_RPM      = 2,   // CH3: 电机实际转速（RPM）
    OSC_CH_MOTOR_OUTPUT   = 3,   // CH4: 电机 PID 输出值
    OSC_CH_TARGET_RPM     = 4,   // CH5: 目标转速（RPM）
    OSC_CH_PITCH_ANGLE    = 5,   // CH6: 补偿后俯仰角（°）
    OSC_CH_COUNT          = 6,   // 实际使用通道数
} oscilloscope_channel_enum;

/* ===========================================================
 * 初始化函数
 * =========================================================== */
void wireless_debug_init(void)
{
    uint8_t ret;

    /* ---- Step 1: WiFi SPI 硬件初始化 + 连接路由器 ---- */
    /* wifi_spi_init 内部完成：SPI3 初始化、RST 复位、WiFi 连接。
       若连接失败返回非零值。 */
    ret = wifi_spi_init(WIRELESS_WIFI_SSID, WIRELESS_WIFI_PASSWORD);
    if(0U != ret)
    {
        /* 连接失败：仅打印提示，不影响主控制系统启动 */
        printf("[WIRELESS] WiFi connect failed (ret=%d), wireless debug disabled.\r\n", (int)ret);
        g_wifi_connected = 0;
        return;
    }

    /* ---- Step 2: 连接到 PC 端逐飞助手 TCP 服务器 ---- */
    /* 参数说明：transport_type="TCP", ip=PC IP, port=8086, local_port="0"(自动分配) */
    ret = wifi_spi_socket_connect("TCP",
                                   WIRELESS_TCP_SERVER_IP,
                                   WIRELESS_TCP_SERVER_PORT,
                                   "0");
    if(0U != ret)
    {
        printf("[WIRELESS] TCP connect failed (ret=%d). Please check PC IP and port.\r\n", (int)ret);
        g_wifi_connected = 0;
        return;
    }

    /* ---- Step 3: 绑定逐飞助手通信接口到 WiFi SPI ---- */
    /* SEEKFREE_ASSISTANT_WIFI_SPI 内部将 send/receive 回调绑定到
       wifi_spi_send_buffer / wifi_spi_read_buffer */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    g_wifi_connected = 1;
    printf("[WIRELESS] WiFi debug init OK. Module IP: %s\r\n", wifi_spi_ip_addr_port);
    printf("[WIRELESS] Param CH1~CH7: balance_kp/ki/kd, motor_kp/ki/kd, target_rpm\r\n");
    printf("[WIRELESS] Osc  CH1~CH6: roll_angle, servo_out, motor_rpm, motor_out, target_rpm, pitch_angle\r\n");
}

/* ===========================================================
 * 内部函数：参数接收与分发
 * 注意：volatile float 的单次写入在 TC264D 上是原子操作，
 *       中断中读取这些参数是安全的。
 * =========================================================== */
static void wireless_apply_params(void)
{
    uint8_t ch;
    float   val;

    /* 驱动逐飞助手内部 FIFO 解析，把收到的字节流转成参数 */
    seekfree_assistant_data_analysis();

    /* 遍历所有通道，检查是否有新数据 */
    for(ch = 0U; ch < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; ch++)
    {
        if(0U == seekfree_assistant_parameter_update_flag[ch])
        {
            continue;   /* 本通道无新数据，跳过 */
        }

        /* 清除更新标志（先清标志再使用值，防止重复处理） */
        seekfree_assistant_parameter_update_flag[ch] = 0U;
        val = seekfree_assistant_parameter[ch];

        switch((param_channel_enum)ch)
        {
            case PARAM_CH_BALANCE_KP:
                /* 直接写 volatile 变量，servo_pid.c 的 balance_control() 每次都同步该值 */
                g_balance_kp = val;
                printf("[WIRELESS] balance_kp = %.4f\r\n", val);
                break;

            case PARAM_CH_BALANCE_KI:
                g_balance_ki = val;
                printf("[WIRELESS] balance_ki = %.4f\r\n", val);
                break;

            case PARAM_CH_BALANCE_KD:
                g_balance_kd = val;
                printf("[WIRELESS] balance_kd = %.4f\r\n", val);
                break;

            case PARAM_CH_MOTOR_KP:
                /* 直接写 volatile 变量，motor_pid.c 的 motor_control() 下次执行时会使用 */
                g_motor_kp = val;
                /* 同步更新 PID 结构体，让变化立即生效（增量式PID依赖结构体参数） */
                motor_set_params(g_motor_kp, g_motor_ki, g_motor_kd);
                printf("[WIRELESS] motor_kp = %.4f\r\n", val);
                break;

            case PARAM_CH_MOTOR_KI:
                g_motor_ki = val;
                motor_set_params(g_motor_kp, g_motor_ki, g_motor_kd);
                printf("[WIRELESS] motor_ki = %.4f\r\n", val);
                break;

            case PARAM_CH_MOTOR_KD:
                g_motor_kd = val;
                motor_set_params(g_motor_kp, g_motor_ki, g_motor_kd);
                printf("[WIRELESS] motor_kd = %.4f\r\n", val);
                break;

            case PARAM_CH_MOTOR_SPEED:
                /* 通过已有 API 设置，内部会同步 target_motor_speed_m_s */
                motor_set_target_rpm(val);
                printf("[WIRELESS] target_rpm = %.1f\r\n", val);
                break;

            case PARAM_CH_RESERVED:
            default:
                /* 预留通道，忽略 */
                break;
        }
    }
}

/* ===========================================================
 * 内部函数：波形数据打包并发送
 * =========================================================== */
static void wireless_send_oscilloscope(void)
{
    seekfree_assistant_oscilloscope_struct osc;

    /* 填充 6 个波形通道的实时数据 */
    osc.data[OSC_CH_ROLL_ANGLE]   = roll_ctrl_angle;    /* 补偿后横滚角（°） */
    osc.data[OSC_CH_SERVO_OUTPUT] = balance_pid.out;    /* 舵机 PID 输出值（balance_pid 定义在 servo_pid.c） */
    osc.data[OSC_CH_MOTOR_RPM]    = motor_speed_rpm;    /* 电机实际转速（RPM） */
    osc.data[OSC_CH_MOTOR_OUTPUT] = g_motor_pid_output; /* 电机 PID 输出值（见 motor_pid.c）*/
    osc.data[OSC_CH_TARGET_RPM]   = target_motor_rpm;   /* 目标转速（RPM） */
    osc.data[OSC_CH_PITCH_ANGLE]  = pitch_ctrl_angle;   /* 补偿后俯仰角（°） */

    /* 告知库本次使用 6 个通道（内部按通道数计算包长，节省带宽） */
    osc.channel_num = (uint8_t)OSC_CH_COUNT;

    /* 发送到 PC 逐飞助手波形窗口 */
    seekfree_assistant_oscilloscope_send(&osc);
}

/* ===========================================================
 * 对外周期任务（scheduler.c 中以 50ms 周期调用）
 * =========================================================== */
void wireless_debug_task(void)
{
    if(0U == g_wifi_connected)
    {
        return;   /* WiFi 未就绪，直接返回，不影响其他任务 */
    }

    /* 1. 接收并应用 PC 下发的调参数据 */
    wireless_apply_params();

    /* 2. 发送实时波形数据到 PC */
    wireless_send_oscilloscope();
}
