#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LQR增益计算器 - GUI版本（3状态完整模型）
用于转向平衡车（摩托车结构）

物理模型：点质量模型
状态：x = [phi, phi_dot, delta]^T  (3维)
输入：u = delta_dot

状态方程：
    A = [0    1      0     ]     B = [   0   ]
        [g/h  0  -v²/(hw)  ]         [-bv/hw]
        [0    0      0     ]         [   1  ]

Author: 无心之举
Date: 2026
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from scipy import linalg

# ==================== 统一配置常量 ====================
# 说明：
# 1. 这里集中放置“增益表计算相关”的默认参数，作用类似 C 里的宏定义
# 2. 后续如果要调整默认模型、默认权重或默认速度范围，优先改这里
# 3. GUI 初始值、参数校验和 C 代码输出都统一从这里取值，避免多处散落的魔法数字

# ==================== 一、必须优先实测的参数 ====================
# 这一组参数描述的是“你的车本身长什么样”，属于模型物理量。
# 原则：
# 1. 它们不是靠主观感觉调出来的，应该尽量来自实车测量或 CAD
# 2. 如果这些量明显填错，后面 Q/R 调得再辛苦，LQR 也很难真正匹配你的车
# 3. 实测完这些量之后，再进入 Q/R 调参阶段

# 模型默认物理参数
# DEFAULT_COM_HEIGHT_M:
#   质心高度 h，单位 m。
#   含义：车辆整体质心到地面的垂直高度。
#   作用：h 越大，倒立摆特性越明显，系统天然越“不稳”，LQR 往往会给出更激进的控制增益。
#   来源：应以实车测量值为准，这里的 0.12 只是默认初值，不是通用标准答案。
DEFAULT_COM_HEIGHT_M = 0.12

# DEFAULT_WHEELBASE_M:
#   轴距 w/L，单位 m。
#   含义：前后轮接地点之间的水平距离。
#   作用：影响转弯半径和离心力建立能力；在模型中出现在 v^2/(h*w) 和 b*v/(h*w) 项里。
#   注意：这个脚本沿用原始工具命名，用 w；你的工程代码里通常写 L，它们表示同一个物理量。
DEFAULT_WHEELBASE_M = 0.21

# DEFAULT_TRAIL_M:
#   拖曳距 b，单位 m。
#   含义：后轮接地点到质心投影点的水平距离。
#   作用：决定 B 矩阵中的惯性耦合项大小，也就是“转向角速度对侧倾角加速度的即时影响”强弱。
#   注意：这个量通常不容易凭经验猜准，建议按几何结构或 CAD 实测。
DEFAULT_TRAIL_M = 0.02

# DEFAULT_GRAVITY_M_S2:
#   重力加速度 g，单位 m/s^2。
#   含义：地球重力常数，参与 (g/h) 项。
#   作用：通常固定使用 9.8 即可，除非你明确要做仿真条件切换，否则一般不需要改。
DEFAULT_GRAVITY_M_S2 = 9.8

# ==================== 二、需要通过调参逐步优化的参数 ====================
# 这一组参数不是“车辆固有尺寸”，而是“你希望控制器偏向什么风格”的权重配置。
# 原则：
# 1. 它们主要决定控制器更激进还是更平滑、更强调扶正还是更强调回正
# 2. 这类参数通常需要先有一个可用初值，再结合实车现象不断微调
# 3. 如果物理参数还没测准，不建议过早死磕这组参数

# 默认 LQR 权重
# DEFAULT_Q_PHI:
#   Q[0]，侧倾角 phi 的惩罚权重。
#   含义：你有多“讨厌车身倾斜偏离直立”。
#   调大效果：更优先压制侧倾角误差，车更想立起来，响应会更激进。
#   调太大风险：舵机动作可能过猛，噪声敏感度上升，低速段容易显得“冲”。
DEFAULT_Q_PHI = 100.0

# DEFAULT_Q_PHI_DOT:
#   Q[1]，侧倾角速度 phi_dot 的惩罚权重。
#   含义：你有多“讨厌车身还在继续倒向一边”。
#   调大效果：增加阻尼，减轻振荡和来回摆动。
#   调太大风险：控制会变“闷”，响应速度下降，可能出现反应迟钝。
DEFAULT_Q_PHI_DOT = 15.0

# DEFAULT_Q_DELTA:
#   Q[2]，转向角 delta 的惩罚权重。
#   含义：你有多“讨厌前轮长期偏离中位”。
#   调大效果：更强调回正能力，直线保持更强，蛇行通常会减轻。
#   调太大风险：会压制转向动作，导致纠偏不够或转向建立过慢。
DEFAULT_Q_DELTA = 30.0

# DEFAULT_R_INPUT:
#   R[0,0]，控制输入 u=delta_dot 的惩罚权重。
#   含义：你有多“讨厌舵机动作太快、太猛”。
#   调大效果：控制更平滑，舵机抖动更少。
#   调太大风险：整体响应变慢，车快倒前可能来不及给足转向。
DEFAULT_R_INPUT = 2.0

# ==================== 三、与增益调度范围相关的参数 ====================
# 这一组参数决定“你想在哪个速度区间生成增益表”以及“速度点划分多细”。
# 原则：
# 1. 它们既不是纯物理量，也不是单纯控制权重，而是离线查表策略
# 2. 应结合你的真实运行速度区间来设置
# 3. 如果实车主要跑低速，就不要把重点放在特别高的速度段；反之亦然

# 默认速度范围
# DEFAULT_VELOCITY_MIN_M_S:
#   增益表起始速度，单位 m/s。
#   含义：从这个速度点开始计算第一组 LQR 增益。
#   作用：决定查表的最低速度边界；它不是运行时 v_min 保护阈值本身，但两者通常应保持逻辑一致。
#   注意：过低速度下自行车模型本身就不太成立，盲目往更低速扩展通常意义不大。
DEFAULT_VELOCITY_MIN_M_S = 0.5

# DEFAULT_VELOCITY_MAX_M_S:
#   增益表结束速度，单位 m/s。
#   含义：计算最后一组增益所对应的最高速度点。
#   作用：决定查表覆盖的高速边界。
#   注意：如果你的实车运行速度会超过这个值，运行时虽然可以“钳到最高点增益”，但严格来说最好重新扩展表。
DEFAULT_VELOCITY_MAX_M_S = 7.0

# DEFAULT_VELOCITY_POINT_COUNT:
#   增益表速度点个数。
#   含义：在 v_min 到 v_max 之间线性均分出多少个速度点来离线求解 LQR。
#   作用：点越多，插值越细；点越少，表更短，但速度变化时的增益近似更粗。
#   工程取舍：15 点通常已经够用，除非你需要覆盖特别宽的速度范围或更细的调度精度。
DEFAULT_VELOCITY_POINT_COUNT = 15

# ==================== 四、工具自身的约束参数 ====================
# 这一组参数只是为了保护当前 GUI 工具的使用体验，不属于 LQR 数学模型本身。

# 参数范围限制
# MIN_VELOCITY_POINT_COUNT:
#   速度点数下限。
#   含义：至少要有 2 个点，才能形成一个有意义的速度区间。
MIN_VELOCITY_POINT_COUNT = 2

# MAX_VELOCITY_POINT_COUNT:
#   速度点数上限。
#   含义：这是当前 GUI 工具人为设置的保护上限，避免一次生成太密的表导致界面和输出代码过长。
#   注意：这不是 LQR 理论限制，只是当前工具的使用约束。
MAX_VELOCITY_POINT_COUNT = 20

# ==================== 五、代码生成输出参数 ====================
# 这一组参数只影响“生成出来的 C 代码名字长什么样”，不影响数值计算结果。

# 生成 C 代码时使用的符号名
# DEFAULT_C_GAIN_TABLE_NAME:
#   输出到 C 代码里的增益表数组名。
#   作用：生成代码时写成
#       static const LQR_GainEntry_t default_gain_entries[] = {...};
#   用途：如果你工程里想换成别的名字，可以只改这里，不用改字符串拼接逻辑。
DEFAULT_C_GAIN_TABLE_NAME = "default_gain_entries"

# DEFAULT_C_GAIN_COUNT_MACRO:
#   输出到 C 代码里的“增益表长度宏名”。
#   作用：生成代码时写成
#       #define DEFAULT_GAIN_COUNT  15
#   用途：如果你的工程命名规范不同，也只需要改这里。
DEFAULT_C_GAIN_COUNT_MACRO = "DEFAULT_GAIN_COUNT"

# ==================== 六、GUI 显示参数 ====================
# 这一组参数只影响图形界面显示，不参与任何 LQR 数值计算。

# GUI 默认配置
# GUI_WINDOW_WIDTH / GUI_WINDOW_HEIGHT:
#   图形界面默认窗口尺寸，单位 px。
#   作用：只影响工具界面显示，不影响任何 LQR 计算结果。
GUI_WINDOW_WIDTH = 850
GUI_WINDOW_HEIGHT = 700

# GUI_STATUS_READY_TEXT:
#   GUI 状态栏初始提示文本。
#   作用：只影响界面提示，不影响任何计算结果。
GUI_STATUS_READY_TEXT = "就绪 - 3状态LQR完整模型"

# ==================== 七、内部语义索引参数 ====================
# 这一组参数用于提高代码可读性，避免裸数字索引。

# 状态和增益索引
# STATE_IDX_PHI / STATE_IDX_PHI_DOT / STATE_IDX_DELTA:
#   状态索引常量。
#   作用：明确 K 向量中每一项的语义，避免直接写 K[0]/K[1]/K[2] 这种可读性差的魔法数字。
#   对应关系：
#       K[STATE_IDX_PHI]      -> K_phi
#       K[STATE_IDX_PHI_DOT]  -> K_phi_dot
#       K[STATE_IDX_DELTA]    -> K_delta
STATE_IDX_PHI = 0
STATE_IDX_PHI_DOT = 1
STATE_IDX_DELTA = 2


def build_q_matrix(q_phi, q_phi_dot, q_delta):
    """根据 3 个状态权重构建 Q 矩阵

    参数说明:
        q_phi:
            侧倾角 phi 的权重，越大越优先“扶正”车身。
        q_phi_dot:
            侧倾角速度 phi_dot 的权重，越大阻尼越强。
        q_delta:
            转向角 delta 的权重，越大越强调回正和抑制蛇行。
    """
    return np.diag([q_phi, q_phi_dot, q_delta])


def build_r_matrix(r_input):
    """根据控制输入权重构建 R 矩阵

    参数说明:
        r_input:
            控制输入 u=delta_dot 的权重。
            越大表示越不希望舵机动作过猛，控制更平滑；越小则动作更激进。
    """
    return np.array([[r_input]])


class LQRCalculator:
    """LQR增益计算核心（3维状态空间，完整模型）"""

    def __init__(self, h, w, b, g=DEFAULT_GRAVITY_M_S2):
        """
        初始化LQR计算器

        Args:
            h: 质心高度 (m)
            w: 轴距，前后轮接地点距离 (m)
            b: 后轮接地点到质心投影的距离 (m)
            g: 重力加速度 (m/s^2)
        """
        self.h = h
        self.w = w
        self.b = b
        self.g = g

    def get_state_space(self, v):
        """构建3阶状态空间矩阵

        状态: x = [phi, phi_dot, delta]^T
        输入: u = delta_dot

        状态方程:
            phi_dot = phi_dot
            phi_ddot = (g/h)*phi - (v²/hw)*delta - (bv/hw)*delta_dot
            delta_dot = delta_dot = u
        """
        h, w, b, g = self.h, self.w, self.b, self.g

        # 3x3 A矩阵
        A = np.array([
            [0,   1,            0         ],   # phi_dot = phi_dot
            [g/h, 0,   -v**2/(h*w)        ],   # phi_ddot = (g/h)*phi - (v^2/hw)*delta
            [0,   0,            0         ]    # delta_dot = u (由B控制)
        ])

        # 3x1 B矩阵（完整模型，包含-bv/hw项）
        B = np.array([
            [0],
            [-b*v/(h*w)],   # 转向角速度对倾角加速度的影响
            [1]
        ])

        return A, B

    def compute_gain(self, v, Q, R):
        """计算单个速度点的LQR增益

        返回: K = [K_phi, K_phi_dot, K_delta]
        """
        A, B = self.get_state_space(v)

        try:
            P = linalg.solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ P
            return K.flatten()
        except Exception as e:
            print(f"速度 {v:.2f} m/s 计算失败: {e}")
            return None

    def generate_table(self, v_min, v_max, n_points, Q, R):
        """生成LQR增益表"""
        velocities = np.linspace(v_min, v_max, n_points)
        table = []
        for v in velocities:
            K = self.compute_gain(v, Q, R)
            if K is not None:
                table.append((v, K))
        return table


class LQRCalculatorGUI:
    """LQR计算器GUI界面"""

    def __init__(self, root):
        self.root = root
        self.root.title("LQR增益计算器 - 转向平衡车")
        self.root.geometry(f"{GUI_WINDOW_WIDTH}x{GUI_WINDOW_HEIGHT}")
        self.root.resizable(True, True)

        self.create_widgets()

    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ========== 物理参数区 ==========
        param_frame = ttk.LabelFrame(main_frame, text="物理参数", padding="10")
        param_frame.pack(fill=tk.X, pady=5)

        # 第一行
        ttk.Label(param_frame, text="质心高度 h (m):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.h_var = tk.StringVar(value=str(DEFAULT_COM_HEIGHT_M))
        ttk.Entry(param_frame, textvariable=self.h_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(param_frame, text="轴距 w (m):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.w_var = tk.StringVar(value=str(DEFAULT_WHEELBASE_M))
        ttk.Entry(param_frame, textvariable=self.w_var, width=10).grid(row=0, column=3, padx=5, pady=2)

        ttk.Label(param_frame, text="拖曳距 b (m):").grid(row=0, column=4, sticky=tk.W, padx=5, pady=2)
        self.b_var = tk.StringVar(value=str(DEFAULT_TRAIL_M))
        ttk.Entry(param_frame, textvariable=self.b_var, width=10).grid(row=0, column=5, padx=5, pady=2)

        # 第二行
        ttk.Label(param_frame, text="重力加速度 g (m/s²):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.g_var = tk.StringVar(value=str(DEFAULT_GRAVITY_M_S2))
        ttk.Entry(param_frame, textvariable=self.g_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        # ========== LQR权重区 ==========
        weight_frame = ttk.LabelFrame(main_frame, text="LQR权重", padding="10")
        weight_frame.pack(fill=tk.X, pady=5)

        ttk.Label(weight_frame, text="Q[0] phi权重:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.q0_var = tk.StringVar(value=str(DEFAULT_Q_PHI))
        ttk.Entry(weight_frame, textvariable=self.q0_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(weight_frame, text="Q[1] phi_dot权重:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.q1_var = tk.StringVar(value=str(DEFAULT_Q_PHI_DOT))
        ttk.Entry(weight_frame, textvariable=self.q1_var, width=10).grid(row=0, column=3, padx=5, pady=2)

        ttk.Label(weight_frame, text="Q[2] delta权重:").grid(row=0, column=4, sticky=tk.W, padx=5, pady=2)
        self.q2_var = tk.StringVar(value=str(DEFAULT_Q_DELTA))
        ttk.Entry(weight_frame, textvariable=self.q2_var, width=10).grid(row=0, column=5, padx=5, pady=2)

        ttk.Label(weight_frame, text="R 输入权重:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.r_var = tk.StringVar(value=str(DEFAULT_R_INPUT))
        ttk.Entry(weight_frame, textvariable=self.r_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        # ========== 速度范围区 ==========
        vel_frame = ttk.LabelFrame(main_frame, text="速度范围", padding="10")
        vel_frame.pack(fill=tk.X, pady=5)

        ttk.Label(vel_frame, text="最小速度 (m/s):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.v_min_var = tk.StringVar(value=str(DEFAULT_VELOCITY_MIN_M_S))
        ttk.Entry(vel_frame, textvariable=self.v_min_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(vel_frame, text="最大速度 (m/s):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.v_max_var = tk.StringVar(value=str(DEFAULT_VELOCITY_MAX_M_S))
        ttk.Entry(vel_frame, textvariable=self.v_max_var, width=10).grid(row=0, column=3, padx=5, pady=2)

        ttk.Label(vel_frame, text="速度点数:").grid(row=0, column=4, sticky=tk.W, padx=5, pady=2)
        self.n_points_var = tk.StringVar(value=str(DEFAULT_VELOCITY_POINT_COUNT))
        ttk.Entry(vel_frame, textvariable=self.n_points_var, width=10).grid(row=0, column=5, padx=5, pady=2)

        # ========== 按钮区 ==========
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        ttk.Button(btn_frame, text="计算", command=self.calculate, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="复制代码", command=self.copy_code, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="清空", command=self.clear_output, width=15).pack(side=tk.LEFT, padx=5)

        # ========== 输出区 ==========
        output_frame = ttk.LabelFrame(main_frame, text="生成的C代码（可直接复制到 lqr_driver.c）", padding="10")
        output_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # 文本框 + 滚动条
        text_frame = ttk.Frame(output_frame)
        text_frame.pack(fill=tk.BOTH, expand=True)

        self.output_text = tk.Text(text_frame, wrap=tk.NONE, font=("Consolas", 10))
        self.output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar_y = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.output_text.configure(yscrollcommand=scrollbar_y.set)

        scrollbar_x = ttk.Scrollbar(output_frame, orient=tk.HORIZONTAL, command=self.output_text.xview)
        scrollbar_x.pack(fill=tk.X)
        self.output_text.configure(xscrollcommand=scrollbar_x.set)

        # ========== 状态栏 ==========
        self.status_var = tk.StringVar(value=GUI_STATUS_READY_TEXT)
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(fill=tk.X, pady=5)

    def calculate(self):
        """计算LQR增益并生成C代码"""
        try:
            # 读取参数
            h = float(self.h_var.get())
            w = float(self.w_var.get())
            b = float(self.b_var.get())
            g = float(self.g_var.get())
            q0 = float(self.q0_var.get())
            q1 = float(self.q1_var.get())
            q2 = float(self.q2_var.get())
            r = float(self.r_var.get())
            v_min = float(self.v_min_var.get())
            v_max = float(self.v_max_var.get())
            n_points = int(self.n_points_var.get())

            # 验证参数
            if h <= 0 or w <= 0 or b <= 0 or g <= 0:
                raise ValueError("物理参数必须为正数")
            if r <= 0:
                raise ValueError("R权重必须为正数")
            if v_min >= v_max:
                raise ValueError("最小速度必须小于最大速度")
            if n_points < MIN_VELOCITY_POINT_COUNT or n_points > MAX_VELOCITY_POINT_COUNT:
                raise ValueError(
                    f"速度点数必须在{MIN_VELOCITY_POINT_COUNT}-{MAX_VELOCITY_POINT_COUNT}之间"
                )

            # 计算
            calc = LQRCalculator(h, w, b, g)
            Q = build_q_matrix(q0, q1, q2)
            R = build_r_matrix(r)
            table = calc.generate_table(v_min, v_max, n_points, Q, R)

            if not table:
                raise ValueError("计算失败，请检查参数")

            # 生成C代码
            code = self.generate_c_code(h, w, b, g, q0, q1, q2, r, table)

            # 显示结果
            self.output_text.delete(1.0, tk.END)
            self.output_text.insert(tk.END, code)

            self.status_var.set(f"计算完成！生成了 {len(table)} 个速度点的LQR增益")

        except ValueError as e:
            messagebox.showerror("参数错误", str(e))
            self.status_var.set("计算失败")
        except Exception as e:
            messagebox.showerror("计算错误", f"发生错误: {str(e)}")
            self.status_var.set("计算失败")

    def generate_c_code(self, h, w, b, g, q0, q1, q2, r, table):
        """生成C语言代码"""
        lines = []
        lines.append("// ==================== LQR增益表（自动生成）====================")
        lines.append(f"// 物理参数: h={h:.3f}m, w={w:.3f}m, b={b:.3f}m, g={g:.2f}m/s^2")
        lines.append(f"// LQR权重: Q=diag({q0:.1f}, {q1:.1f}, {q2:.1f}), R={r:.1f}")
        lines.append("// 状态: x = [phi, phi_dot, delta], 输入: u = delta_dot")
        lines.append("// K = [K_phi, K_phi_dot, K_delta]")
        lines.append("// 控制律: u = -(K[0]*phi + K[1]*phi_dot + K[2]*delta)")
        lines.append("")
        lines.append(f"static const LQR_GainEntry_t {DEFAULT_C_GAIN_TABLE_NAME}[] = {{")

        for i, (v, K) in enumerate(table):
            comment = ""
            if i == 0:
                comment = "  // 低速"
            elif i == len(table) - 1:
                comment = "  // 高速"
            lines.append(
                "    "
                f"{{{v:.1f}f,  "
                f"{{{K[STATE_IDX_PHI]:.4f}f, "
                f"{K[STATE_IDX_PHI_DOT]:.4f}f, "
                f"{K[STATE_IDX_DELTA]:.4f}f}}}},"
                f"{comment}"
            )

        lines.append("};")
        lines.append("")
        lines.append(f"#define {DEFAULT_C_GAIN_COUNT_MACRO}  {len(table)}")

        return "\n".join(lines)

    def copy_code(self):
        """复制代码到剪贴板"""
        code = self.output_text.get(1.0, tk.END).strip()
        if code:
            self.root.clipboard_clear()
            self.root.clipboard_append(code)
            self.status_var.set("代码已复制到剪贴板！")
        else:
            self.status_var.set("没有可复制的代码")

    def clear_output(self):
        """清空输出"""
        self.output_text.delete(1.0, tk.END)
        self.status_var.set("已清空")


def main():
    root = tk.Tk()
    app = LQRCalculatorGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
