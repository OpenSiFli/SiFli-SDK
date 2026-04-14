# 编码电机驱动示例（RT-Thread）

[English](README_EN.md) | [中文](README.md)

源码路径：example/rt_device/motor

## 概述
本示例展示了在 RT-Thread 上驱动并控制一对直流减速电机的完整流程。包含：

- 使用脉冲编码器读取电机转速/位置。
- 使用 PWM 输出控制电机占空比和方向。
- 实现速度环与位置环的 PID 控制（包含抗饱和处理的版本）。
- 提供 finsh 命令行接口（motor）用于实时控制与调试。

### PID 算法简要说明
本例程在速度环与位置环中使用经典的 PID 控制器。PID 控制器根据设定值与测量值的误差产生控制量，用以驱动 PWM 占空比。连续时间下的 PID 控制器表达式为：

$$
u(t)=K_P e(t) + K_I \int_0^{t} e(\tau)\,d\tau + K_D \frac{d e(t)}{d t}
$$

其中，$e(t)=r(t)-y(t)$ 为设定值 $r(t)$ 与被控量 $y(t)$ 的误差，$K_P,K_I,K_D$ 分别为比例、积分、微分增益。

在嵌入式采样实现中，常用离散形式更便于实现（$T_s$ 为采样周期，$k$ 为采样索引）：

$$
u[k]=K_P e[k] + K_I T_s \sum_{i=0}^{k} e[i] + K_D \frac{e[k]-e[k-1]}{T_s}
$$

或以递推形式写成：

$$
u[k]=K_P e[k] + I[k] + K_D \frac{e[k]-e[k-1]}{T_s},\qquad I[k]=I[k-1]+K_I T_s e[k]
$$

抗饱和（抗积分饱和）策略：当驱动信号 $u[k]$ 超出执行器允许范围（例如 PWM 占空比 0~100%）时，应限制输出并停止或回退积分累加，以避免积分分量继续增大导致超调。常用的简单方法为积分限幅（clamping）：

1) 计算未饱和的控制量（含所有项）；
2) 将控制量裁剪到允许范围；
3) 若发生裁剪，则不更新或有条件更新积分项（仅在未饱和或误差方向有利时累加）。

调参建议：先设 $K_I=0, K_D=0$，通过增大 $K_P$ 达到可接受的响应速度，再引入 $K_I$ 消除稳态误差，最后微调 $K_D$ 以抑制超调与振荡。速度环通常采样频率较高（例如数十到数百 Hz），位置环采样可以更低，且通常位置环在速度环之外作为外环。

### 正交编码器

本例使用增量式正交编码器（A/B 双通道）来测量电机转速与位置。编码器输出两路相位差约 90° 的方波，利用计数器对 A/B 通道脉冲进行计数以获得增量值。

接线

- 使用 `ENC1A_PIN` / `ENC1B_PIN` 接编码器 1 的 A/B 通道；使用 `ENC2A_PIN` / `ENC2B_PIN` 接编码器 2（见硬件连接处表）。

采样窗口与计数

- 代码在主循环中按固定窗口读取并清零编码器计数（主循环用 wake_time += 20 控制），窗口约为 20 ms（即 0.02 s）。
- 因此 `pulse_to_rpm(int32_t pulse)` 的输入 `pulse` 是该 20 ms 时间窗口内的脉冲增量。

脉冲与 RPM 的换算

- 将窗口内脉冲换算为每秒脉冲数： pulses_per_second = pulse / 0.02 = pulse * 50
- 使用每转脉冲数（PPR，含编码器本身分辨率、减速比和四倍计数）将脉冲数换算为转速：

  rpm = pulses_per_second / PPR * 60 = pulse * 50 / PPR * 60

- 逆转换（把目标 RPM 转为窗口内脉冲数，用于映射）为：

  pulse = (int)(rpm / 60 * PPR / 50)

PPR 的来源

- 在本例中，PPR 由三部分决定：编码器本体的 PPR、减速箱减速比，以及正交编码的 4 倍计数。计算公式为：

  total_ppr = encoder_ppr * gearbox_ratio * 4

- 代入本例硬件参数： encoder_ppr = 13, gearbox_ratio = 20.409，得到：

  total_ppr = 13 * 20.409 * 4 = 1061.268

示例

- 目标速度 120 rpm 时，预计 20 ms 窗口内脉冲数：

  pulse ≈ 120 / 60 * 1061.268 / 50 ≈ 42

  将检测到的 pulse = 42 带入 `pulse_to_rpm`：

  rpm ≈ 42 * 50 / 1061.268 * 60 ≈ 118.7 rpm

注意

- 脉冲方向：代码使用带符号的脉冲值表示方向（正/负），命令接口通过 `dir` 参数设置目标的正/反向（符号）。
- 请根据你所用编码器与减速箱替换 PPR，若要精确，建议参考编码器数据手册或实测得到。
- 若信号有抖动或噪声，建议在硬件端（滤波）或软件端（去抖、移动平均或低通滤波）处理，以提高速度计算稳定性。

## 用法
程序启动后，通过串口指令控制电机。
finsh指令格式为：motor dir spd num
控制指令参数包括，转动方向，转速，转动圈数。
转动方向(dir)：0表示正转，1表示反转
转速(spd)：单位为rpm，表示每分钟的圈速
转动圈数(num)：总共转动圈数，如果设置为0表示一直转不停
例如：
- motor 0 120 10 表示正转，转速120rpm，转动10圈
- motor 1 180 12 表示反转，转速180rpm，转动12圈

### 支持的开发板
例程可以运行在以下开发板
* sf32lb52-lcd_n16r8

### 硬件参考

- 电机：MG310 电机，带减速箱，减速比1：20.409，空载转速约 300 rpm，额定电压 7.4V。
- 编码器：霍尔正交编码器，PPR=13。
- 驱动器：TB6612FNG（双路、低压驱动，常见模块 TB6612）——优点是体积小、驱动简单，额定电流通常 ≤ 1~2A（峰值更高），适用于小功率减速电机。
- 其他：连接线（杜邦线），稳压/电源模块（可提供足够启动电流），必要时示波器/万用表用于调试。

### 硬件连接

开发板与驱动器：
| 作用 | 宏名 | 管脚号 | 管脚名称 |
|:---|:---:|:---:|:---:|
| PWM A 输出 | `PWMA_PIN` | 36 | PA36 |
| PWM B 输出 | `PWMB_PIN` | 41 | PA41 |
| AIN1 (左电机方向) | `AIN1_PIN` | 44 | PA44 |
| AIN2 (左电机方向) | `AIN2_PIN` | 35 | PA35 |
| STBY (使能) | `STBY_PIN` | 43 | PA43 |
| BIN1 (右电机方向) | `BIN1_PIN` | 42 | PA42 |
| BIN2 (右电机方向) | `BIN2_PIN` | 40 | PA40 |

开发板与编码器：
| 作用 | 宏名 | 管脚号 | 管脚名称 |
|:---|:---:|:---:|:---:|
| 编码器1 A相 | `ENC1A_PIN` | 38 | PA38 |
| 编码器1 B相 | `ENC1B_PIN` | 37 | PA37 |
| 编码器2 A相 | `ENC2A_PIN` | 39 | PA39 |
| 编码器2 B相 | `ENC2B_PIN` | 30 | PA30 |

驱动器与电机：
- 左电机M+接驱动器AO1，M-接AO2
- 右电机M+接驱动器BO2，M-接BO1

**注意**：请根据实际使用的硬件型号调整硬件连接方式，并相应修改代码中的宏定义与电机控制相关实现（例如引脚宏、PWM 极性、使能/方向脚映射、电源要求等）。

**注意（配置与迁移指南）**：如你使用不同型号的电机/驱动器，请按照下列要点逐项检查并修改代码：

- 引脚与宏：首先修改文件顶部的引脚宏（如 `PWMA_PIN`、`PWMB_PIN`、`AIN1_PIN`、`AIN2_PIN`、`STBY_PIN`、`ENC1A_PIN`、`ENC1B_PIN`、`ENC2A_PIN`、`ENC2B_PIN` 等）以匹配你的硬件连接。
- PWM 极性与频率：某些驱动器对 PWM 的极性（高电平有效或低电平有效）或频率有要求。若输出方向与预期相反，可在软件中反转占空比或交换高低电平映射；必要时调整定时器的 PWM 频率。
- 使能/待机与方向信号：确认驱动器的使能（enable / standby）和方向（IN1/IN2）信号的电平极性（高有效/低有效）。如果极性不同，需要在驱动代码中反转对应控制逻辑或调整引脚电平输出。
- 电源与接地：确保电机电源与驱动器电源能提供足够的启动电流，并与主控板共地。错误的电源或未共地会导致驱动异常或编码器读数错误。
- 霍尔编码器接口：霍尔传感器输出可能需要上拉电阻或滤波；确认编码器供电与 MCU 电平兼容（3.3V vs 5V），必要时使用电平转换或隔离。
- 保护与散热：检查驱动器的额定电流与功耗，必要时加装散热片或限流保护（保险丝、电流检测）。

修改顺序建议：
1) 更新引脚宏并编译确认无语法错误；
2) 用简单测试（单向短时运转）验证 PWM 与方向控制逻辑；
3) 连接编码器并确认脉冲计数与方向；
4) 调整 PID 参数并在低速下验证闭环性能。

### 编译和烧录

切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```


## 示例输出
motor 0 120 10
Motor command set:
  Direction: Forward
  Speed: 120 rpm
  Rotations: 10
msh />left speed: 113.072289 rpm, right speed: 113.072289 rpm.
left speed: 118.725899 rpm, right speed: 118.725899 rpm.
left speed: 118.725899 rpm, right speed: 121.552711 rpm.
left speed: 115.899094 rpm, right speed: 115.899094 rpm.
left speed: 118.725899 rpm, right speed: 115.899094 rpm.
left speed: -5.653615 rpm, right speed: 0.000000 rpm.
left speed: 0.000000 rpm, right speed: 0.000000 rpm.

motor 1 180 12
Motor command set:
  Direction: Reverse
  Speed: 180 rpm
  Rotations: 12
msh />left speed: -172.435242 rpm, right speed: -175.262039 rpm.
left speed: -178.088852 rpm, right speed: -178.088852 rpm.
left speed: -175.262039 rpm, right speed: -175.262039 rpm.
left speed: -178.088852 rpm, right speed: -180.915665 rpm.
left speed: 0.000000 rpm, right speed: 0.000000 rpm.
left speed: 0.000000 rpm, right speed: 0.000000 rpm.

## 异常诊断

若电机不转或者转速异常，请检查以下几点：
1. 确认电源连接正确，电压和电流满足电机
2. 检查电机接线是否正确，特别是PWM和方向控制引脚
3. 确认编码器接线正确，A/B通道连接到正确的引脚

如果以上检查后问题依旧存在，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
