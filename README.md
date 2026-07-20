# MSPM0G3507 自动行驶小车 - 2024自动化驾驶竞赛(H题)

## 项目概述

基于 TI MSPM0G3507 MCU 控制的自动行驶小车，使用八路红外巡线模块和 MPU6050 陀螺仪，在指定黑色弧线赛道上自动行驶。

## 赛道信息

- 场地面积：≥220cm × 120cm，白色哑光喷绘布
- 两个对称半圆弧，半径 40cm，黑色线宽 1.8cm
- 四个顶点：A、B、C、D

## 小车硬件

| 模块 | 型号 | 接口 |
|------|------|------|
| 主控 | MSPM0G3507 LaunchPad | - |
| 巡线 | 八路红外串口模块 | UART0 |
| 陀螺仪 | MPU6050 | I2C0 |
| 电机驱动 | TB6612 | TIMA0 PWM |
| 编码器 | 450RPM 减速电机 | GPIO中断 |
| 显示 | 0.96寸 OLED | 软件I2C |
| 底盘 | 两轮+牛眼轮 差分驱动 | - |

## 引脚映射

### 传感器
| 外设 | 引脚 | 功能 |
|------|------|------|
| 巡线模块 TX | PA11 | UART0 RX |
| 巡线模块 RX | PA10 | UART0 TX |
| MPU6050 SDA | PA0 | I2C 数据 |
| MPU6050 SCL | PA1 | I2C 时钟 |

### 电机
| 外设 | 引脚 | 功能 |
|------|------|------|
| 左电机前进 | PB8 | TIMA0 CCP0 |
| 左电机后退 | PB12 | TIMA0 CCP1 |
| 右电机前进 | PA3 | TIMA0 CCP2 |
| 右电机后退 | PB2 | TIMA0 CCP3 |
| 左编码器 A/B | PB21/PB22 | 速度反馈 |
| 右编码器 A/B | PB13/PA31 | 速度反馈 |

### 交互
| 外设 | 引脚 | 功能 |
|------|------|------|
| OLED SCL/SDA | PA15/PA30 | 调试显示 |
| K1/K2/K3 | PA18/PB23/PB20 | 模式选择 |
| 蜂鸣器 | PB5 | 声提示 |
| RGB WS2812 | PB10 | 光提示 |

## 软件架构

### 控制策略
- **Mode 1 直行段**：MPU6050 航向 PI 外环生成左右轮目标速度差，两个编码器增量式 PI 内环分别生成 PWM
- **弧线段**：纯八路巡线 PID 跟踪，Yaw 角度判断终点

### Mode 1 串级直线控制

1. 选择 Mode 1 后，程序继续读取 DMP，等待 20 个有效航向样本。
2. 将当前 MPU6050 航向保存为目标航向，车速从 0 缓升到 200mm/s。
3. 航向外环每 20ms 计算一次角度误差，输出限制在 ±60mm/s 的差速修正。
4. 左右轮各自使用编码器反馈和增量式 PI，将目标速度转换成 PWM。
5. 程序先确认小车已经离开 A 点黑线，然后连续三次检测到 B 点黑线才停车。
6. MPU6050 数据超过 80ms 未更新时进入故障停车，防止使用失效航向继续运行。

### Mode 1 首次调试步骤

1. 先把小车架空，保证两个轮子离地，再选择 Mode 1。
2. 在 Keil Watch 窗口观察 `motor_data.speed_set[0]`、`motor_data.speed_set[1]`、`motor_data.speed_mm_s[0]`、`motor_data.speed_mm_s[1]`。两个目标为正时，两个反馈也必须为正。
3. 如果某个轮子前进时反馈为负，只修改 `BSP/Motor/bsp_encoder.h` 中对应的 `ENCODER_LEFT_FORWARD_SIGN` 或 `ENCODER_RIGHT_FORWARD_SIGN`，取值在 `1` 和 `-1` 之间切换。
4. 手动向左转动车头，观察 `straight_yaw_error` 和 `straight_speed_correction`；控制器应产生让车头向右恢复的左右目标速度差。如果恢复方向相反，应先停止测试并核对 MPU6050 安装方向。
5. 架空测试没有反转和前后摩擦后，再放到地面进行 1m 直线测试。
6. 先只调速度内环：保持 `SPD_KD_POS=0`，逐步调整 `SPD_KP_POS`，确认不一停一走后再小幅增加 `SPD_KI_POS`。
7. 速度内环稳定后再调航向外环：优先调 `STRAIGHT_YAW_KP`，最后小幅调 `STRAIGHT_YAW_KI`；不要同时大幅修改两层参数。

### 关键调试参数

| 参数 | 文件 | 初始值 | 作用 |
|------|------|--------|------|
| `SPD_KP_POS` | `APP/bsp_PID_motor.h` | 0.30 | 左右轮速度环比例系数 |
| `SPD_KI_POS` | `APP/bsp_PID_motor.h` | 0.025 | 补偿长期负载和电机差异 |
| `SPD_KD_POS` | `APP/bsp_PID_motor.h` | 0 | 初始关闭，避免编码器量化噪声引起抖动 |
| `STRAIGHT_YAW_KP` | `BSP/control/control.c` | 4.0 | 航向误差转为差速修正的比例系数 |
| `STRAIGHT_YAW_KI` | `BSP/control/control.c` | 0.15 | 消除长期航向残差 |
| `STRAIGHT_YAW_MAX_CORRECTION` | `BSP/control/control.c` | 60mm/s | 限制最大转向力度 |
| `STRAIGHT_YAW_SLEW_STEP` | `BSP/control/control.c` | 3mm/s/周期 | 限制转向变化速度，使纠偏平滑 |

### 巡线算法（八路加权平均）
```
传感器权重：X1=-20, X2=-15, X3=-10, X4=-5, X5=+5, X6=+10, X7=+15, X8=+20
err = Σ(值 × 权重) / 检测到黑线数
```

### 任务调度
- 20ms 周期：编码器更新 + 电机PID控制
- 10ms 周期：MPU6050 姿态获取
- 5ms 周期：DMP 欧拉角读取

## 四个模式

| Mode | 路径 | 限时 |
|------|------|------|
| 1 | A → B | ≤15s |
| 2 | A → B → C(弧) → D → A(弧) | ≤30s |
| 3 | A → C → B(弧) → D → A(弧) | ≤40s |
| 4 | Mode3 × 4圈 | 越快越好 |

## 开发环境

- **IDE**: Keil MDK / TI CCS
- **SDK**: MSPM0 SDK v2.02.00.05
- **SysConfig**: v1.21.1+

## 参考资源

- [雅博 MSPM0 智能小车](https://www.yahboom.com/study/MSPM0-SmartCar)
- [TI MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507)
