# SysID

面向 RoboMaster 电机与云台的闭环系统辨识模块，用实测数据估计实装机构的等效转动惯量 **J**、粘性阻尼 **b** 和库仑摩擦项 **fc**，为 LQR 建模、力矩前馈和运动仿真提供参数依据。支持 XRobot/YAML 实例化，在 Ozone 中修改命令即可启动实验。

## 为什么需要系统辨识

同一款电机装上不同云台后，实际运动取决于整个机构的惯量、传动、线束、摩擦和负载。电机型号本身不能给出装配后机构的全部动力学参数；机械计算惯量也不能直接给出粘性阻尼和摩擦。

如果只凭经验填写 J、b，控制器或仿真模型可能与真实机构不一致。系统辨识通过“施加已知输入—记录运动响应—拟合并复测”的流程，为这些参数提供实验依据：

- **转动惯量 J**：描述力矩改变角速度的难易程度，用于加速度前馈和动力学建模。
- **粘性阻尼 b**：描述与角速度相关的阻力项，用于速度动态和 LQR 模型。
- **库仑摩擦 fc**：描述运动中随方向改变符号的阻力项，可作为摩擦补偿的候选参数；它不同于起动静摩擦阈值。

本模块使用局部单轴模型：

`τ = J·dω/dt + b·ω + fc·sign(ω) + k·q + offset`

其中 k 和 offset 用于容纳局部位置相关负载和偏置。辨识结果属于对应负载、安装姿态和机械固定条件，需要在另一组激励数据上检查重复性。

## 历史实测成果：Double Yaw 小 yaw（2026-11-17）

验证对象为 GM6020 驱动的小 yaw，固定底座、大 yaw 和 pitch，以电机编码器提供位置、IMU 的有符号 yaw 分量提供角速度。实验采用三阶段 PRBS＋弱 PD 回中，持续约 15 秒。以下引用的是原 Double Yaw 实测报告，采用当时的积分最小二乘流程；不混用当前 v3 的新采样结果或模拟数据。

### 惯量对照中最接近机械参考的一次候选

| 项目 | 数值 |
| --- | ---: |
| 机械提供的惯量参考值 | 0.005460 kg·m² |
| 辨识惯量 J | 0.005506 kg·m² |
| 相对参考值差异 | 约 0.84% |
| 输入与积分窗口 | 命令力矩，80 ms |
| 有效拟合窗口数 | 50 |

来源：[原始拟合报告](examples/results/2026-11-17/yaw_actual_run1.fit.json)、[实验配置与完成状态](examples/results/2026-11-17/yaw_actual_run1.metadata.json)。这次拟合没有固定 J 为机械参考值。

### 为什么辨识值可能略大于机械参考值

这次候选比机械参考值高约 **0.0000458 kg·m²（0.84%）**。一个可能原因是两者覆盖的物理对象不同：机械参考若主要计算刚性结构，未完整计入随云台运动的线束、接插件等，而实机辨识得到的是当前装配条件下的**等效动力学参数**，随动线束的质量可能贡献额外惯量。

线束还可能产生扭转弹性、摩擦和阻尼，这些作用不应全部解释为惯量；在局部近似模型中，它们可能进入 k、b、fc，也可能通过参数耦合影响 J 的估计。测量时差、力矩换算误差和有限数据同样可能造成这一级别的差异。因此，“计入了线束影响”是合理的待验证解释，**不能把这 0.0000458 kg·m² 直接认定为线束惯量**。验证时应核对机械参考包含的随动部件，并记录安装与走线条件进行对照。

**这是单次、单输入、单窗口下的惯量对照结果，不是模块的普遍精度指标。**同一候选中的 b 落在零下边界，因此不能据此宣称 J、b、fc 已同时得到高精度辨识，也不能用“0.84%”代表所有记录的误差。

### 另一组未触参数边界的联合候选

| 参数 | 估计值 |
| --- | ---: |
| 转动惯量 J | 0.005136 kg·m² |
| 粘性阻尼 b | 0.06796 N·m·s/rad |
| 库仑摩擦项 fc | 0.03154 N·m |

这组结果来自第三次记录的反馈力矩输入、80 ms 积分窗口，含 105 个有效窗口，拟合参数未触边界。来源：[拟合报告](examples/results/2026-11-17/yaw_actual_run3.fit.json)、[实验配置](examples/results/2026-11-17/yaw_actual_run3.metadata.json)。b 与 fc 没有独立真值对照，保留为待复测的候选值。

历史结果与计算口径汇总在 [summary.json](examples/results/2026-11-17/summary.json)。本次文档附上拟合与配置报告，不包含原始采样文件，不能仅凭报告独立重算全部过程。

## 模块如何工作

三阶段二电平 PRBS 提供力矩激励，弱 PD 将机构保持在启动位置附近；记录实际 CAN 打包命令及入队结果，通过 IMU 与编码器采样拟合参数。当前 v3 采用同区间 IMU 积分平均采样，减少降采样前丢弃样本带来的误差。

依赖 QDU-Robomaster 的 Motor/RMMotor/DMMotor，以及 xrobot-org 的 BMI088、MadgwickAHRS 和 LibXR。通用模块不预设机械惯量、机器零点或专用限位；具体硬件、方向和实验参数通过 YAML 配置。

## 按 XRobot 1.0.1 接入

1. 在 Modules/sources.yaml 的 sources 列表增加：

```yaml
- url: https://raw.githubusercontent.com/QDU-Robomaster/SysID/main/index.yaml
  priority: 0
```

2. 在 Modules/modules.yaml 的 modules 列表增加 QDU-Robomaster/SysID@main。
   SysID.hpp 的 depends 自动拉取 Motor、RMMotor、DMMotor、BMI088、MadgwickAHRS。
3. 参考 [examples/small_yaw.yaml](examples/small_yaw.yaml)，依次实例化 IMU、AHRS、电机、SysID。
4. **电机的 can_bus 使用 SysID::Observe(can1)，SysID 的 can_bus 使用实际 can1。**
   记录层透明转发收发数据，并记录原驱动打包结果及 AddMessage 入队结果，不修改 QDU 驱动。
5. 执行 xrobot setup、xrobot format、xrobot gen -c User/xrobot.yaml，然后正常构建。
   无需写 app_main 调用、额外启动头文件或 Poll 线程。

SysID 对被测电机独占 Update/Control/Relax。一个物理 CAN 总线一次绑定一个辨识目标，
被测电机的控制组也须独占；不能同时交给 Gimbal 或其他控制器。
本模块记录成功入队的实际字节，不把入队成功等同于电机已执行。

## 恢复的实验流程

| 阶段 | 时间 | 激励 |
| --- | --- | --- |
| 稳定 | 0～2 s | 只有回中 PD |
| 快档 | 2～6 s | 二电平 ±A，保持 0.04 s |
| 中档 | 6～10 s | 二电平 ±0.65A，保持 0.10 s |
| 慢档 | 10～14 s | 二电平 ±A，保持 0.25 s |
| 收尾 | 14～15 s | 只有回中 PD |

默认 A=0.07 N·m、kp=2、kd=0.12、总力矩上限 0.20 N·m、相对单侧行程 0.06 rad、seed=0xACE1。
这些值在 Ozone 启动时锁存。开始前可编辑 amplitude_、kp_、kd_、torque_limit_、max_travel_、
min_position_、max_position_、seed_。默认绝对边界为 ±1000000，实际机构应在 YAML 指定。

Experiment.hpp 保留原 Double Yaw 方案的 LFSR、阶段切换和先积分上一控制区间再记录的顺序。
核心的参考模式与原核心逐样本一致；实际适配器现在启用 v3 区间测量，以免在 100 Hz 下直接抽取高带宽 IMU 的端点值。
IMU 使用带硬件时间戳的 32 条队列，消费每次发布；PD 使用当前控制区间的陀螺仪平均值，日志使用整段记录区间的原始陀螺仪积分均值。
与原专用 GM6020 代码的区别是接入可配置、参数/故障码与文件版本标准化。
这项软件对照不保证物理实验每次得到同一个 J、b，仍需相同负载、安装和固定条件验证。

## 单位、方向与硬件条件

位置由电机编码器提供，用于回中/限位；角速度由 IMU 提供，用于 PD 和拟合。
角度/角速度/力矩必须对应同一个机械自由度，单位 rad、rad/s、N·m。
固定底座、大 yaw 和 pitch，使 pitch 接近机械零位、安装轴对齐且 roll 固定。
直接使用 gyro 分量没有任意姿态下的 yaw 投影补偿。pitch 检查恢复为相对启动姿态变化不超过 0.05 rad。

YAML 中的 gyro_axis 是 IMU rotation 之后的分量（0/1/2 对应 x/y/z），gyro_sign 统一到 position 正方向。
pitch_zero 只补偿已确认的坐标零偏，不能用它把实际抬头伪装成水平。
BMI088 话题类型 Eigen::Matrix<float,3,1>，Euler 为 LibXR::EulerAngle<float>。
gyro 20 ms、Euler 50 ms、电机反馈 50 ms 未更新会退出，不回退到其他速度源。

公开版反向 RM 的反馈力矩保持原始符号，所以示例 reverse=true 时 feedback_torque_sign=-1。
DM 已翻转反馈力矩，通常为 +1。实际 CAN 命令映射使用参数：

- tx_id：控制帧 ID；tx_byte_offset：两个力矩字节起始位置。
- tx_encoding：SIGNED16（RM 电流编码）或 UNSIGNED12（DM MIT 末 12 位）。
- tx_torque_scale / tx_torque_bias：位置正方向力矩 = 原始编码 × scale + bias。
- torque_constant：记录用电机力矩常数，不能填机械惯量。

GM6020 示例使用 0x1FE、字节 0、scale=(-0.741f*3.0f/16384.0f)、bias=0、torque_constant=0.741。
这是示例机器和驱动的编码配置，不写死在算法中。DM MIT 可设置字节 6，scale=方向×2Tmax/4095、
bias=方向×(-Tmax)，并核实电机内部量程与驱动一致。

## Ozone 启动与观测

实例名建议 g_sysid。上电 ready_=true 后线程运行但实验空闲，command_=0。
确认反馈、位置及安装条件后将 command_=1；默认运行 15 s 后 command_ 自动清零。
command_=0 可中止。原 command_=2 的独立检查模式不属于恢复流程。
试验时不要暂停 CPU；完成后再暂停并导出。重新启动会覆盖上一条记录。

done_ 只表示结束，status_=2 才是正常完成。状态码：0 等待、1 运行、2 完成、3 输入/参数非法、
4 电机失联、5 超速、6 绝对位置边界、7 缓冲区满、10 相对行程、11 过温、12 命令非法、
13 IMU 失联、14 pitch 变化、15 CAN 入队失败、16 手动停止、17 周期非法、18 未观察到命令帧。
18 通常需检查电机是否使用 SysID::Observe(can1)、目标控制帧映射和控制组独占条件。

Watch 可看 last_rx_us_、motor_yaw.no_response_count_、motor_yaw.feedback_.position/omega/torque/temp、
omega_imu_、omega_motor_、pitch_imu_、time_s_、count_、stage_、prbs_、pd_、tau_cmd_。
短名称无法解析时在生成的 XRobotMain 中展开静态实例。

## 导出：新版恢复格式

**起点改为 &g_sysid.capture_，Size 十六进制 12080，即 73856 字节。不要再使用 buf_ / C008。**
先确认 done_=true 并暂停 CPU，再 Save Memory Data。收起 Watch 大数组，不展开整个 capture_。
格式为 128 字节头＋1536×48 字节样本。头中记录配置、有效数量和终止状态；样本记录时间、
绝对位置端点、IMU 角速度区间均值、电机速度端点、总命令/反馈/激励/PD 区间均值、pitch、位置区间均值、阶段、标志。
新版本号为 3；第十个 float 由旧 gyro_y 改为位置区间均值，大小不变。终止码采用本模块定义。

```powershell
python -m pip install -r requirements.txt
python identify.py capture.bin --fit
```

输出 CSV、metadata.json、fit.json。解码器也能读取之前 C008 的四电平格式用于历史比较。
v1/v2 数据保留旧积分最小二乘；v3 将区间均值放到共同时间中心，使用相同线性滤波处理等式各项，并以 PRBS 构造工具变量做两阶段积分拟合，减少速度测量误差和闭环相关性带来的偏差。
分别比较命令/反馈力矩和 40/80/160 ms 窗口，同时输出 ±20 ms 输入对齐候选，按末段留出残差筛选，绝不按已知 J 选择时延。时延结果不是独立传感器标定，仍需不同种子的记录验证。
模型 tau=J*domega/dt+b*omega+fc*sign(omega)+k*q+offset；总输入已含 PD，不再从 b 中减 kd。
--fixed-j 只用于用户提供独立可信 J 后的条件拟合，默认不固定 J。

## 测试

安装 numpy/scipy 和 g++ 后运行 python tests/run_tests.py。
包括原核心逐样本对照、实际 CAN 编码和入队错误、CAN 中断时序、YAML 参数及线程初始化、
旧/新解码、全速降采样前抗混叠测试与含噪闭环模拟参数恢复。tests/reference 保存原核心作为对照，不参与固件。

模拟验证使用独立参数 J=0.006/0.012/0.024、b=0.04/0.09/0.15 及 0.08 rad/s 的源端速度噪声；它不能替代真实机构的精度验证。
