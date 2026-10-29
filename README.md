# 无人机 J 题交付 · J31 · 2026-10-27

基础版为默认工程，已实现 800 mm 闭环起飞、C1 悬停、C4 实时推力对位接管、C2 急停、BH1750 检光及强制稳定悬停、完整灯效和遥控蜂鸣反馈。保留两套指定原工程，所有修改均在本目录。

2026-10-29 修复：已补齐 TIM1 时基结构体初始化并明确设定重复计数器为 0，消除因未初始化 RCR 将 3 ms 控制周期扩大、解锁后触发故障 13 的缺陷。基础、单光源和 A/B 飞控 HEX 已重新生成；配套遥控固件不变。代码构建与 ARM 初始化仿真已通过，尚未烧录或进行修复后的实板验证。详见 [TIM1 修复与验证](docs/TIM1控制超时修复_20261029.md)。

2026-10-27 接线更新：GY-302 改用 UART1 插座的 **TX1/PA9→SCL、RX1/PA10→SDA**，VCC→3V3、GND/ADDR→GND；激光 LC08 和 MPU6050 继续使用 PB6/PB7。当前基础及升级飞控 HEX 均对应此接法，UART1 不再用于串口。详见 [改线与固件说明](docs/BH1750独立总线接线与验证.md)。遥控源码及无线协议不变。

| 用途 | 工程 / 文件 |
| --- | --- |
| 基础飞控 Keil 入口 | [flight/mini.uvprojx](flight/mini.uvprojx) |
| 配套遥控 Keil 入口 | [remote/remote.uvprojx](remote/remote.uvprojx) |
| 本次基础固件 | [firmware/basic/](firmware/basic/)：`flight_basic.hex`、`remote_basic.hex` |
| 单光源、A/B 增量 | [upgrade/README_替换与回退.md](upgrade/README_替换与回退.md) |
| 操作、灯效和声音 | [docs/使用与台架验收.md](docs/使用与台架验收.md) |
| 参数与实物标定 | [docs/参数与标定.md](docs/参数与标定.md)、[完整宏表](docs/集中参数完整表.md) |
| 状态、故障与无线协议 | [docs/控制与协议说明.md](docs/控制与协议说明.md) |
| 构建、测试与能力边界 | [docs/验证与限制.md](docs/验证与限制.md) |

先使用基础版。飞控与遥控须配套更新为本次 J31 协议，不能混用旧遥控固件。C3 不解锁；先按原来的低油门→高油门→低油门流程人工解锁，再按 C3。

高度只在 LC08 入口换算一次：`原始 cm × 10 = mm`，`80 cm = 800 mm`。光源检测基于冻结环境基线和照度阈值，基础版需要人工水平对准。

升级代码提供真正的九点移动采样、十字邻域峰值复核和 A/B 顺序状态机。交付升级 HEX 的 `J_NAV_CALIBRATED=0`，会拒绝未经尺度标定的自主搜索；完成标定并改为 1 后必须重新构建。单/双模式均有以同一实际 C 实现运行的已标定条件软件测试。

本次只修改源码、编译和执行本机桩测试，没有烧录、连接飞控调试器或运行电机。初始 PID、推力、光照阈值、光流尺度及 LED 实物映射仍须按说明验证。报警使用遥控蜂鸣器；无线 C2 不等于题面独立硬件急停。没有新增备用高度源、自动降落或可靠低电量保护。

重建命令（在交付目录运行，需 Python 3 和本机 ARMCC 5.06）：

```powershell
python -X utf8 verification/build.py --variant basic --board both
python -X utf8 verification/build.py --variant upgrade_single --board flight
python -X utf8 verification/build.py --variant upgrade_ab --board flight
python -X utf8 verification/verify.py --suite all
```

构建器只调用 ARMCC/ARMASM/ARMLINK/FROMELF，检查源文件路径、容量、HEX 校验和、启动向量和原标定页空洞；不调用烧录工具。测试默认使用相邻 `27-diansai/验证/工具/tcc/tcc.exe`，也可通过 `verify.py --tcc 路径` 指定 TinyCC。逐条结果、工具命令、资源占用和源文件哈希保存在 `verification/`，最终汇总见 [verification/最终核验.json](verification/最终核验.json)。
