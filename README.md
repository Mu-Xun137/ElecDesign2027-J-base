# 无人机 J 题交付 · J31 · 近地起飞修复版 · 2026-10-29

基础版为默认工程，实现受限近地起飞后转入 800 mm 闭环、C1 悬停、C4 实时推力对位接管、C2 急停、BH1750 检光及强制稳定悬停、灯效和遥控蜂鸣反馈。2026-10-30 按用户要求，将近地起飞修复版同步到当前 `Remote and mini 5.0`。另存的近地起飞、测距单位修复和油门调整目录保留；本次不生成压缩包。

2026-10-29 近地起飞修复：版本 `J31-20261029-NEAR-LIFTOFF`。用户已确认近地测距盲区，并选择方案 A：解锁后尚未启动过电机时，允许地面底杆 C3 进入最多 2 秒的 IMU 姿态闭环起飞阶段。公共推力按 450 PWM/s 上升至 400 PWM；取得至少 200 mm 的健康高度和垂速、读取时间戳持续更新并确认 100 ms 后，从当前实测高度平滑转入 800 mm 闭环。2 秒内不能接管则停机，C2、失联、IMU、倾覆和控制超时保护保留。近地没有高度反馈，400 PWM 尚未实测标定。详见 [实现、参数和拆桨验收](docs/近地起飞阶段与验证_20261029.md)。

保留此前测距单位修复：本实机原始高度已经是 mm，不乘 10。用户烧录 RANGE-MM 后，拆桨静态读数约 29.5 cm 对应 272/276 mm、约 80 cm 对应 766/773 mm；正常落地约 1 cm 时无有效高度。该结果验证单位修复，不能作为本新版起飞验证。本次同步未烧录或试飞；现有配套 J31 遥控器无需重烧。

此前的 TIM1 初始化及 RCR=0 修复保留。详见 [TIM1 修复记录](docs/TIM1控制超时修复_20261029.md)。手动油门增益仍为原基线 0.45。

2026-10-27 接线更新：GY-302 改用 UART1 插座的 **TX1/PA9→SCL、RX1/PA10→SDA**，VCC→3V3、GND/ADDR→GND；激光 LC08 和 MPU6050 继续使用 PB6/PB7。当前基础及升级飞控 HEX 均对应此接法，UART1 不再用于串口。详见 [改线与固件说明](docs/BH1750独立总线接线与验证.md)。遥控源码及无线协议不变。

| 用途 | 工程 / 文件 |
| --- | --- |
| 基础飞控 Keil 入口 | [flight/mini.uvprojx](flight/mini.uvprojx) |
| 本次近地起飞与验收说明 | [docs/近地起飞阶段与验证_20261029.md](docs/近地起飞阶段与验证_20261029.md) |
| 配套遥控 Keil 入口 | [remote/remote.uvprojx](remote/remote.uvprojx) |
| 本次基础固件 | [firmware/basic/](firmware/basic/)：`flight_basic.hex`、`remote_basic.hex` |
| 单光源、A/B 增量 | [upgrade/README_替换与回退.md](upgrade/README_替换与回退.md) |
| 操作、灯效和声音 | [docs/使用与台架验收.md](docs/使用与台架验收.md) |
| 参数与实物标定 | [docs/参数与标定.md](docs/参数与标定.md)、[完整宏表](docs/集中参数完整表.md) |
| 状态、故障与无线协议 | [docs/控制与协议说明.md](docs/控制与协议说明.md) |
| 构建、测试与能力边界 | [docs/验证与限制.md](docs/验证与限制.md) |

先使用基础版。无线协议仍为J31，已运行配套J31的遥控器无需重复烧录，不能混用原始单向协议固件。C3 不解锁；先按原来的低油门→高油门→低油门流程人工解锁，再按 C3。

当前实机LC08的原始高度已是mm，直接使用，不乘10。`80 cm = 800 mm`是目标的单位表达换算，目标仍为800mm。光源检测基于冻结环境基线和照度阈值，基础版需要人工水平对准。

升级代码提供真正的九点移动采样、十字邻域峰值复核和 A/B 顺序状态机。交付升级 HEX 的 `J_NAV_CALIBRATED=0`，会拒绝未经尺度标定的自主搜索；完成标定并改为 1 后必须重新构建。单/双模式均有以同一实际 C 实现运行的已标定条件软件测试。

本次只读实机观察针对原版及用户自行烧录的 RANGE-MM。当前 NEAR-LIFTOFF 完成本机构建和仿真，助手没有烧录或运行电机。近地超时直接停机，已离地时会下落；限制推力和时间不保证真实高度上限。推力、PID、光照阈值、光流尺度及飞行效果仍须实测。报警使用遥控蜂鸣器；无线 C2 不等于题面独立硬件急停。没有新增备用高度源、自动降落或可靠低电量保护。

重建命令（在交付目录运行，需 Python 3 和本机 ARMCC 5.06）：

```powershell
python -X utf8 verification/build.py --variant basic --board both
python -X utf8 verification/build.py --variant upgrade_single --board flight
python -X utf8 verification/build.py --variant upgrade_ab --board flight
python -X utf8 verification/verify.py --suite all
```

交付打包前还需执行实际ARM机器码验证。以下使用本机诊断目录中的Unicorn 2.1.4；其他电脑可在Python环境安装同版本，按位置指定`--dependency-path`，或已可直接导入时省略该参数。

```powershell
foreach ($variant in @('basic','upgrade_single','upgrade_ab')) {
    python -X utf8 verification/verify_timer.py --variant $variant --dependency-path ../diagnostics_20261029/python_deps
    if ($LASTEXITCODE -ne 0) { throw 'Timer check failed' }
    python -X utf8 verification/verify_range_units.py --variant $variant --dependency-path ../diagnostics_20261029/python_deps
    if ($LASTEXITCODE -ne 0) { throw 'Range unit check failed' }
    python -X utf8 verification/verify_near_takeoff.py --variant $variant --dependency-path ../diagnostics_20261029/python_deps
    if ($LASTEXITCODE -ne 0) { throw 'Near takeoff check failed' }
}
```

构建器只调用 ARMCC/ARMASM/ARMLINK/FROMELF，检查源文件路径、容量、HEX 校验和、启动向量和原标定页空洞；不调用烧录工具。测试默认使用相邻 `27-diansai/验证/工具/tcc/tcc.exe`，也可通过 `verify.py --tcc 路径` 指定 TinyCC。逐条结果、工具命令、资源占用和源文件哈希保存在 `verification/`，最终汇总见 [verification/最终核验.json](verification/最终核验.json)。

2026-10-30 目录同步：本次复用已通过验证的同字节源码和固件，逐项复核编译输入、软件测试输入及固件哈希；未重复编译或运行测试，不执行打包器。Keil 的 HEX/AXF 和对应链接表已同步到当前工程输出位置，调试配置、界面状态和 Git 元数据保留。同步结果见 [verification/同步核验_20261030.json](verification/同步核验_20261030.json)。此前已有 ZIP 保持原内容，不代表当前同步后的目录。后续首次在 Keil 构建请使用 Rebuild。
