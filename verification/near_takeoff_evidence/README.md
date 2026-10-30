# 近地起飞修复依据

本目录保存本次改动之前的证据，不能当作 NEAR-LIFTOFF 新版已经烧录或实际飞行通过。

- `user_decision.json`：用户明确选择 A，软件增加受限近地起飞，恢复测距后转 800 mm 闭环。记录中的 pending 是决策当时状态。
- `range_mm_firmware_match.json`、`range_mm_symbols.json`：上一版 RANGE-MM 用户烧录后，J-Link只读Flash与该版HEX的匹配结果及符号，用于解读此处历史RAM；不能拿这些地址解释新近地版RAM。
- `range_mm_static_verification.json`：上一版拆桨上锁的静态汇总及用户后续更正。约1～30cm测试均手持，对象为桌面，只有80cm测试照地面；后来明确标记foam25/foam_ground的是原试飞泡沫板。返回30cm那次未确认实际距离，不据此推断模块锁死。next_step是作出A决策前的历史状态。
- 各 `*_ram_01/02.json`、`*_read.txt`：对应只读采集与Commander输出。机身约29.5cm时272/276mm、约80cm时766/773mm，单位错误消除；原泡沫板约25cm时234mm有效，正常落地约1cm时无效。非原子读数与手持误差不构成精密标定，也不确定硬件最小量程。
- `before_fix/`：新用例在新增近地状态之前运行的失败记录，包含当时test_core.c与日志。保留失败结论，不纳入当前通过数。

当前近地代码的通过记录位于上层 `software_results.json`、`native/` 和 `near_takeoff_*_results.json`。软件用合成数据检查状态、限幅与时限，未模拟实际升力。没有访问当前近地版实机，助手没有烧录或运行电机。
