# 旧固件诊断证据

本目录是旧版实板只读数据与旧代码失败回放，不纳入当前通过用例统计。实机只在拆桨上锁、四路PWM为零时读取，没有触发C1/C3读取实机故障号。

height30_*：用户架高约300mm，旧固件显示2910～2940mm且有效；height80_*：约800mm手扶，旧固件测距/光流无效，440mm只是旧值。module_ok为1且质量、IMU、光感、遥控仍更新。完整原始Flash/RAM和J-Link日志留在measurements.json指向的工作区诊断目录。

unit_scalar_red/与c1_admission_red/是修复前C桩失败记录；后者复现原始800被拒绝、C1得到mode=1/fault=8。range_arm_before.json是旧ARM固件机器码的单位回放，4个有效高度输入失败。
