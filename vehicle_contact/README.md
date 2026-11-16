# vehicle_contact

轮式载具接触运动模块：显式射线 / 球体扫描确定每轮支撑，悬挂压缩与轮胎
接触力驱动刚体底盘，车轮接地滚动与悬空自由旋转分开累计。

轮地部分（支撑查询、悬挂、轮胎）位于 `engine/wheel_ground/`（JS 镜像
`web/js/wheel_ground.js`），接触坐标系接口统一；旧入口 `engine.vehicle.Vehicle`
保留兼容外观并委托该模块。重构前后的等价性由 `data/evidence/` 的原始记录
逐字段核验。

## 图形入口（推荐）

```bash
cd vehicle_contact
python3 -m cli.vc serve
# 浏览器打开 http://127.0.0.1:8000/
```

可驾驶车辆、实时调整悬架参数、查看轮地接触与车身运动诊断（异常按
轮地接触 / 车身运动分组）、录制 / 回放驾驶输入，并与 Python 参考引擎做
轨迹校验。完整说明见 [`docs/contact_window.md`](docs/contact_window.md)。

## 无头辅助命令

```bash
python3 -m cli.vc record     # 生成标准场景回放与轨迹 CSV
python3 -m cli.vc run edge   # 运行场景并打印接触 / 行程 / 运动诊断
python3 -m cli.vc test       # 引擎测试套件（含重构等价性与边界测试）
node tests/js_parity.js      # JS / Python 双引擎轨迹一致性
```
