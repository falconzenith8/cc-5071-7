# vehicle_contact

轮式载具接触运动模块：显式射线 / 球体扫描确定每轮支撑，悬挂压缩与轮胎
接触力驱动刚体底盘，车轮接地滚动与悬空自由旋转分开累计。

## 图形入口（推荐）

```bash
python3 -m cli.vc serve
# 浏览器打开 http://127.0.0.1:8000/
```

可驾驶车辆、实时调整悬架参数、查看轮地接触与车身运动诊断、录制 / 回放
驾驶输入，并与 Python 参考引擎做轨迹校验。完整说明见
[`docs/contact_window.md`](docs/contact_window.md)。

## 无头辅助命令

```bash
python3 -m cli.vc record     # 生成标准场景回放与轨迹 CSV
python3 -m cli.vc run edge   # 运行场景并打印接触 / 行程 / 运动诊断
python3 -m cli.vc test       # 引擎测试套件
node tests/js_parity.js      # JS / Python 双引擎轨迹一致性
```
