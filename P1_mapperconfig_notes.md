# S570 集成备忘

> 实机调试时需要修改 config.yaml，以下调用已在 mapper.py/s570.py 中实现，
> 但 **run_system.py 尚未全部调用**。联调时补充即可。

## run_system.py 需补充的调用

在 `factory.create_mapper()` 之后、`mapper.calibrate()` 之前，增加三行：

```python
mapper = factory.create_mapper()

# ── 补充：从 config.yaml 同步配置到 mapper ──
mapper.set_joint_mapping(cfg.s570.joint_mapping)         # S570→UR 关节对应
mapper.set_all_directions(cfg.s570.joint_direction)      # 关节方向 ±1
mapper.set_scale(cfg.mapper.scale)                       # 运动范围缩放
# joint_offset 逐关节设置（如需要）:
for i, off in enumerate(cfg.mapper.joint_offset):
    mapper.set_offset(i, off)

# 然后才是零点校准
if cfg.device == "s570" and cfg.s570.enable_calibration:
    ...
    mapper.calibrate(state.joint)
```

## config.yaml 可调参数

| 参数 | 含义 | 调试时机 |
|------|------|----------|
| `joint_mapping` | 哪6个S570关节→UR12e，1-based | 关节错位时调整（如 J3控制UR J2） |
| `joint_direction` | 每个关节旋转方向 ±1 | 从臂反向跟随主臂时 |
| `scale` | 全局运动缩放 | 人臂运动范围与机械臂不匹配时 |
| `joint_offset` | 逐关节零位微调 | calibrate() 后个别关节仍有偏差 |
| `enable_calibration` | 启动时自动零点 | 开发期可关闭跳过校准 |
| `active_arm` | 左臂/右臂 | 切换穿戴手臂 |
