# 底盘运动学

本文解释 [ChassisMotion::init/update](chassis_core/chassis_motion/chassis_motion.cpp) 的当前公式；模块执行与功率链见 [底盘](README.md)。几何数值来自 [整车配置](../config.h)。

## 输入与尺度

`v` 是 m/s，`theta` 和 `w_rot` 分别为 rad、rad/s，轮半径 r、轴距 L、轮距 W 为 m。cmd 先将操作者方向与云台角组合，再把 theta 交给 motion；不要对这个 theta 再叠加旧推导里的 45°。

若配置 D 为正，初始化先令 `L=W=D×sqrt(2)`；否则使用正的 L、W。几何不满足这两个条件时 init 返回，未提供初始化成功状态。

基础尺度为 `move_scale=1/(sqrt(2)×r)`、`rotate_scale=(L+W)/(2×sqrt(2)×r)`；麦轮类型将两者再乘 sqrt(2)，得到 `1/r` 和 `(L+W)/(2r)`。

## 四轮分配

```text
x = v × (cos(theta) - sin(theta)) × move_scale
y = v × (sin(theta) + cos(theta)) × move_scale
z = w_rot × rotate_scale

rf =  y + z
lf = -x + z
lr = -y + z
rr =  x + z
```

轮序固定为 `[rf, lf, lr, rr]`。上述量是中间 rad/s 轮速；源码随后对四项整体取负并乘 rad→deg 系数，传给速度环。平移分量的左轮取反、旋转分量同号和最后整体取负共同构成当前电机符号约定，不能仅从中间变量推定实物正转方向。

当 theta=0、w_rot=0 时，中间量为 `[v/r, -v/r, -v/r, v/r]`（麦轮）；纯旋转时四项中间量均为 z。这里是公式代入说明，不是新增实车验证。几何、安装方向或电机方向变化后，需重新检查实际移动方向。
