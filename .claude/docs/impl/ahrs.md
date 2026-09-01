# AHRS 模块规划

## 安装角旋转

### 旋转顺序

安装过程：先 yaw → 再 pitch → 再 roll（ZYX）
逆旋转（sensor → body）：先 roll → 再 pitch → 再 yaw（X → Y → Z）

### 分步旋转公式（不取反，直接用 board 角度）

**第一步：Roll 旋转（绕 X 轴，角度 θ）**
```
New Y = Y·cos(θ) - Z·sin(θ)
New Z = Y·sin(θ) + Z·cos(θ)
X 不变
```

**第二步：Pitch 旋转（绕 Y 轴，角度 φ）**
```
New X = X·cos(φ) + Z·sin(φ)
New Z = -X·sin(φ) + Z·cos(φ)
Y 不变
```

**第三步：Yaw 旋转（绕 Z 轴，角度 ψ）**
```
New X = X·cos(ψ) - Y·sin(ψ)
New Y = X·sin(ψ) + Y·cos(ψ)
Z 不变
```

### 矩阵形式（Rz × Ry × Rx，不取反）

```c
ahrs->install_rot[0] = cy*cp;
ahrs->install_rot[1] = -sy*cr + cy*sp*sr;
ahrs->install_rot[2] = sy*sr + cy*sp*cr;
ahrs->install_rot[3] = sy*cp;
ahrs->install_rot[4] = cy*cr + sy*sp*sr;
ahrs->install_rot[5] = -cy*sr + sy*sp*cr;
ahrs->install_rot[6] = -sp;
ahrs->install_rot[7] = cp*sr;
ahrs->install_rot[8] = cp*cr;
```

### 验算（board_yaw=90°, board_pitch=0°, board_roll=90°）

**Sensor X = (1,0,0)：**
```
Roll(90°):  Y=0, Z=0  → (1, 0, 0)
Pitch(0°): 无变化      → (1, 0, 0)
Yaw(90°):  X=0, Y=1   → (0, 1, 0)
```
sensor X → body Y（左方）✓

**Sensor Y = (0,1,0)：**
```
Roll(90°):  Y=0, Z=1  → (0, 0, 1)
Pitch(0°): 无变化      → (0, 0, 1)
Yaw(90°):  X=0, Y=0   → (0, 0, 1)
```
sensor Y → body Z（上方）✓

**Sensor Z = (0,0,1)：**
```
Roll(90°):  Y=-1, Z=0 → (0, -1, 0)
Pitch(0°): 无变化      → (0, -1, 0)
Yaw(90°):  X=1, Y=0   → (1, 0, 0)
```
sensor Z → body X（前方）✓

### 状态

- [x] 初始四元数加安装角旋转
- [x] 角速度零偏旋转到 body frame
- [ ] 确认矩阵公式正确性（待测试验证）

## 其他待办

- [ ] 磁力计融合测试
- [ ] 安装角纠偏测试
