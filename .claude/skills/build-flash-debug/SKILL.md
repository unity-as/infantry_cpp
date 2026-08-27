---
name: build-flash-debug
description: 构建、烧录、调试及配置烧录参数。触发词：编译、烧录、调试、配置烧录、换芯片。
triggers: [编译, 构建, build, 烧录, flash, 调试, debug, 配置烧录, 换芯片, 改烧录器]
---

# Build / Flash / Debug

**配置来源**：`.vscode/settings.json`（key: `flash.*`、`debug.tool`），缺失则交互询问并写入。

---

## 1. Build
命令：`cmake --build ${config:flash.build_dir}`（默认 `build`）

## 2. Flash
1. 读取 `flash.debugger`（`stlink`/`dap`/`custom`）选择 interface 配置文件
2. 固件不存在时自动先 Build
3. 执行：
```bash
openocd -f ${config:flash.interface.${config:flash.debugger}} -f ${config:flash.target_cfg} -c "program ${config:flash.build_dir}/${config:flash.firmware_file} verify reset exit"
```

## 3. Debug
读取 `debug.tool`（`ozone`/`gdb`/`vscode`）启动：
- **Ozone**：`Ozone.exe ${workspaceFolder}/debug.jdebug`
- **GDB**：`openocd -f ${config:flash.interface.${config:flash.debugger}} -f ${config:flash.target_cfg} & arm-none-eabi-gdb ${config:flash.build_dir}/${config:flash.firmware_file} -ex "target remote localhost:3333"`
- **VSCode**：提示按 `F5`

## 4. Config
交互修改 `flash.debugger`、`flash.target_cfg`，根据芯片型号自动匹配 target 配置文件：
- STM32F1 → `target/stm32f1x.cfg`
- STM32F4 → `target/stm32f4x.cfg`
- STM32F7 → `target/stm32f7x.cfg`
- STM32H7 → `target/stm32h7x.cfg`
- STM32G4 → `target/stm32g4x.cfg`

结果写入 `.vscode/settings.json`

## 组合执行

用户说 | AI 执行
---
“编译” | Build
“烧录” | Build + Flash
“调试” | Build + Debug
“配置烧录” / “换芯片” | Config
“烧录并调试” | Build + Flash + Debug
