# 调试启动

配置在 `.vscode/settings.json`（`flash.*`）。技能：`.claude/skills/build-flash-debug/`。

## 编译

```
cmake --build build
```

固件：`build/infantry.elf`。

## 烧录（当前 debugger = dap）

```
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program build/infantry.elf verify reset exit"
```

ST-Link 则改用 `interface/stlink.cfg`。芯片 target：`target/stm32f4x.cfg`。

## 调试

Ozone：`Ozone.exe` + 仓库根目录 `debug.jdebug`。VSCode 可 F5。
