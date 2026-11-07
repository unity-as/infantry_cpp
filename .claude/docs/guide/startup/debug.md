# 开发启动

从仓库根目录执行。整车操作见 [整车启动](production.md)，源码与板端状态的区别见 [验证记录](../../../records/validation.md)。本文命令按当前构建文件核对，文档整改没有重跑构建或烧录。

## 首次配置与编译

准备支持 [CMakePresets.json](../../../../CMakePresets.json) 中 preset 格式的 CMake、Ninja、含 C++20 支持的 Arm GNU Toolchain。`arm-none-eabi-gcc/g++/objcopy/size` 需要在 PATH 中；工具链规则见 [gcc-arm-none-eabi.cmake](../../../../cmake/gcc-arm-none-eabi.cmake)。

```powershell
cmake --version
ninja --version
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version
cmake --preset default
cmake --build --preset default
```

default preset 在 build 下生成 Ninja 工程并交叉编译 ARM 固件。仓库中的桌面 Clang preset 含个人路径，不作为本项目固件的通用启动入口。已配置后可直接 `cmake --build build`。

产物为 build/infantry.elf、infantry.hex、infantry.bin。换检出路径或工具链后应重新配置；旧构建缓存与当前源码不能混用。

本项目 [工具链文件](../../../../cmake/gcc-arm-none-eabi.cmake)引用的链接脚本名大小写与仓库文件存在差异，Windows 通常不敏感；本指南未验证 Linux 构建，移植环境前应核对该项。此轮没有修改构建配置。

## 烧录

准备 OpenOCD 和对应探针驱动，连接目标板与 CMSIS-DAP 后执行：

```powershell
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program build/infantry.elf verify reset exit"
```

使用 ST-Link 时替换 interface 为 interface/stlink.cfg。烧录前核对 ELF 对应的源码版本和初始化开关；记录目标、产物和 verify 结果。verify 只证明写入校验，实车行为按 [整车启动](production.md)检查。

## 调试与编辑器

Ozone 打开根目录 [debug.jdebug](../../../../debug.jdebug)，脚本加载 build/infantry.elf。现有脚本含固定 USB 探针序列号与旧源码路径替换，换机器或探针时需先核对；菜单或任务入口不保证这些个人设置适用于新环境。

[VS Code tasks](../../../../.vscode/tasks.json) 提供 build、flash 和 Ozone 任务，但 flash 依赖本机 `.vscode/settings.json` 的 `flash.*`，该文件被 Git 忽略；Ozone 任务也含本机绝对路径。首次检出优先使用上文明确命令。本仓库没有随附 launch.json，不能把 F5 当作已配置的通用调试入口。

需要配置编辑器任务时，按自己的安装位置填写 `flash.build_dir`、`flash.firmware_file`、`flash.target_cfg`、探针 interface 和 Ozone 路径；不要把个人路径当作团队默认环境。

## 按改动选择验证

主机回归命令和覆盖范围只维护在 [leg_sync](../../../../tools/tests/leg_sync/README.md)。接口、计时或驱动变更需按涉及模块选择检查，实车控制变更仍需 ARM 构建及对应实机验证。仅修改文档时检查源码依据、链接、编码与改动范围即可。
