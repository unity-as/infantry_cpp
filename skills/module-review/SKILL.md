---
name: module-review
description: "模块入库审查规范。Use when: 新模块进 DRIVER/kit、审查已有模块是否符合框架规范、同步模块 README、检查文档与目录漂移。用户话术触发：\"审查模块\"、\"入库\"、\"README 同步\"、\"检查文档\"、\"这个模块合格吗\"。"
---

# module-review —— 模块入库审查

> 审查 = 拿 c-coding-standards 和 module-writing 逐条对照，不是凭感觉。
> 入库是写操作，走 change-discipline 的确认流程。

## 1. 审查依据

- c-coding-standards：命名/头文件/内存/静态表
- module-writing：四类模板、Register 六步曲、状态归模块

## 2. 审查清单

- 功能：接口完整、Register 六步曲齐全、回调与 owner 还原正确
- 依赖：分层正确（bsp=外设、modules=设备）、依赖方向不反向
- 安全：注册≠启动、危险外设默认不输出、daemon 按类选配
- 命名：按 c-coding-standards 命名规则表逐项核对
- 复用：有没有重复实现（两份 mahony / 两份 bmi088 的历史教训）
- 文档：README 有接口/依赖/接线/已知坑
- 封装：模块 config 不得暴露 bsp 结构（如 PWM_Init_Config_s）；
  bsp 注册细节封装在模块内部（servo 模块教训）

## 3. 审查报告先行

- 先给用户问题清单，按严重度分级：严重 / 中等 / 建议
- 用户确认后才允许入库

## 4. 入库动作

- 复制到 DRIVER 对应目录（bsp/modules/examples）
- 更新 README 模块地图
- 更新模块清单
- 更新 ONBOARDING 索引

## 5. 文档漂移检查

- 目录 vs README 对比（可配脚本自动做）
- 发现漂移立即修 README，禁止留"文档过期"

## 6. 问题处理

- 严重问题：退回修复（走 code-modification）
- 小问题：可入库后跟进，但要记录

## 7. kit 的特殊性

- kit 是共享库，入库影响所有引用它的项目：审查更严格，宁缺毋滥
- 能标记 deprecated 就不删

## 引用

- 审查标准 → c-coding-standards、module-writing
- 行为流程 → change-discipline（入库确认）
- 问题修复 → code-modification
