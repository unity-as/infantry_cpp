# CRC 校验

本模块用于当前 RoboMaster 通信，接口、初值、多项式和实现开关统一见 [crc.h](crc.h)，实际表和逐位运算见 [crc.cpp](crc.cpp)。调用方包括裁判、小电脑和 VT13。

两种算法都按 LSB-first 反射方式更新。CRC8 沿用 MAXIM 多项式但采用协议规定的初值，不能直接套用标准 MAXIM 的默认校验向量；CRC16 也不能与非反射的 CCITT-FALSE 混同。切换查表或逐位实现时需保持这些参数一致。

## 接口语义

- Calculate 的长度只覆盖待计算数据，不包含最后的校验字节。
- Verify 的长度包含校验尾部：CRC8 占最后一字节，CRC16 占最后两字节并按小端组合。
- 调用者必须保证缓冲和长度有效，并使用对应协议的起始位置与覆盖范围；CRC 通过不证明报文语义有效。

## 验证入口

从仓库根目录可运行 [crc8_gen.py](../../../tools/crc8_gen.py) 生成参考表：

```powershell
python tools/crc8_gen.py
```

生成表不是已经编译执行生产实现的证明。实际生产 CRC 与独立参考输入的覆盖见 [模块回归](../../../tools/tests/leg_sync/README.md)。本轮文档整理没有重新运行算法测试。
