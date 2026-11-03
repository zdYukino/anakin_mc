# anakin_mc

原生 C++ ONNX 模型控制模块，独立 ROS 2 Humble ament 包。MC 保留原模型、历史、动作缩放与过渡计算，只通过 `anakin_sdk` 发布最终控制目标；低层写入和实机权限由独立 Env 管理。

先安装 [anakin_sdk.revision](anakin_sdk.revision) 锁定的 SDK，再构建：

```sh
ANAKIN_SDK_PREFIX=/path/to/sdk/install/anakin_sdk ./build.sh
./en_run.sh
```

也可把 SDK / MC / Env 三个独立仓库链接到 `../anakin_cpp_ws/src` 后运行工作区 `build.sh`。MC 需要官方原生 ONNX Runtime 1.20.1；`ONNXRUNTIME_ROOT` 可指定其目录。默认采用相邻工作区的 SDK、ORT 和原有 ONNX 模型；`ANAKIN_LOCO_MODEL`、`ANAKIN_MIMIC_MODEL` 可明确指定模型文件。

MC 与 Env、官方模拟器分别启动，长期等待新数据；没有固定时长退出，也不会启动/重置模拟器。MC 的实际入口是本仓库 `en_run.sh`，没有工作区 `mc_run.sh`。完整三终端操作、模式/键位、YAML、实机显式网卡和用户验收说明见 [Env 独立运行说明](../anakin_env/docs/standalone-operation.md)。默认 ROS domain 87，发现限制本机；自定义 Env `transport.ros_domain` 后需在 MC 启动前设置相同 `ROS_DOMAIN_ID`。

启动器优先本仓库 `install/anakin_mc`，不存在时选择共享工作区安装；`ANAKIN_PREFIX` 显式选择当前 MC 前缀，`ANAKIN_SDK_PREFIX` 指定 SDK。存在但不完整的独立安装会报错，不回退其他 overlay。启动前检查实际 ELF、本地包标记和 SDK 构建记录；只执行 C++ ELF，不构建、不安装依赖、不运行 Python 启动脚本。

对缺失、过期、重复或不允许推理的状态帧，MC 保留模型/历史/时间线并等待，不发布旧目标。Env 的 Preparing 可允许当前事务预热；这不是电机授权。真实 SDK 写入及其实机新鲜度/停止期限仍由 Env 管理。详见 [SDK 合同](../anakin_sdk/docs/contract.md)。

通信无关的软件检查：

```sh
ctest --test-dir build/anakin_mc -L offline --output-on-failure --no-tests=error
```

[历史 native checkpoint](docs/native-checkpoint.md) 保留旧结果，其旧 loopback 入口/机械参考限制不是当前运行合同。当前软件检查不代表实际仿真策略稳定性、实机增益或物理执行已验收。SDK 若有未提交源代码差异，HEAD-only 构建记录无法标识这份差异的新版本；应成套重建，SDK 提交后再更新两个消费者锁并重建。
