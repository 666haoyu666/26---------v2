# platform_common 设计说明

本目录从第 4 课的平台公共基础层继续演进。第 4 课建立类型、错误码和公共宏；第 5 课在这个基础上建立 object / device / service 的轻量对象模型，让后续 Platform、Service、App 都能使用同一套对象身份和生命周期规则。

## 本课落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_type.h` | 从 `04_Impl/impl_board/board_types.h` 引出平台统一基础类型 |
| `platform_error.h` | 定义 `platform_err_t` 和统一的成功/失败判断宏 |
| `platform_def.h` | 收纳通用状态、布尔、NULL、对齐、数组长度和延时声明 |
| `platform_object.h/.c` | 定义所有平台对象共有的 magic、name、type、state 和基础校验 |
| `platform_lifecycle.h` | 定义 init/start/process/stop/deinit 生命周期回调入口 |
| `platform_device.h/.c` | 定义设备对象基类，继承 `platform_object_t` 并扩展设备类别、运行能力、功耗状态和生命周期入口 |
| `platform_service.h/.c` | 定义服务对象基类，继承 `platform_object_t` 并扩展服务类别和生命周期入口 |
| `README.md` | 说明本目录的边界、依赖方向和课堂验收点 |

## 依赖方向

```text
04_Impl/impl_board/board_types.h
        |
        v
03_Platform/platform_common/platform_type.h
        |
        +--> platform_error.h
        |
        +--> platform_def.h
        |
        +--> platform_object.h
              |
              +--> platform_device.h
              |
              +--> platform_service.h
              |
              +--> platform_lifecycle.h
```

`board_types.h` 是当前板级/当前编译环境的基础类型源头。`platform_type.h` 是给 Platform、Service、App 使用的类型出口。上层代码不建议直接 include `board_types.h`。

## 课堂验收

学生应能解释第 4 课基础内容：

1. 为什么 `board_types.h` 放在 `04_Impl/impl_board`。
2. 为什么 `platform_type.h` 要作为 Platform 层类型出口。
3. 为什么接口返回值优先使用 `platform_err_t`。
4. `PLATFORM_IS_OK()` 和 `PLATFORM_IS_ERR()` 如何统一判断错误。
5. `PLATFORM_ALIGN()` 和 `ARRAY_SIZE()` 的正确使用边界。
6. 为什么 `platform_delay_ms()` / `platform_delay_us()` 只在这里声明，实现应放到 Impl 层。

学生还应能解释第 5 课对象模型：

1. `platform_object_t` 为什么要有 `magic` / `name` / `type` / `state`。
2. 为什么 `platform_object_is_valid()` 要同时检查 magic 和 type。
3. `PLATFORM_OBJECT_DEVICE` 和 `PLATFORM_DEVICE_CLASS_IMU` 的区别。
4. 为什么 `platform_device_t` 与 `platform_service_t` 的第一个字段都必须是 `platform_object_t object`。
5. 为什么 state 只记录对象阶段，真正动作入口放在 `platform_lifecycle_ops_t`。
6. 为什么 `caps` 表示静态运行能力，而 `power_state` 表示当前功耗状态。
7. 为什么第 5 课不把 `cfg` / `ctx` / `data` / `ops` 放进通用基类里提前讲。

## 第 5 课边界

本课只回答“平台对象如何被统一管理”的问题：

```text
object      解决对象是谁、是否合法、处于什么状态
device      解决硬件设备如何进入统一管理面
service     解决业务服务如何进入统一管理面
lifecycle   解决对象如何被统一启动、停止和运行
caps        解决系统策略如何识别设备支持的运行能力
power_state 解决系统策略如何记录设备当前功耗状态
```

本课暂不回答“某个具体设备到底怎么读写”的问题。显示屏、触摸、IMU、电池、Flash、背光的具体动作差异很大，不强行塞进统一的 `read/write/control`。这些具体设备操作表会在后续课程里作为 typed ops 展开，例如 `display_ops_t`、`touch_ops_t`、`storage_ops_t`。

本课中的 `caps` 不再沿用传统 `READ/WRITE/CONTROL` 的设备文件思路，而是描述智能手表整机策略会关心的静态能力，例如是否支持休眠、深度休眠、断电、作为唤醒源、掉电后重初始化、降频、降采样和亮度调节。真正“当前是否处于休眠状态”，由 `power_state` 记录。

因此，第 5 课里的 `platform_device_t` 是设备的公共管理面，不是所有设备的完整行为接口。

## 注意点

`platform_error.h` 不再同时使用同名 `#define` 和 enum 枚举值。错误码只保留 enum 定义，避免预处理阶段把枚举名替换成数字后导致编译错误。

`int64` / `uint64` 在 STM32/Keil 这类 32 位环境下应使用 `long long`，不能写成 `long`，否则很可能只有 32 位。
