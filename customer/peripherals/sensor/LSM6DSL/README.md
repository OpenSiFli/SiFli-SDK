# LSM6DSL

## 简介

本软件包是为 ST LSM6DSL 加速度和陀螺仪 6 轴传感器提供的通用传感器驱动包。通过使用此软件包，开发者可以快速的利用 RT-Thread 将此传感器驱动起来。

本篇文档主要内容如下：

- 传感器介绍
- 支持情况
- 使用说明

## 传感器介绍

LSM6DSL 是 ST（意法半导体）公司专为可穿戴设备和 IOT 市场开发的一款超低功耗传感器，尺寸小巧且内置计步器。该模块在组合正常模式下功耗为 0.4mA，在组合高性能模式下功耗为 0.65mA。

## 支持情况

| 包含设备         | 加速度计 | 陀螺仪 | 计步计 |
| ---------------- | -------- | ------ | ------ |
| **通讯接口**     |          |        |        |
| IIC              | √        | √      | √      |
| SPI              |          |        |        |
| **工作模式**     |          |        |        |
| 轮询             | √        | √      | √      |
| 中断             |          |        |        |
| FIFO             |          |        |        |
| **电源模式**     |          |        |        |
| 掉电             | √        | √      | √      |
| 低功耗           |          |        |        |
| 普通             | √        | √      | √      |
| 高功耗           | √        | √      | √      |
| **数据输出速率** | √        | √      |        |
| **测量范围**     | √        | √      |        |
| **自检**         |          |        |        |
| **多实例**       |          |        |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：LSM6DSL 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动框架支持；
- PIN 驱动：用于处理设备中断引脚；

### 获取软件包

使用 LSM6DSL 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
/* TODO:完善路径 */
LSM6DSL: LSM6DSL Universal Sensor Drive
    [*]   Enable lsm6dsl acce
    [*]   Enable lsm6dsl gyro
    [*]   Enable lsm6dsl step 
        Version (latest)  --->
```

**Enable lsm6dsl acce**： 配置开启加速度计功能

**Enable lsm6dsl gyro**： 配置开启陀螺仪功能

**Enable lsm6dsl step**：配置开启计步计功能

**Version**：软件包版本选择

### 使用软件包

LSM6DSL 软件包初始化函数如下所示：

```
int rt_hw_lsm6dsl_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 LSM6DSL 设备的注册；

#### 初始化示例

```
#include "sensor_st_lsm6dsl.h"

int lsm6dsl_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)LSM6DSL_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_lsm6dsl_init("lsm6dsl", &cfg);
    return 0;
}
INIT_APP_EXPORT(lsm6dsl_port);
```

## 注意事项

暂无

## 联系人信息

维护人:

- [guozhanxin](https://github.com/Guozhanxin) 
