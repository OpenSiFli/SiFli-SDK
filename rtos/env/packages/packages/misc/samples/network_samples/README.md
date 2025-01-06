# network samples package

## 1、介绍

这个软件包包含一些网络操作的例程。

### 1.1例程说明

| 文件             | 说明                            |
| ---------------- | ------------------------------- |
| httpclient_sample.c       | 创建一个 http 客户端获取天气数据       |
| tcpclient_sample.c        | 创建一个 TCP 客户端             |
| tcpserver_sample.c        | 创建一个 TCP 服务端             |
| udpclient_sample.c        | 创建一个 UDP 客户端             |
| udpserver_sample.c        | 创建一个 UDP 服务端             |
| tcpclient_select_sample.c | 使用 select 接口实现 TCP 客户端 |

### 1.2 许可证

network samples package 遵循 Apache license v2.0 许可，详见 `LICENSE` 文件。

### 1.3 依赖

依赖网络组件。

## 2、如何打开 network samples package

使用 network samples package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    miscellaneous packages --->
        samples: kernel and components samples --->
            a network_samples package for rt-thread --->

```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、使用 network samples package

在打开 network samples package 后，当进行 BSP 编译时，选择的软件包相关源代码会被加入到 BSP 工程中进行编译。

## 4、注意事项

暂无。

## 5、联系方式 & 感谢

* 维护：[misonyo](https://github.com/misonyo)
* 主页：https://github.com/RT-Thread-packages/network-sample
