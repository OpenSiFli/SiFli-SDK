# YAFFS filesystem for RT-Thread MTD NAND

## 1、介绍

### 1.1 目录结构

| 名称 | 说明 |
| ---- | ---- |
| . | OS porting glue. |
| yaffs | yaffs sources & header |

### 1.2 许可证

YAFFS 遵循 GPL，详见 YAFFS 官方網站。

### 1.3 依赖

- RT-Thread 4.1.1+

## 2、如何下載源碼包

使用 YAFFS2 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    system packages --->
        [*] yaffs: Yet Another Flash File System  --->
		    yaffs version (latest)  --->
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、使用 YAFFS package

無

## 4、注意事项

無

## 5、联系方式

無