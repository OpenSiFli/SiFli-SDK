# ACPU执行自定义任务
源码路径: `example/multicore/acpu_ctrl`
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ ec-lb583
+ ec-lb587


## 概述
<!-- 例程简介 -->
本例程展示了如何配置ACPU执行自定义任务

## 目录结构
- `project/hcpu`：HCPU的工程
- `project/acpu`：ACPU的工程
- `src/acpu`：ACPU的应用代码
- `src/hcpu`：HCPU的应用代码

### 编译和烧录
在`project/hcpu`目录下执行`scons --board=<board_name>`命令，编译生成所需板子的镜像文件，如执行`scons --board=ec-lb587`命令生成`587-evb`开发板的镜像文件。编译完成后运行命令`build_<board_name>\download.bat`烧写镜像文件，比如`build_ec-lb587\download.bat`


## 例程的预期结果

在串口console窗口发送`run_acpu <task_id>`命令（需要带回车符），其中`<task_id>`取值为大于等于0的数，依次对应TASK_0、TASK_1等等，运行结果如下
```
12-28 20:17:23:794    msh />
12-28 20:17:23:844    msh />
12-28 20:17:26:560 TX:run_acpu 0
12-28 20:17:26:732    run_acpu 0
12-28 20:17:26:772    [I/main] task_0
12-28 20:17:26:790    msh />
12-28 20:17:26:809    msh />
12-28 20:17:29:006 TX:run_acpu 1
12-28 20:17:29:149    run_acpu 1
12-28 20:17:29:160    [I/main] task_1
12-28 20:17:29:179    msh />
12-28 20:17:29:194    msh />
12-28 20:17:30:203 TX:run_acpu 2
12-28 20:17:30:332    run_acpu 2
12-28 20:17:30:347    [I/main] unknown task
12-28 20:17:30:358    msh />
12-28 20:17:30:366    msh />
12-28 20:17:31:285 TX:run_acpu 3
12-28 20:17:31:425    run_acpu 3
12-28 20:17:31:437    [I/main] unknown task
12-28 20:17:31:464    msh />
```

## 代码说明
`src/acpu/main.c`中的函数`acpu_main`为ACPU处理任务的入口函数，按收到的任务ID执行对应代码

`src/hcpu/main.c`中调用`acpu_run_task`函数配置ACPU执行某个任务，该函数以阻塞模式运行，直到ACPU返回结果后函数才会返回，这期间调用该函数的线程会因等信号量而被挂起

ACPU的镜像文件存放在Flash中，由烧写脚本烧录到Flash，二级boot将ACPU的代码拷贝到ACPU指令空间0地址对应的RAM中，
例如下面的代码摘取自编译生成ftab.c，`.base=0x69100000`表示ACPU的镜像文件存放在0x69100000地址开始的Flash中，`xip_base=0x20200000`表示二级boot会将ACPU的代码拷贝到0x20200000开始的RAM中，0x20200000对应ACPU指令空间的0地址。

```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```

二级boot拷贝ACPU代码的实现见`example\boot_loader\project\sf32lb58x_v2\board\main.c`中函数`boot_images`如下部分：
```c
if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```



## 异常诊断


## 参考文档

## 更新记录
