# RT-PKG 命令使用指南

## 环境配置

在SDK路径下使用 `.\export.ps1` 配置环境后，切换到需要配置packages的工程目录，使用以下命令进入配置界面：

```shell
sdk.py menuconfig --board=sf32lb52-lcd_n16r8
```

进入配置界面后，选择 `RT-Thread online packages` 来配置相关的软件包。

**注意**：在首次使用时需要先执行 `sdk.py rt-pkg-upgrade` 下载更新软件包相关配置。

![](../../assets/rt_pkg_option_packages.png)

## 命令列表

### 查看帮助信息

```shell
sdk.py --help
```

该命令可以查询rt-pkg相关命令的使用说明。

**注意**：使用该命令后，若Commands下无rt-pkg相关命令请更新本地SDK。

![](../../assets/rt_pkg_sdk.py--help.png)

### 列出已配置的软件包

```shell
sdk.py rt-pkg-list
```

在 `menuconfig` 中添加了新的软件包后，使用该命令可以输出已经添加的软件包。

**使用示例**：
1. 在 `menuconfig` 中选择 `RT-Thread online packages`
2. 进入 `multimedia packages`
3. 按下空格选择 Openmv
4. 保存退出

![](../../assets/rt_pkg_sdk.py_rt-pkg-list_1.png)

使用该命令可以看到已经添加的软件包以及软件包版本：

![](../../assets/rt_pkg_sdk.py_rt-pkg-list_2.png)

### 更新软件包配置

```shell
sdk.py rt-pkg-update
```

`menuconfig` 中添加了新的软件包后，使用该命令下载相应软件包。

**添加操作**：
![](../../assets/sdk.py_rt-pkg-update_add.png)

**删除操作**：在menuconfig中取消软件包选中后，使用该命令输入Y即可删除对应的软件包。

![](../../assets/rt_pkg_sdk.py_rt-pkg-update_delete.png)

### 打印环境变量

```shell
sdk.py rt-pkg-printenv
```

该命令可以打印环境变量以进行检查。

![](../../assets/sdk.py_rt-pkg-printenv.png)

### 升级软件包

```shell
sdk.py rt-pkg-upgrade
```

使用该命令可以对已经配置的软件包进行升级。在首次使用rt-pkg时需要先使用该命令配置软件包。

![](../../assets/rt-pkg-upgrade.png)

### 升级Python模块

```shell
sdk.py rt-pkg-upgrade-modules
```

使用该命令对Python模块对象进行升级（例如requests）。

### 创建新软件包

```shell
sdk.py rt-pkg-wizard
```

使用该命令可以使用wizard创建新的软件包。

## 使用流程建议
1. 首先使用 `sdk.py --help` 查看可用命令
2. 使用 `sdk.py rt-pkg-list` 查看当前配置
3. 通过menuconfig配置需要的软件包
4. 使用 `sdk.py rt-pkg-update` 更新配置 整理一下格式
