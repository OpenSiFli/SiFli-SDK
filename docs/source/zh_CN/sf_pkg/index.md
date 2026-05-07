# SiFli 组件注册表

与 [RT-PKG](../app_note/rt-pkg.md) 类似，SiFli Package Registry 也是一个包管理器。但 SiFli Package Registry 底层采用 [Conan](https://conan.io/) 作为包管理工具，支持丰富的包管理功能，方便创建、上传和下载驱动包，并支持语义化版本管理。
- 点此访问 [SiFli组件注册表](https://packages.sifli.com/)
## 使用前准备

- 进入有效的 SDK 工程目录
- 执行 `.\export.ps1`（Windows）或 `. ./export.sh`（Linux/macOS）初始化环境

## 常用命令速查表

| 命令                                                   | 说明             |
| ------------------------------------------------------ | ---------------- |
| `sdk.py sf-pkg login -u <用户名> -t <令牌>`            | 登录 sf-pkg 系统并保存本地凭据 |
| `sdk.py sf-pkg users`                                  | 查看本地已登录用户 |
| `sdk.py sf-pkg use --name <用户名>`                    | 切换当前活跃用户 |
| `sdk.py sf-pkg current-user`                           | 查看当前活跃用户 |
| `sdk.py sf-pkg new --name <包名>`                      | 创建新的包配置   |
| `sdk.py sf-pkg build --version <版本号>`               | 构建包           |
| `sdk.py sf-pkg upload --name <包名>/<版本号>@<用户名>` | 上传包到服务器   |
| `sdk.py sf-pkg --user <用户名> upload --name ...`      | 临时指定本次命令使用的用户 |
| `sdk.py sf-pkg remove --name <包名>`                   | 清除本地缓存     |
| `sdk.py sf-pkg init`                                   | 初始化依赖配置   |
| `sdk.py sf-pkg install`                                | 安装依赖包       |
| `sdk.py sf-pkg search <包名>`                          | 搜索可用的包     |

```{toctree}
:titlesonly:

use
create
faq
```
