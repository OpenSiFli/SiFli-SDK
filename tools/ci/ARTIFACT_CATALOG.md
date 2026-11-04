# SDK 产物查找入口

流水线完成后，在任务很少的**父流水线**打开 `artifact_catalog` 任务的
**Browse artifacts → artifact_catalog/**，不需要等待几千个任务的子流水线图加载。
子流水线同名任务也保留目录入口。
下载 `index.html` 后用浏览器打开，选择板子、搜索工程或文件名，展开文件列表即可下载单个文件；
状态选“失败”可集中查看失败目标和日志。搜索支持多个空格分隔的关键词，文件列表保留子工程路径。
页面不需要用户提供 API token；下载私有项目的文件时，浏览器需要登录 GitLab。

GitLab 开启 Pages 及私有项目访问控制时，可以在线预览 HTML artifact。
未开启时仍可下载目录页使用，也可以在 GitLab 中查看 `README.md` 的完整目标清单。
这次改动不启用或修改 Pages 配置。

## 快速验证，不编译固件

代码进入 GitLab 的待测分支后，在 **Build → Pipelines → Run pipeline** 选择该分支，
添加变量 `CATALOG_SMOKE_TEST=1` 后运行。不需要设置 `BUILD_SDK`；
不要启用 `SYNC_GITHUB` 或 `MIRROR_EXTERNAL`，它们是其他独立任务的开关。

此模式用独立配置 `tools/ci/catalog_smoke.yaml` 创建三个模拟目标：两个成功，一个预期失败。
只生成数百字节的 `SMOKE_TEST_ONLY.txt`、日志和状态，不调用编译器、不拉取子模块。
预期失败的模拟编译任务允许失败；收集、上传、目录及父流水线入口仍严格检查错误。
目录显示“快速验证数据”，测试文件不含可烧录固件。

它会实际验证 Generic Package 指针读写、单文件 artifact API 权限、目录生成和父流水线入口。
在父流水线打开目录后，应能看到两个可下载的测试文件、一个失败目标和三个日志入口。
实际点击下载并核对测试文件，可完成浏览器鉴权和下载链接验证。
总耗时取决于 Runner 排队、仓库检出和上传，通常远短于完整编译，不承诺固定分钟数。

本地也可先检查 YAML 和目录逻辑：

```sh
python3 tools/ci/dynamic_pipeline.py --smoke-test --dry-run
python3 -m unittest discover -s tests -p 'test_dynamic_pipeline.py'
python3 -m unittest discover -s tests -p 'test_artifact_catalog.py'
```

本地检查依赖 PyYAML。它不能代替真实 CI 对服务器权限和 artifact 上传的验证。
正常构建不设置 `CATALOG_SMOKE_TEST=1`，仍使用完整 SDK 配置。

## 实现

- 编译和批次收集方式不变。每个 collector 仍依赖最多 50 个编译任务。
- collector 向本项目 Generic Package Registry 写入一个数百字节的指针，记录本次
  project、pipeline、commit、batch 和 collector job ID。版本号是子流水线 ID。
- `artifact_catalog` 在 collect 阶段完成后运行，`dependencies: []` 避免自动下载全部固件。
  它读取指针，再通过 job ID 的单文件 API 读取 `merged_artifacts/file_index.json`。
  另外只查询当前流水线的 job 列表，以识别 collector 重试后的最新 job ID 和状态；
  不会列举/下载其他流水线任务的 artifact。读取任务列表或单文件 API 返回 401/403/404 时，
  使用 `GITLAB_API_READ_TOKEN` 备用认证。
- 获取索引发生在批次 artifact 上传结束之后。上传失败、指针缺失、索引身份不符或目标缺失都会显示为异常；
  目录仍上传，但任务失败以提示目录不完整。编译失败本身只记录失败状态，不阻止目录生成。
- 所有下载链接绑定具体 collector job ID，文件路径经过转义。不会回退到某分支的最后一次成功构建。
- collector 启动时先发布本次尝试的 job ID；目录生成时再次核对指针是否对应当前最新且成功的 collector。
  因重试失败或上传失败而遗留的旧指针不会被当作本次结果。
- 子流水线将目录任务指针写入父流水线 ID 对应的包版本。父流水线等待子流水线结束后，
  只读取小型 `catalog.json` 并生成同一份 HTML 入口，不下载固件。入口通过 GitLab trigger/bridge
  记录核对当前关联的子流水线 ID，再校验目录指针、项目和 commit 身份；重试后旧指针不会被接受。
  查询 trigger/bridge 优先使用 `CI_JOB_TOKEN`；若服务器拒绝该 API，则使用 `GITLAB_API_READ_TOKEN`。
- 冒烟目录会严格核验三个目标、两份小型测试文件和三个日志，并确认预期失败目标没有产物；
  任一项缺失或状态不符都会使目录任务失败。
- 目录及分组 artifact 保留一周。链接不会在过期后指向其他构建；下载时 GitLab 可能提示文件不存在。
  目录页生成时确认文件清单可读，页面打开后不会实时探测文件是否已过期或被手动删除。

## 服务配置与保留策略

需要启用本项目 Package Registry，且流水线的 `CI_JOB_TOKEN` 能上传/下载 Generic Packages
以及读取同项目的 job artifacts。SDK 项目目前已启用 Package Registry。
如服务器不允许 `CI_JOB_TOKEN` 读取任务列表、Pipelines Bridges API 或单个 job artifact，则需配置 masked CI/CD 变量
`GITLAB_API_READ_TOKEN`，其只读 API token 至少具有 `read_api` 权限。可将变量限制在受信任的分支，
并在测试分支受保护时设为 protected。若两种认证均不可用，父目录入口会明确失败，不会回退显示旧目录。
这些权限需要在真实 CI 中验证；目录生成失败时查看任务日志中的 HTTP 状态码。
在 Run pipeline 页面临时填写的 `GITLAB_API_READ_TOKEN` 也会显式传给子流水线，
不依赖 GitLab 默认不转发的 pipeline variables；其他父流水线变量仍按原有继承规则处理。
重试 collector 会再次发布同流水线、同文件名的指针，Generic Packages 必须允许该包重复文件名
（GitLab 默认允许）；下载端使用最新指针，仍校验流水线和 commit。

包名为 `sdk-artifact-catalog-pointers`。包中只有指针，不保存固件、日志、文件清单或凭证。
这些指针不受 artifact 的 `expire_in` 控制，会留在包仓库；可按流水线保留期定期清理旧版本。
清理指针不会影响已经生成的 HTML/JSON 目录及其下载链接，但之后重试旧流水线的目录任务将无法重新读取指针。

参考 GitLab 官方文档：

- <https://docs.gitlab.com/user/packages/generic_packages/>
- <https://docs.gitlab.com/api/job_artifacts/>
- <https://docs.gitlab.com/ci/jobs/job_artifacts/>
