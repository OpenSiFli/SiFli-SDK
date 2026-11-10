# SDK 产物查找入口

**固定入口：`http://10.21.10.172:8091`**（公司内网或 VPN）。打开后自动进入 `main`
分支最近一次完整构建的产物目录，首次访问需登录具有项目访问权限的 GitLab 账号。
该页面由父流水线的 `pages` 任务发布，每次 `main` 分支的完整构建（schedule/web/api 且设置
`BUILD_SDK`）完成后自动更新；MR 和其他分支的流水线不会替换它。
只有完整的目录才会发布：编译失败只是目录中的失败状态，不影响发布（编译失败会使子流水线失败，
因此 `pages` 任务设为 `when: always`，由脚本通过 API 核验本流水线父 `artifact_catalog` 任务成功）；
目录本身有异常（collector 索引缺失、上传失败、权限错误等）或父目录任务失败/取消时，
`pages` 任务失败且不生成 `public/`，保留原入口，不会发布失败任务遗留的目录文件。
发布任务通过 `resource_group` 串行执行，发布前检查是否已有更新的 `main` 流水线**实际部署成功**：
依据 GitLab 在每次 `pages` 任务成功后异步生成的 `pages:deploy` 状态，而不是 `pages` 任务状态；
包含重试前的历史尝试，部署成功后再重试失败也算已发布，任务成功但部署失败不算。
更新的流水线部署仍在进行（或部署状态尚未生成）时，最多等待 10 分钟，超时则任务失败、不发布。
较早的流水线晚完成时会跳过发布（任务以退出码 3 显示为允许失败），不会让入口回退；
GitLab 自身也会拒绝提交 SHA 已过期且已有更新流水线部署的发布。
服务器 nginx 将 `8091` 的根路径重定向到 Pages 站点 `/sw/SiFli-SDK/`。
页面中的下载链接仍指向对应构建任务，产物过期（通常一周）后链接失效，需等待下一次构建。
固定入口同时汇总 `main` 和 `release/v2.5` 最近 3 天内的完整构建。页面顶部可切换分支，
再按北京时间选择某次构建；日期相同的构建仍分别列出。历史只保存在当前 Pages 发布物中，
每次发布都会从 GitLab API 重新读取小型目录，不会下载固件；候选流水线的目录查询使用最多 4 个并发请求。
读取单条历史失败只跳过该条，
历史流水线列表 API 失败则本次 Pages 发布失败并保留原入口。构建日期按流水线创建时间计算，
超过 3 天后从选择器和分享链接中移除。分享链接支持 `?ref=main&date=2026-11-02`，
也支持 `?pipeline=<流水线 ID>`。
固定入口始终显示最新的 `main` 完整构建，不会用 `release/v2.5` 替代。若最近 3 天没有有效的
`main` 目录，即使存在有效的 `release/v2.5` 目录，Pages 发布也会失败并保留原入口；待有新的
`main` 完整构建后再更新。冒烟发布也不会绕过此保护：只有完全没有有效历史时，才显示模拟目录；
若仅有其他分支的有效历史，冒烟发布同样失败并保留原入口。
若重试的是超过 3 天的构建且没有其他有效历史，发布会失败并保留原入口，不会重新展示可能过期的下载链接。

查看某一次特定流水线时，在任务很少的**父流水线**打开 `artifact_catalog` 任务的
**Browse artifacts → artifact_catalog/**，不需要等待几千个任务的子流水线图加载。
子流水线同名任务也保留目录入口。
点击 `index.html` 在线预览，选择板子、搜索工程或文件名，展开文件列表即可下载单个文件；
状态选“失败”可集中查看失败目标和日志。搜索支持多个空格分隔的关键词，文件列表保留子工程路径。
页面不需要用户提供 API token；下载私有项目的文件时，浏览器需要登录 GitLab。

在线预览使用内网 GitLab Pages `http://10.21.10.172:8091`，需要连接公司内网或 VPN，
并登录具有项目访问权限的 GitLab 账号。点击 `index.html` 后，GitLab 会显示外部页面提示，
点击提示中的内网链接即可打开。预览地址绑定具体任务，例如：
`http://10.21.10.172:8091/sw/-/SiFli-SDK/-/jobs/<job_id>/artifacts/artifact_catalog/index.html`。
外网无法访问预览入口时，仍可下载 `index.html` 本地打开，
也可以在 GitLab 中查看 `README.md` 的完整目标清单。

## 快速验证，不编译固件

代码进入 GitLab 的待测分支后，在 **Build → Pipelines → Run pipeline** 选择该分支，
添加变量 `CATALOG_SMOKE_TEST=1` 后运行。不需要设置 `BUILD_SDK`；
不要启用 `SYNC_GITHUB` 或 `MIRROR_EXTERNAL`，它们是其他独立任务的开关。

此模式用独立配置 `tools/ci/catalog_smoke.yaml` 创建三个模拟目标：两个成功，一个预期失败。
只生成数百字节的 `SMOKE_TEST_ONLY.txt`、日志和状态，不调用编译器、不拉取子模块。
预期失败的模拟编译任务允许失败；收集、上传、目录及父流水线入口仍严格检查错误。
目录显示“快速验证数据”，测试文件不含可烧录固件。
快速验证默认不更新固定入口；需要验证 Pages 发布时，在 `main` 分支运行并额外设置
`CATALOG_PUBLISH=1`。模拟数据不会进入构建历史：最近 3 天已有完整构建时，入口仍按上述规则显示可用的真实构建，
可用来验证历史汇总和发布流程；没有任何历史时，入口才会暂时显示模拟数据，直到下一次完整构建覆盖。

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
如服务器不允许 `CI_JOB_TOKEN` 读取任务列表、Pipelines Bridges API 或单个 job artifact，
以及 `pages` 任务使用的流水线列表和提交状态（commit statuses）API，则需配置 masked CI/CD 变量
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
