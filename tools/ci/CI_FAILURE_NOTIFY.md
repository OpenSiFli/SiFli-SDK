# CI 失败企业微信通知

SDK 的完整构建失败时，向企业微信群机器人推送一条消息：列出失败的任务，并附上每个任务日志里的第一条报错、任务链接和产物目录链接。
服务部署在内网服务器上，接收 GitLab 的 Pipeline webhook。脚本是 `tools/ci/ci_failure_notify.py`，
只用 Python 标准库，需要 Python 3.8 及以上。

## 什么时候推送

同时满足以下条件才推送：

- 顶层流水线状态变为 `failed`。子流水线（`parent_pipeline`）的事件不处理，失败信息由顶层流水线统一汇总。
- 分支是 `main` 或 `release/v2.5`（可用 `NOTIFY_REFS` 修改），不是 tag。
- 触发方式是 schedule、web 或 api。MR 和 push 流水线不推送。
- 流水线包含 `trigger_child_pipeline`，即完整 SDK 构建。同步、镜像等流水线不推送。
- 没有设置 `CATALOG_SMOKE_TEST=1`（冒烟验证不推送）。

每个项目的流水线 ID 会永久保存在状态文件中，避免较早的重复 webhook 再次推送。状态文件损坏或被删除后，历史去重记录会丢失。

服务先把 webhook payload 写入 SQLite 持久队列，再回复 200 并在后台处理。队列最多保存 256 个待处理事件、1024 条总记录（包含待人工核对项）；任一上限触发或写入失败时返回 503，不确认接收。
处理失败（企业微信不可达、GitLab API 临时出错等）时分别在 1、5、15 分钟后重试，尝试次数和下次执行时间保存在数据库中，服务重启后继续处理。
三次重试都失败后，事件保留为待人工处理，日志会打印补发命令。处理中记录使用一小时租约；服务启动或取队列时发现租约过期，会将其标为 `uncertain`，不会自动重发。多个服务进程共用队列时，事件领取由 SQLite 事务串行化。
每次处理前都会向 GitLab 查询流水线的当前状态，期间已被重跑（不再是 `failed`）的流水线不推送。
读取子流水线时遇到 403/404 会把 bridge 本身列为失败项；遇到 5xx 或连接失败则整条事件交给重试，
不会发出缺少真实失败任务的消息。状态文件分别记录 `pending` 和 `sent` 流水线 ID。发送前先原子记录为 `pending`，收到成功响应后改为 `sent`；明确拒绝时移除记录并重试。
网络超时、连接中断或进程在发送窗口退出时，队列和状态会保留为 `uncertain` / `pending`，避免自动重复推送。
WeCom 与本地数据库之间没有事务或幂等键，无法保证严格的恰好一次投递。遇到 `uncertain` 时先检查群消息；确认未送达后再执行手动补发。

## 消息内容

- 分支、流水线链接、触发方式、提交 SHA 和标题、失败任务数。
- 失败任务列表：从顶层流水线沿 bridge 向下查找子流水线中的失败任务。目前 `main` 和 `release/v2.5` 都是
  "父 → 子"一层：`main` 子流水线里的批次是 `parallel:matrix` 任务，名字形如 `build_batch_001: [example_xxx]`，
  消息里会把方括号显示为圆括号，避免破坏企业微信的链接格式。
  标记了 `allow_failure` 的任务和 bridge、已被重试替换的旧 bridge 都不计入；状态不是失败的子流水线不展开。
  未能创建子流水线的 bridge（例如子流水线 YAML 有错）、读取失败的子流水线，都把 bridge 本身作为失败项列出。
- 前 5 个失败任务会附上第一条 `error:`、`fatal error:`、`undefined reference to` 或 `scons: ***` 行。
  编译任务的 scons 输出不在任务日志里，而是由 `tools/ci/build_job.sh` 写进产物 `ci_build_logs/<任务标签>.log`，
  所以先读这个产物；产物不存在或已过期（保留一周）时再读任务日志。GitLab 日志每行前的时间戳、
  runner 上的构建目录前缀都会去掉，只保留仓库内的相对路径。
- 所有任务都成功、流水线却是失败时（通常是 Pages 的 `pages:deploy` 部署失败，这个状态不在任务列表里），
  消息会注明这一点。
- 如果流水线的 `artifact_catalog` 任务成功，附上产物目录链接，可在目录中按"失败"状态筛选。
- 企业微信单条 markdown 消息上限是 4096 字节，超出的任务汇总为"另有 N 个失败任务未列出"。

消息不 @ 任何人，也不调用大模型。日志只在内网服务器上读取，不会发送给第三方，只有摘出的那一行报错会随消息发到企业微信。

## 部署

### 1. 准备

- **企业微信群机器人**：在目标群里"添加群机器人"，复制 webhook 地址中 `key=` 后面的部分。
- **GitLab 只读 token**：建一个只有 `read_api` 权限的 Project Access Token。项目开启了公开流水线时，
  Guest 角色就能读取流水线、任务日志和产物；读不到代码仓库只影响 `notify` 命令里的提交标题，会被省略
  （webhook 自带提交标题）。没有开启公开流水线时需要 Reporter 角色。
- **确认服务器能访问企业微信**。能 ping 通 GitLab 不代表能访问外网，先在服务器上执行：

  ```sh
  curl -sS -o /dev/null -w '%{http_code}\n' https://qyapi.weixin.qq.com/
  ```

  需要走代理时，设置 `https_proxy` 环境变量即可，脚本会自动使用。同时要把 GitLab 排除在代理之外，
  否则访问内网 GitLab 的请求也会走代理：`no_proxy=<内网别名>`。
- **GitLab 地址要能通过证书校验**。如果 GitLab 配置的地址是 IP，而证书只签发给域名，
  直接用 IP 访问时 Python 会报 `CERTIFICATE_VERIFY_FAILED`（`http://` 通常也会跳转到 https）。
  这时在服务器的 `/etc/hosts` 里给这个 IP 起一个能匹配证书的名字，`GITLAB_URL` 就用这个名字：

  ```sh
  echo '<GitLab IP> <证书能匹配的内网别名>' | sudo tee -a /etc/hosts
  curl -sS -o /dev/null -w '%{http_code}\n' https://<内网别名>/api/v4/version   # 401 即正常
  ```

  别名不要和公网 DNS 里已有的名字重复，否则会影响服务器上其他程序。
  消息里的流水线和任务链接取自 GitLab 返回的地址，浏览器打开不受影响。

### 2. 放置脚本和配置

```sh
sudo mkdir -p /opt/ci-failure-notify
sudo cp tools/ci/ci_failure_notify.py /opt/ci-failure-notify/
sudo install -m 600 /dev/null /etc/ci-failure-notify.env
sudoedit /etc/ci-failure-notify.env
```

`/etc/ci-failure-notify.env` 内容：

```sh
GITLAB_URL=https://<内网别名>
GITLAB_TOKEN=<read_api token>
WEBHOOK_SECRET=<自己生成的随机字符串，例如 openssl rand -hex 24 的输出>
WECOM_BOT_KEY=<群机器人 key>
# 可选：
# NOTIFY_REFS=main,release/v2.5
NOTIFY_STATE_FILE=/var/lib/ci-failure-notify/notified.json
NOTIFY_QUEUE_FILE=/var/lib/ci-failure-notify/pending.sqlite3
# https_proxy=http://<代理地址>
# no_proxy=<内网别名>
```

这个文件包含三个凭证，权限保持 `600`，不要提交到仓库。

### 3. 先手动验证

env 文件只有 root 能读，所以用 `sudo sh -c` 加载后再运行：

```sh
sudo sh -c 'set -a; . /etc/ci-failure-notify.env; set +a; exec python3 /opt/ci-failure-notify/ci_failure_notify.py test-message'
sudo sh -c 'set -a; . /etc/ci-failure-notify.env; set +a; exec python3 /opt/ci-failure-notify/ci_failure_notify.py notify --project <项目 ID> --pipeline <某条失败流水线 ID> --dry-run'
```

`test-message` 会发一条测试消息到群里，用来确认企业微信可达、key 正确。

`notify --dry-run` 只打印某条历史流水线的消息内容，不发送，也不写状态文件，可以用来确认 token 权限和消息格式。
它会先打印一行 `full SDK build (has trigger_child_pipeline): yes/NO`。如果完整构建显示 `NO`，请检查 `read_api` 权限及 GitLab 实例版本接口是否可读。

#### 手动补发

去掉 `--dry-run` 会真正发送，可用于补发重试三次仍失败或状态为 `uncertain` 的通知。发送前会写入状态文件；对 `uncertain` 事件先检查群消息，避免手动补发造成重复。

查看待人工核对的队列事件和发送状态：

```sh
sqlite3 /var/lib/ci-failure-notify/pending.sqlite3 "SELECT event_key, attempt FROM events WHERE status='uncertain'"
cat /var/lib/ci-failure-notify/notified.json
```

核对并完成手动恢复后，删除对应的 `uncertain` 队列记录以释放总记录额度。将 `<项目 ID>:<流水线 ID>` 替换为查询结果中的 `event_key`：

```sh
sqlite3 /var/lib/ci-failure-notify/pending.sqlite3 "DELETE FROM events WHERE event_key='<项目 ID>:<流水线 ID>' AND status='uncertain'"
```

### 4. 作为 systemd 服务运行

`/etc/systemd/system/ci-failure-notify.service`：

```ini
[Unit]
Description=SDK CI failure notification to WeCom
After=network-online.target
Wants=network-online.target

[Service]
EnvironmentFile=/etc/ci-failure-notify.env
ExecStart=/usr/bin/python3 /opt/ci-failure-notify/ci_failure_notify.py serve --port 8099
DynamicUser=yes
StateDirectory=ci-failure-notify
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now ci-failure-notify
curl http://127.0.0.1:8099/health      # 返回 ok
journalctl -u ci-failure-notify -f     # 查看每个 webhook 的处理结果
```

使用 `DynamicUser=yes` 时，状态文件和队列数据库都必须放在 `StateDirectory` 下（即上面的 `/var/lib/ci-failure-notify/`）。

### 5. 配置 GitLab webhook

在项目 **Settings → Webhooks** 添加：

- URL：`http://<服务器地址>:8099/gitlab`
- Secret token：与 `WEBHOOK_SECRET` 相同
- Trigger：只勾选 **Pipeline events**

webhook 地址是内网 IP 时，GitLab 默认会拒绝保存。需要管理员在
**Admin → Settings → Network → Outbound requests** 中勾选允许 webhook 访问本地网络，
或把服务器地址加入允许列表。

保存后可以点 **Test → Pipeline events** 发送一次测试事件。GitLab 用的是项目最近一条流水线的真实数据：
多数情况下不满足推送条件，日志里出现 `skip pipeline ...` 就说明链路是通的；
但如果最近一条恰好是 `main` 或 `release/v2.5` 上失败的完整构建，而且还没推送过，群里会真的收到一条通知。

## 安全

- 服务只接受 `X-Gitlab-Token` 与 `WEBHOOK_SECRET` 一致的 `POST /gitlab` 请求，请求体上限 4 MB。
  每个连接单独一个线程，10 秒内不发完请求就断开，空连接不会卡住服务。
- GitLab token 只需要 `read_api`，脚本不会修改 GitLab 上的任何内容。
- 日志和报错信息中不会打印 token 或企业微信 key。
- 服务是明文 HTTP，只应在内网中监听；如需跨网段，可在前面加 nginx 并限制来源为 GitLab 服务器。

## 本地测试

```sh
python3 -m unittest discover -s tests -p 'test_ci_failure_notify.py'
```

测试使用模拟的 GitLab 和企业微信，不会访问网络（webhook 测试只监听 `127.0.0.1`）。
它不能代替在服务器上用 `test-message` 和 `notify --dry-run` 做的实际验证。

## 兼容性与未验证项

- `pages` 任务配置了 `allow_failure: exit_codes: [3]`（跳过旧流水线的发布）。GitLab API 返回的
  `allow_failure` 字段在这种情况下是否为 `true` 尚未确认；如果不是，它会出现在失败列表里。
- 服务首次查询 trigger job 时读取 `/api/v4/version`：GitLab 19.2 及以上使用 `pipelines/:id/trigger_jobs`，旧版使用 `pipelines/:id/bridges`。
  版本接口不可读或返回无法识别的版本时会报错，不会把空结果误判为非完整构建。版本读取和接口选择由单测覆盖；实际 GitLab 实例权限及部署版本仍需部署验证。
