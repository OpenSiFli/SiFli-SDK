# SWT6621S WiFi AT命令使用说明

## 1. 编译

使用 `example/rt_device/wifi` 例程编译工程。

## 2. 命令格式

```
skw_wifi_rf_test "<AT指令>"
```

## 3. 使用示例

```
msh />skw_wifi_rf_test "at+wifimpset=1"
```

## 4. 日志说明

```
TX:skw_wifi_rf_test at+wifimpset=1    <- 发送的AT指令
skw_wifi_rf_open: 0                    <- 端口0打开成功（0=成功）
send_data port0 size=16                <- 发送16字节数据到端口0
rf_cmd:at+wifimpset=1                  <- 发送的命令内容
send_len: 16 ret:16                    <- 发送长度16，返回16表示发送成功

Attempt 1 to recv_data...
recv_data rx_len:4 String: OK          <- WiFi芯片返回"OK"，表示命令已接收

Attempt 2 to recv_data...
recv_data rx_len:5 String: +TAP        <- WiFi芯片返回中间响应

Attempt 3 to recv_data...
recv_data rx_len:18 String: NPI_MP INIT DONE  <- WiFi芯片返回最终结果，命令执行完成
Command completed, exiting receive loop
test done
```

## 5. 返回值说明

| ret值 | 含义 |
|-------|------|
| > 0   | 发送/接收成功，值为数据长度 |
| -5 (-EIO) | 端口未初始化或WiFi芯片未就绪 |
| -22 (-EINVAL) | 参数无效 |

## 6. 常用AT指令

| 指令 | 作用 |
|------|------|
| `at+wifimpset=1` | 进入WiFi MP测试模式 |
| `at` | 基础AT测试 |

## 7. 注意事项

- 发送前确保WiFi芯片已初始化完成（`skw_wifi_rf_open` 返回0）
- AT指令会自动追加 `\r\n` 结尾
- 接收循环最多5次，收到 `NPI_MP INIT DONE` 后自动退出
