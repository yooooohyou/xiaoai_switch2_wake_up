# XiaoAI Switch2 Wake Up — 涂鸦(Tuya) 版

ESP32-C3 固件：通过涂鸦 IoT 平台的 **TuyaLink MQTT 标准协议**接收开关指令，
收到"开"时发出一段 BLE 广播唤醒小爱音箱。

> 本分支是涂鸦方案。仓库主线走的是巴法云方案，两者只是云端接入不同，
> BLE 唤醒部分完全一致。

完整的烧录、配置和排查步骤见 **[GUIDE.md](GUIDE.md)**。

## 功能

- 上电自动开热点配网，之后走局域网 Web 页配置云端参数
- TuyaLink MQTT（TLS 8883）接入，HMAC-SHA256 设备证书认证
- **DP 标识符可留空**，从云端下发的第一条指令中自动识别
- 数据中心可选（中国 / 美西 / 欧洲 / 印度）
- 内置 `/status` 诊断页：WiFi、NTP、MQTT 返回码、剩余堆内存、最近一条指令
- BLE 广播唤醒，MAC 与广播数据均可配置
- 按键短按手动触发广播，长按 3 秒出厂重置
- 看门狗 + 配置持久化（NVS）

## 硬件

| ESP32-C3 | 连接 |
|---|---|
| GPIO9 | 按钮一端（另一端接地，用内部上拉）|
| GPIO12 | 状态 LED 正极（负极接地）|
| GPIO13 | 控制 LED 正极（负极接地）|

## 编译与烧录

```bash
pio run -t upload      # 编译并烧录
pio device monitor     # 查看串口输出（115200）
```

## 快速上手

1. 烧录后设备开热点 `ESP32_BLE_Wake`（密码 `12345678`），
   浏览器访问 `http://192.168.4.1` 填写家庭 WiFi
2. 设备连上 WiFi 后访问 `http://esp32c3-wake.local`，填写涂鸦 Device ID、
   Device Secret，并选择与云项目一致的数据中心；DP 标识符留空即可
3. 访问 `http://esp32c3-wake.local/status` 确认 MQTT 已连接
4. 在涂鸦智能 App 里绑定这台设备后即可控制

## LED 状态

| 闪烁 | 含义 |
|---|---|
| 0.3 秒 | 配置热点模式 |
| 0.5 秒 | 正在连接 WiFi |
| 2 秒 | 已连接 |

## 故障排查

先看 `http://esp32c3-wake.local/status`，再对照 [GUIDE.md 第 12 节](GUIDE.md#12-常见问题)。
