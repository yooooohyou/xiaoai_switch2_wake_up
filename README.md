# XiaoAI Switch2 Wake Up — 统一固件

ESP32-C3 固件：通过 MQTT 接收开关指令，收到"开"时发出一段 BLE 广播唤醒小爱音箱。

**串口只烧一次，之后换平台、改参数、升级固件全走网页。**

完整的烧录、配置和排查步骤见 **[GUIDE.md](GUIDE.md)**。

## 支持的云平台（网页下拉切换，不用重烧）

| 平台 | 说明 |
|---|---|
| 巴法云 | 明文 MQTT，主题收发纯文本 `on`/`off`。经米家可直接给小爱同学用 |
| 涂鸦 TuyaLink | TLS + HMAC-SHA256 设备证书，tylink 物模型。DP 标识符自动识别 |
| 通用 MQTT | 自填服务器、凭据、主题。纯文本匹配或按 JSON 路径取值 |

三个平台的参数各自独立保存，来回切换不会丢。

## 网页

| 地址 | 用途 |
|---|---|
| `/` | 配置页：选平台、填云端参数、设 BLE 唤醒目标 |
| `/status` | 诊断页：WiFi、NTP、MQTT 返回码、堆内存、最近一条指令，5 秒自刷新 |
| `/update` | 固件升级：上传 `.bin` 做 OTA |
| `/wake` | 手动触发一次 BLE 唤醒，脱离云端单测 |

## 硬件

| ESP32-C3 | 连接 |
|---|---|
| GPIO9 | 按钮一端（另一端接地，用内部上拉）|
| GPIO12 | 运行状态 LED 正极（负极接地）|
| GPIO13 | 开关状态 LED 正极（负极接地）|

需要 4MB flash。分区表用 `min_spiffs.csv`（app0/app1 各约 1.9MB），双分区是 OTA
的前提；编译放不下时按 `platformio.ini` 的注释换 NimBLE。

## 编译与烧录

```bash
pio run -t upload      # 首次：串口烧录
pio device monitor     # 串口输出（115200）
pio run                # 之后：只编译，产物走 /update 页面上传
```

## 快速上手

1. 上电后连热点 `ESP32_BLE_Wake`（密码 `12345678`），访问 `http://192.168.4.1` 填家庭 WiFi
2. 设备连上 WiFi 后访问 `http://esp32c3-wake.local`，选平台、填参数、保存重启
3. 访问 `/status` 确认 MQTT 已连接
4. 在对应平台的 App 里绑定设备

## LED 状态

| 闪烁 | 含义 |
|---|---|
| 0.3 秒 | 配置热点模式 |
| 0.5 秒 | 正在连接 WiFi |
| 2 秒 | 已连接 |

GPIO13 跟随开关状态：收到"开"常亮。

## 按键

- **短按**：配置模式下重启热点；正常模式下手动触发一次 BLE 广播
- **长按 > 3 秒**：出厂重置，清除所有配置

## 分支

| 分支 | 内容 |
|---|---|
| `claude/unified-mqtt-ota-YsnYv` | 本分支：多平台统一固件 + 网页 OTA |
| `claude/tuya-mqtt-YsnYv` | 只有涂鸦方案的单平台版本 |
| `claude/switch-to-chinese-YsnYv` | 只有巴法云方案的单平台版本 |
