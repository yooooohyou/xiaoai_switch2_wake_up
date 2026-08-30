/**
 * BLE 唤醒小爱音箱 (ESP32-C3)
 * 版本: 8.0 - 统一固件（多平台 + 网页 OTA）
 *
 * 设计目标：串口只烧这一次「底子」，之后换平台、改参数、升级固件全走网页。
 *
 * 支持的云端接入方式（配置页下拉切换，无需重新烧录）：
 *   1. 巴法云       —— 明文 MQTT，主题收发纯文本 on/off
 *   2. 涂鸦 TuyaLink —— TLS + HMAC-SHA256 设备证书，tylink 物模型
 *   3. 通用 MQTT     —— 自填服务器/凭据/主题，纯文本匹配或 JSON 取值
 *
 * 首次配置流程：
 *   1. 上电后 LED 快闪，设备开启配置热点 "ESP32_BLE_Wake"（密码 12345678）
 *   2. 手机连接热点 → 浏览器访问 192.168.4.1，填写 WiFi 账号密码
 *   3. 设备连上 WiFi 后，访问 http://esp32c3-wake.local 选择平台并填写参数
 *
 * 局域网页面：
 *   /         配置页（平台、云端参数、BLE 唤醒目标）
 *   /status   诊断页（WiFi / NTP / MQTT 返回码 / 堆内存 / 最近一条指令）
 *   /update   固件升级页（上传 .bin 做 OTA）
 *   /wake     手动触发一次 BLE 唤醒广播，用于脱离云端单测
 *
 * 按键操作：
 *   短按 → 配置模式：重启热点；正常模式：手动触发 BLE 广播测试
 *   长按(>3s) → 出厂重置，清除所有配置
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <esp_mac.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "mbedtls/md.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "time.h"

#define FIRMWARE_VERSION "8.0"

// ==================== 引脚定义 ====================
#define TRIGGER_PIN    9
#define LED_PIN        12
#define STATUS_LED_PIN 13

// ==================== 时间常量 ====================
#define WDT_TIMEOUT_SECONDS   180
#define BUTTON_DEBOUNCE_MS    50
#define LONG_PRESS_MS         3000
#define BLE_WAKE_DURATION_MS  1000
#define MQTT_RECONNECT_MS     8000
#define WIFI_TIMEOUT_MS       30000
#define WIFI_RECONNECT_MS     15000
#define NTP_TIMEOUT_S         20
#define MQTT_SOCKET_TIMEOUT_S 20
#define MQTT_HEARTBEAT_MS     45000   // 定期上报状态，帮助云端维持在线判定
#define ECHO_SUPPRESS_MS      2000    // 上报后短暂忽略回环消息

// ==================== 配置热点 ====================
#define SETUP_AP_SSID     "ESP32_BLE_Wake"
#define SETUP_AP_PASSWORD "12345678"

// ==================== 平台 ====================
enum Platform {
    PLATFORM_BEMFA   = 0,   // 巴法云
    PLATFORM_TUYA    = 1,   // 涂鸦 TuyaLink
    PLATFORM_GENERIC = 2,   // 通用 MQTT
};

// ---- 巴法云 ----
#define BEMFA_HOST "bemfa.com"
#define BEMFA_PORT 9501

// ---- 涂鸦：接入地址由项目所在数据中心决定，选错会认证失败 ----
#define DEFAULT_TUYA_HOST "m1.tuyacn.com"
#define TUYA_PORT 8883

struct TuyaRegion { const char* host; const char* label; };
static const TuyaRegion TUYA_REGIONS[] = {
    {"m1.tuyacn.com", "中国"},
    {"m1.tuyaus.com", "美西"},
    {"m1.tuyaeu.com", "欧洲"},
    {"m1.tuyain.com", "印度"},
};
static const size_t TUYA_REGION_COUNT = sizeof(TUYA_REGIONS) / sizeof(TUYA_REGIONS[0]);

// ==================== BLE ====================
#define BLE_DEVICE_NAME "ESP32C3_BLE_Beacon"

// ==================== 默认 BLE 参数 ====================
const char* DEFAULT_BLE_MAC  = "78:81:8c:06:9a:c4";
const char* DEFAULT_BLE_DATA = "0201061BFF53050100037E0566200001816D60168C81780F00000000000000";

// ==================== 配置缓冲区（从 NVS 加载）====================
char wifi_ssid[33]          = "";
char wifi_pass[65]          = "";

int  platform               = PLATFORM_BEMFA;

// 巴法云
char bemfa_uid[65]          = "";
char bemfa_topic[33]        = "";

// 涂鸦
char tuya_device_id[25]     = "";
char tuya_device_secret[33] = "";
char tuya_dp_key[33]        = "";   // 留空 = 自动识别
char tuya_host[41]          = DEFAULT_TUYA_HOST;

// 通用 MQTT
char gen_host[65]           = "";
int  gen_port               = 1883;
bool gen_tls                = false;
char gen_client_id[49]      = "";
char gen_user[49]           = "";
char gen_pass[65]           = "";
char gen_sub_topic[65]      = "";
char gen_pub_topic[65]      = "";
char gen_on_text[25]        = "on";
char gen_off_text[25]       = "off";
char gen_json_key[49]       = "";   // 留空 = 纯文本匹配；填了则从 JSON 取值

// BLE
char ble_mac[19]            = "";
char ble_data[65]           = "";

// ==================== 运行时状态 ====================
char          learned_dp_key[33] = "";   // 涂鸦：从下发指令学到的标识符
String        lastCommandLog     = "";   // 最近一条云端指令，供 /status 查看
int           lastMqttState      = -100; // 最近一次 MQTT 返回码
unsigned long lastHeartbeat      = 0;
unsigned long echoSuppressUntil  = 0;
String        lastPublished      = "";   // 自己最近发出的内容，用于识别回环
uint32_t      mqttConnectCount   = 0;
uint32_t      mqttFailCount      = 0;
String        otaError           = "";

// ==================== BLE 广播数据 ====================
uint8_t baseMAC[6] = {0x78, 0x81, 0x8c, 0x06, 0x9a, 0xc4};

static uint8_t wake_adv_raw[64] = {
    0x02, 0x01, 0x06,
    0x1B, 0xFF,
    0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05, 0x66, 0x20, 0x00, 0x01, 0x81,
    0x6D, 0x60, 0x16, 0x8C, 0x81, 0x78,
    0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static size_t wake_adv_len = 28;

// ==================== 系统状态 ====================
enum SystemStatus {
    STATUS_BOOT,
    STATUS_CONFIGAP,
    STATUS_WIFI_CONNECTING,
    STATUS_CONNECTED
};
SystemStatus current_status = STATUS_BOOT;

unsigned long lastLedToggle = 0;
bool          ledState      = false;

// ==================== BLE 状态 ====================
bool            bleInitialized = false;
unsigned long   bleAdvStart    = 0;
BLEAdvertising* pWakeAdv       = nullptr;

// ==================== 跨调用标志 ====================
volatile bool pendingBLEStart = false;
volatile bool pendingBLEStop  = false;
volatile bool pendingReport   = false;
bool          switchState     = false;   // 云端最后一次下发的开关状态

// ==================== 重连计时 ====================
unsigned long lastMqttAttempt = 0;
unsigned long lastWifiAttempt = 0;

// ==================== 对象 ====================
Preferences      prefs;
WebServer        webServer(80);
WiFiClient       plainClient;   // 明文：巴法云 / 通用 MQTT 不开 TLS
WiFiClientSecure tlsClient;     // TLS：涂鸦 / 通用 MQTT 开 TLS
PubSubClient     mqtt;
bool             apActive = false;

// ==================== 函数声明 ====================
void loadConfig();
void saveWiFiConfig(const String& ssid, const String& pass);
void startConfigAP();
void stopConfigAP();
void connectWiFi();
void syncNTP();
String hmacSha256(const char* key, const String& data);
const char* mqttStateText(int state);
const char* platformName(int p);
bool platformConfigured();
bool mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void applyCommand(bool on, const String& source);
void reportState(bool on);
void sendTuyaSetResponse(const String& msgId);
void handleMQTT();
void initWakeupBLE();
void startBLEAdvertising();
void stopBLEAdvertising();
void handleBLEAdvertising();
void checkButton();
void updateStatusLED();
void safeRestart(const char* reason);
String buildWiFiPage();
void   sendServicePage();
String buildStatusPage();
String buildUpdatePage();
void registerAPRoutes();
void registerLANRoutes();

// =====================================================
// 配置加载 / 保存
// =====================================================

void loadConfig() {
    if (!prefs.begin("config", true)) {
        prefs.end();
        strcpy(ble_mac,   DEFAULT_BLE_MAC);
        strcpy(ble_data,  DEFAULT_BLE_DATA);
        strcpy(tuya_host, DEFAULT_TUYA_HOST);
        return;
    }
    prefs.getString("wifi_ssid",   "").toCharArray(wifi_ssid, sizeof(wifi_ssid));
    prefs.getString("wifi_pass",   "").toCharArray(wifi_pass, sizeof(wifi_pass));

    platform = prefs.getInt("platform", PLATFORM_BEMFA);

    prefs.getString("bemfa_uid",   "").toCharArray(bemfa_uid,   sizeof(bemfa_uid));
    prefs.getString("bemfa_topic", "").toCharArray(bemfa_topic, sizeof(bemfa_topic));

    prefs.getString("tuya_did",    "").toCharArray(tuya_device_id,     sizeof(tuya_device_id));
    prefs.getString("tuya_secret", "").toCharArray(tuya_device_secret, sizeof(tuya_device_secret));
    prefs.getString("tuya_dp",     "").toCharArray(tuya_dp_key,        sizeof(tuya_dp_key));
    prefs.getString("tuya_host", DEFAULT_TUYA_HOST).toCharArray(tuya_host, sizeof(tuya_host));

    prefs.getString("gen_host", "").toCharArray(gen_host,      sizeof(gen_host));
    gen_port = prefs.getInt("gen_port", 1883);
    gen_tls  = prefs.getBool("gen_tls", false);
    prefs.getString("gen_cid",  "").toCharArray(gen_client_id, sizeof(gen_client_id));
    prefs.getString("gen_user", "").toCharArray(gen_user,      sizeof(gen_user));
    prefs.getString("gen_pass", "").toCharArray(gen_pass,      sizeof(gen_pass));
    prefs.getString("gen_sub",  "").toCharArray(gen_sub_topic, sizeof(gen_sub_topic));
    prefs.getString("gen_pub",  "").toCharArray(gen_pub_topic, sizeof(gen_pub_topic));
    prefs.getString("gen_on",   "on").toCharArray(gen_on_text,  sizeof(gen_on_text));
    prefs.getString("gen_off",  "off").toCharArray(gen_off_text, sizeof(gen_off_text));
    prefs.getString("gen_json", "").toCharArray(gen_json_key,  sizeof(gen_json_key));

    prefs.getString("ble_mac",  DEFAULT_BLE_MAC).toCharArray(ble_mac,   sizeof(ble_mac));
    prefs.getString("ble_data", DEFAULT_BLE_DATA).toCharArray(ble_data, sizeof(ble_data));
    prefs.end();

    if (strlen(tuya_host) == 0) strcpy(tuya_host, DEFAULT_TUYA_HOST);
    strncpy(learned_dp_key, tuya_dp_key, sizeof(learned_dp_key) - 1);

    Serial.println("✅ 配置已加载 - WiFi: " + String(wifi_ssid)
                   + "  平台: " + String(platformName(platform)));
}

void saveWiFiConfig(const String& ssid, const String& pass) {
    if (!prefs.begin("config", false)) return;
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    Serial.println("💾 WiFi 配置已保存");
}

// 保存配置页提交的全部字段。未选中的平台字段照样保存，
// 这样来回切换平台不会丢掉已经填过的参数。
void saveServiceConfig() {
    if (!prefs.begin("config", false)) return;

    prefs.putInt("platform", webServer.arg("platform").toInt());

    prefs.putString("bemfa_uid",   webServer.arg("bemfa_uid"));
    prefs.putString("bemfa_topic", webServer.arg("bemfa_topic"));

    prefs.putString("tuya_did",    webServer.arg("tuya_did"));
    prefs.putString("tuya_secret", webServer.arg("tuya_secret"));
    prefs.putString("tuya_dp",     webServer.arg("tuya_dp"));   // 允许留空，自动识别
    String th = webServer.arg("tuya_host");
    prefs.putString("tuya_host",   th.length() > 0 ? th : String(DEFAULT_TUYA_HOST));

    prefs.putString("gen_host", webServer.arg("gen_host"));
    int gp = webServer.arg("gen_port").toInt();
    prefs.putInt("gen_port", (gp > 0 && gp < 65536) ? gp : 1883);
    prefs.putBool("gen_tls", webServer.arg("gen_tls") == "1");
    prefs.putString("gen_cid",  webServer.arg("gen_cid"));
    prefs.putString("gen_user", webServer.arg("gen_user"));
    prefs.putString("gen_pass", webServer.arg("gen_pass"));
    prefs.putString("gen_sub",  webServer.arg("gen_sub"));
    prefs.putString("gen_pub",  webServer.arg("gen_pub"));
    String on  = webServer.arg("gen_on");
    String off = webServer.arg("gen_off");
    prefs.putString("gen_on",   on.length()  > 0 ? on  : String("on"));
    prefs.putString("gen_off",  off.length() > 0 ? off : String("off"));
    prefs.putString("gen_json", webServer.arg("gen_json"));

    String mac  = webServer.arg("ble_mac");
    String data = webServer.arg("ble_data");
    prefs.putString("ble_mac",  mac.length()  > 0 ? mac  : String(DEFAULT_BLE_MAC));
    prefs.putString("ble_data", data.length() > 0 ? data : String(DEFAULT_BLE_DATA));

    prefs.end();
    Serial.println("💾 服务配置已保存");
}

// =====================================================
// 工具函数
// =====================================================

const char* platformName(int p) {
    switch (p) {
        case PLATFORM_BEMFA:   return "巴法云";
        case PLATFORM_TUYA:    return "涂鸦 TuyaLink";
        case PLATFORM_GENERIC: return "通用 MQTT";
        default:               return "未知";
    }
}

// 当前平台的必填项是否都填了
bool platformConfigured() {
    switch (platform) {
        case PLATFORM_BEMFA:
            return strlen(bemfa_uid) > 0 && strlen(bemfa_topic) > 0;
        case PLATFORM_TUYA:
            return strlen(tuya_device_id) > 0 && strlen(tuya_device_secret) > 0;
        case PLATFORM_GENERIC:
            return strlen(gen_host) > 0 && strlen(gen_sub_topic) > 0;
        default:
            return false;
    }
}

const char* mqttStateText(int state) {
    switch (state) {
        case -100: return "尚未尝试连接";
        case -4: return "-4 服务器无响应，可能是地址/端口不通";
        case -3: return "-3 连接被断开";
        case -2: return "-2 TCP/TLS 建连失败，检查网络与服务器地址";
        case -1: return "-1 已主动断开";
        case  0: return "0 已连接";
        case  1: return "1 协议版本不被接受";
        case  2: return "2 ClientID 被拒绝";
        case  3: return "3 服务不可用";
        case  4: return "4 用户名或密码错误";
        case  5: return "5 认证未通过";
        default: return "未知状态";
    }
}

String hmacSha256(const char* key, const String& data) {
    uint8_t hmac[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, (const uint8_t*)key, strlen(key));
    mbedtls_md_hmac_update(&ctx, (const uint8_t*)data.c_str(), data.length());
    mbedtls_md_hmac_finish(&ctx, hmac);
    mbedtls_md_free(&ctx);

    String result = "";
    for (int i = 0; i < 32; i++) {
        if (hmac[i] < 0x10) result += "0";
        result += String(hmac[i], HEX);
    }
    return result;
}

// HTML 属性值转义，避免配置里的引号把页面撑坏
String esc(const char* s) {
    String out = "";
    for (const char* p = s; *p; p++) {
        switch (*p) {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '\'': out += "&#39;";  break;
            case '"':  out += "&quot;"; break;
            default:   out += *p;
        }
    }
    return out;
}

void safeRestart(const char* reason) {
    Serial.println("🔄 重启: " + String(reason));
    prefs.end();
    delay(500);
    ESP.restart();
}

// =====================================================
// Web 配置页
// =====================================================

// 两个页面共用的样式
static const char PAGE_STYLE[] =
    "<style>"
    "body{font-family:sans-serif;padding:20px;max-width:480px;margin:auto;color:#333}"
    "h2{margin-bottom:4px}"
    "h3{margin:0 0 6px;font-size:1em;color:#555}"
    "label{display:block;margin-top:14px;font-weight:bold;font-size:.9em}"
    "input[type=text],input[type=password],input[type=number],select{"
    "  width:100%;box-sizing:border-box;padding:9px;background:#fff;"
    "  border:1px solid #ddd;border-radius:6px;font-size:.95em;margin-top:4px}"
    "input[type=submit]{width:100%;background:#1989fa;color:#fff;border:none;"
    "  padding:12px;border-radius:8px;font-size:1em;margin-top:20px;cursor:pointer}"
    "hr{border:none;border-top:1px solid #eee;margin:20px 0}"
    "a{color:#1989fa}"
    ".row{display:flex;gap:10px}.row>div{flex:1}"
    ".tip{font-size:.82em;color:#aaa;margin-top:5px;line-height:1.5}"
    ".ok{background:#f0f9eb;border:1px solid #b3e19d;border-radius:8px;"
    "  padding:10px 14px;margin-bottom:16px;font-size:.9em}"
    ".warn{background:#fff7e6;border:1px solid #ffd591;border-radius:8px;"
    "  padding:10px 14px;margin-bottom:16px;font-size:.9em;color:#874d00}"
    "table{width:100%;border-collapse:collapse;font-size:.9em}"
    "td{padding:8px 6px;border-bottom:1px solid #eee;vertical-align:top}"
    "td:first-child{color:#888;width:40%}"
    "</style>";

// ---- AP 热点页：仅 WiFi 配置 ----
String buildWiFiPage() {
    return
        String("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>")
        + PAGE_STYLE +
        "</head><body>"
        "<h2>WiFi 配置</h2>"
        "<p class='tip'>连接成功后，可通过局域网地址继续选择云平台并填写参数</p>"
        "<form method='POST' action='/save-wifi'>"
        "<label>WiFi 名称 (SSID)</label>"
        "<input type='text' name='ssid' value='" + esc(wifi_ssid) + "' required "
        "placeholder='家庭 WiFi 名称（仅支持 2.4GHz）'>"
        "<label>WiFi 密码</label>"
        "<input type='password' name='pass' value='' placeholder='WiFi 密码（无密码留空）'>"
        "<p class='tip'>保存后设备将自动连接 WiFi，LED 由快闪变为慢闪即为成功</p>"
        "<input type='submit' value='保存并连接'>"
        "</form></body></html>";
}

// ---- 局域网配置页：分段发送，降低大页面对堆内存的冲击 ----
void sendServicePage() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html; charset=utf-8", "");

    webServer.sendContent(
        "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    webServer.sendContent(PAGE_STYLE);
    webServer.sendContent("</head><body><h2>BLE 唤醒配置</h2>");

    // ---- 状态条 ----
    if (!platformConfigured()) {
        webServer.sendContent("<div class='warn'>当前平台（" + String(platformName(platform))
                              + "）参数不完整，设备暂无法接收控制指令</div>");
    } else if (mqtt.connected()) {
        webServer.sendContent("<div class='ok'><b>" + String(platformName(platform))
                              + "</b> 已连接</div>");
    } else {
        webServer.sendContent("<div class='warn'><b>" + String(platformName(platform))
                              + "</b> 未连接（" + String(mqttStateText(lastMqttState)) + "）</div>");
    }

    webServer.sendContent("<form method='POST' action='/save'>");

    // ---- 平台选择 ----
    String sel = "<hr><h3>云平台</h3>"
                 "<select id='platform' name='platform' onchange='pick()'>";
    for (int i = 0; i <= 2; i++) {
        sel += "<option value='" + String(i) + "'" + (platform == i ? " selected" : "")
             + ">" + String(platformName(i)) + "</option>";
    }
    sel += "</select>";
    webServer.sendContent(sel);

    // ---- 巴法云 ----
    webServer.sendContent(
        "<div id='sec_0'>"
        "<p class='tip'>登录 <b>cloud.bemfa.com</b> → 右上角头像 → 私钥（复制整串）<br>"
        "设备管理 → 新建主题 → <b>主题名后三位是设备类型码</b>："
        "001 插座 / 002 灯 / 003 风扇 / 004 传感器 / 005 空调，建议叫 <b>xiaoai001</b><br>"
        "米家 App → 我的 → 其他平台设备 → 巴法 → 绑定账号，即可让小爱语音控制</p>"
        "<label>私钥（UID）</label>"
        "<input type='text' name='bemfa_uid' value='" + esc(bemfa_uid) + "' "
        "placeholder='巴法云控制台右上角私钥'>"
        "<label>主题名</label>"
        "<input type='text' name='bemfa_topic' value='" + esc(bemfa_topic) + "' "
        "placeholder='例如: xiaoai001'>"
        "</div>");

    // ---- 涂鸦 ----
    String tuyaHostOptions = "";
    bool matched = false;
    for (size_t i = 0; i < TUYA_REGION_COUNT; i++) {
        bool s = (strcmp(tuya_host, TUYA_REGIONS[i].host) == 0);
        if (s) matched = true;
        tuyaHostOptions += "<option value='" + String(TUYA_REGIONS[i].host) + "'"
                         + (s ? " selected" : "") + ">"
                         + String(TUYA_REGIONS[i].label) + " - "
                         + String(TUYA_REGIONS[i].host) + "</option>";
    }
    if (!matched) {
        tuyaHostOptions = "<option value='" + esc(tuya_host) + "' selected>"
                        + esc(tuya_host) + "</option>" + tuyaHostOptions;
    }

    webServer.sendContent(
        "<div id='sec_1'>"
        "<p class='tip'>iot.tuya.com → 产品开发 → 设备管理 → 设备详情 → <b>设备证书</b><br>"
        "<b>数据中心必须与云项目所在区域一致</b>，选错会认证失败</p>"
        "<label>设备 ID（Device ID）</label>"
        "<input type='text' name='tuya_did' value='" + esc(tuya_device_id) + "' "
        "placeholder='26c24759c1344038xxxxxx'>"
        "<label>设备密钥（Device Secret）</label>"
        "<input type='text' name='tuya_secret' value='" + esc(tuya_device_secret) + "' "
        "placeholder='XXXXXXXXXXXXXXXX'>"
        "<label>数据中心</label>"
        "<select name='tuya_host'>" + tuyaHostOptions + "</select>"
        "<label>开关 DP 标识符（选填）</label>"
        "<input type='text' name='tuya_dp' value='" + esc(tuya_dp_key) + "' "
        "placeholder='留空则自动识别'>"
        "<p class='tip'>新版设备详情页不再显示标识符，留空即可——固件会从云端下发的"
        "第一条指令里学到它。设备详情里「绑定用户 / 绑定 APP」为空说明还没在"
        "涂鸦智能 App 里绑定，此时 App 无法下发指令。</p>"
        "</div>");

    // ---- 通用 MQTT ----
    webServer.sendContent(
        "<div id='sec_2'>"
        "<p class='tip'>适用于 Home Assistant、EMQX、自建 broker 等标准 MQTT 服务</p>"
        "<div class='row'>"
        "<div><label>服务器地址</label>"
        "<input type='text' name='gen_host' value='" + esc(gen_host) + "' "
        "placeholder='192.168.1.10'></div>"
        "<div><label>端口</label>"
        "<input type='number' name='gen_port' value='" + String(gen_port) + "'></div>"
        "</div>"
        "<label>加密方式</label>"
        "<select name='gen_tls'>"
        "<option value='0'" + String(gen_tls ? "" : " selected") + ">明文 TCP</option>"
        "<option value='1'" + String(gen_tls ? " selected" : "") + ">TLS（不校验证书）</option>"
        "</select>"
        "<label>ClientID（留空自动生成）</label>"
        "<input type='text' name='gen_cid' value='" + esc(gen_client_id) + "' "
        "placeholder='esp32c3-wake'>"
        "<div class='row'>"
        "<div><label>用户名（选填）</label>"
        "<input type='text' name='gen_user' value='" + esc(gen_user) + "'></div>"
        "<div><label>密码（选填）</label>"
        "<input type='text' name='gen_pass' value='" + esc(gen_pass) + "'></div>"
        "</div>"
        "<label>订阅主题（接收指令）</label>"
        "<input type='text' name='gen_sub' value='" + esc(gen_sub_topic) + "' "
        "placeholder='home/xiaoai/set'>"
        "<label>上报主题（回报状态，选填）</label>"
        "<input type='text' name='gen_pub' value='" + esc(gen_pub_topic) + "' "
        "placeholder='home/xiaoai/state'>");

    webServer.sendContent(
        "<label>JSON 字段路径（选填）</label>"
        "<input type='text' name='gen_json' value='" + esc(gen_json_key) + "' "
        "placeholder='留空按纯文本匹配，例如 state 或 data.switch'>"
        "<p class='tip'>留空时直接比对整条消息内容；填了则先按 JSON 解析，"
        "再按这个路径取值（支持用点号进入下一层）。取到的值可以是 true/false、"
        "0/1，或下面填的那两个字符串。</p>"
        "<div class='row'>"
        "<div><label>「开」对应内容</label>"
        "<input type='text' name='gen_on' value='" + esc(gen_on_text) + "'></div>"
        "<div><label>「关」对应内容</label>"
        "<input type='text' name='gen_off' value='" + esc(gen_off_text) + "'></div>"
        "</div>"
        "</div>");

    // ---- BLE ----
    webServer.sendContent(
        "<hr><h3>BLE 唤醒目标</h3>"
        "<p class='tip'>留空使用默认值；如需唤醒特定音箱，填写其 BLE MAC 和广播数据</p>"
        "<label>目标设备 BLE MAC</label>"
        "<input type='text' name='ble_mac' value='" + esc(ble_mac) + "' "
        "placeholder='78:81:8c:06:9a:c4' "
        "pattern='^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$'>"
        "<label>BLE 广播数据（HEX，不含空格）</label>"
        "<input type='text' name='ble_data' value='" + esc(ble_data) + "' "
        "placeholder='0201061BFF...' pattern='^[0-9a-fA-F]*$'>"
        "<input type='submit' value='保存并重启'>"
        "</form>"
        "<p><a href='/status'>运行状态</a> &nbsp;·&nbsp; <a href='/update'>固件升级</a></p>");

    // 只显示当前选中平台的字段块
    webServer.sendContent(
        "<script>"
        "function pick(){var v=document.getElementById('platform').value;"
        "for(var i=0;i<3;i++){var e=document.getElementById('sec_'+i);"
        "if(e)e.style.display=(String(i)===v?'':'none');}}"
        "pick();"
        "</script></body></html>");

    webServer.sendContent("");
}

// ---- 诊断页 /status ----
String buildStatusPage() {
    time_t now = time(nullptr);
    struct tm tmv;
    char timeStr[32] = "未同步";
    if (now > 1000000000L) {
        localtime_r(&now, &tmv);
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tmv);
    }

    String rows = "";
    auto row = [&rows](const String& k, const String& v) {
        rows += "<tr><td>" + k + "</td><td>" + v + "</td></tr>";
    };

    row("固件版本", String(FIRMWARE_VERSION));
    row("平台", String(platformName(platform))
                + (platformConfigured() ? "" : "（参数不完整）"));
    String wifiInfo = "未连接";
    if (WiFi.status() == WL_CONNECTED) {
        wifiInfo = "已连接 " + WiFi.SSID() + " (" + WiFi.localIP().toString()
                   + ", RSSI " + String(WiFi.RSSI()) + " dBm)";
    }
    row("WiFi", wifiInfo);
    row("NTP 时间", String(timeStr));
    row("MQTT", mqtt.connected() ? "已连接" : "未连接");
    row("最近返回码", String(mqttStateText(lastMqttState)));
    row("连接成功 / 失败次数", String(mqttConnectCount) + " / " + String(mqttFailCount));
    if (platform == PLATFORM_TUYA) {
        row("DP 标识符", strlen(learned_dp_key) ? String(learned_dp_key) : String("尚未识别"));
    }
    row("当前开关状态", switchState ? "开" : "关");
    row("最近一条指令", lastCommandLog.length() ? lastCommandLog : String("无"));
    row("BLE", bleInitialized ? "已初始化" : "未初始化");
    row("剩余堆内存", String(ESP.getFreeHeap()) + " B（最低 "
                     + String(ESP.getMinFreeHeap()) + " B）");
    row("运行时长", String(millis() / 1000) + " s");

    return
        String("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='5'>")
        + PAGE_STYLE +
        "</head><body><h2>运行状态</h2><table>" + rows + "</table>"
        "<p><a href='/'>配置页</a> &nbsp;·&nbsp; <a href='/wake'>手动触发 BLE 唤醒</a>"
        " &nbsp;·&nbsp; <a href='/update'>固件升级</a></p>"
        "<p class='tip'>本页每 5 秒自动刷新</p>"
        "</body></html>";
}

// ---- 固件升级页 /update ----
String buildUpdatePage() {
    String note = otaError.length()
        ? "<div class='warn'>上次升级失败：" + otaError + "</div>"
        : "";

    return
        String("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>")
        + PAGE_STYLE +
        "</head><body><h2>固件升级</h2>"
        + note +
        "<p class='tip'>当前版本 <b>" + String(FIRMWARE_VERSION) + "</b>，"
        "可用空间 " + String(ESP.getFreeSketchSpace() / 1024) + " KB<br>"
        "选择 PlatformIO 编译出的 <b>.pio/build/…/firmware.bin</b> 上传，"
        "写入完成后设备自动重启。<br>"
        "升级过程中请保持供电和网络，中途断电不会变砖——旧固件还在另一个分区里。</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<label>固件文件（.bin）</label>"
        "<input type='file' name='firmware' accept='.bin' required style='margin-top:6px'>"
        "<input type='submit' value='上传并升级'>"
        "</form>"
        "<p><a href='/'>配置页</a> &nbsp;·&nbsp; <a href='/status'>运行状态</a></p>"
        "</body></html>";
}

// =====================================================
// 路由
// =====================================================

void registerAPRoutes() {
    webServer.on("/", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildWiFiPage());
    });

    webServer.on("/save-wifi", HTTP_POST, []() {
        String ssid = webServer.arg("ssid");
        String pass = webServer.arg("pass");
        if (ssid.length() == 0) {
            webServer.send(400, "text/html; charset=utf-8",
                "<meta charset='UTF-8'>"
                "<p style='font-family:sans-serif;padding:20px'>❌ WiFi 名称不能为空</p>");
            return;
        }
        saveWiFiConfig(ssid, pass);
        webServer.send(200, "text/html; charset=utf-8",
            "<meta charset='UTF-8'>"
            "<p style='font-family:sans-serif;padding:20px'>"
            "✅ WiFi 已保存，设备重启中...<br>"
            "<small>连接成功后请访问 <b>http://esp32c3-wake.local</b> 选择云平台</small></p>");
        delay(800);
        ESP.restart();
    });
}

void registerLANRoutes() {
    webServer.on("/", HTTP_GET, []() { sendServicePage(); });

    webServer.on("/status", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildStatusPage());
    });

    // 手动触发一次 BLE 唤醒广播，用于脱离云端单测
    webServer.on("/wake", HTTP_GET, []() {
        pendingBLEStart = true;
        webServer.send(200, "text/html; charset=utf-8",
            "<meta charset='UTF-8'><p style='font-family:sans-serif;padding:20px'>"
            "📡 已触发 BLE 唤醒广播，<a href='/status'>返回状态页</a></p>");
    });

    webServer.on("/save", HTTP_POST, []() {
        saveServiceConfig();
        webServer.send(200, "text/html; charset=utf-8",
            "<meta charset='UTF-8'>"
            "<p style='font-family:sans-serif;padding:20px'>✅ 已保存，设备重启中...</p>");
        delay(800);
        ESP.restart();
    });

    // ---- OTA ----
    webServer.on("/update", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildUpdatePage());
    });

    webServer.on("/update", HTTP_POST,
        // 上传结束后的响应
        []() {
            bool ok = (otaError.length() == 0) && !Update.hasError();
            if (ok) {
                webServer.send(200, "text/html; charset=utf-8",
                    "<meta charset='UTF-8'>"
                    "<p style='font-family:sans-serif;padding:20px'>"
                    "✅ 升级完成，设备重启中...<br>"
                    "<small>约 10 秒后访问 <b>http://esp32c3-wake.local</b></small></p>");
                Serial.println("✅ OTA 完成，重启");
                delay(1000);
                ESP.restart();
            } else {
                webServer.send(500, "text/html; charset=utf-8",
                    "<meta charset='UTF-8'>"
                    "<p style='font-family:sans-serif;padding:20px'>❌ 升级失败："
                    + otaError + "<br><a href='/update'>返回</a></p>");
            }
        },
        // 分块接收固件
        []() {
            HTTPUpload& up = webServer.upload();
            static bool firstChunk = false;

            if (up.status == UPLOAD_FILE_START) {
                otaError   = "";
                firstChunk = true;
                Serial.printf("⬆️ OTA 开始: %s\n", up.filename.c_str());
                // 腾出资源：断开云端连接、停掉正在进行的广播
                mqtt.disconnect();
                stopBLEAdvertising();
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    otaError = String(Update.errorString());
                    Serial.println("❌ Update.begin 失败: " + otaError);
                }
            } else if (up.status == UPLOAD_FILE_WRITE) {
                esp_task_wdt_reset();
                if (otaError.length()) return;

                // ESP 固件的第一个字节固定是 0xE9，挡掉传错文件的情况
                if (firstChunk) {
                    firstChunk = false;
                    if (up.currentSize == 0 || up.buf[0] != 0xE9) {
                        otaError = "这不是 ESP32 固件文件（缺少 0xE9 标识）";
                        Update.abort();
                        Serial.println("❌ " + otaError);
                        return;
                    }
                }
                if (Update.write(up.buf, up.currentSize) != up.currentSize) {
                    otaError = String(Update.errorString());
                    Serial.println("❌ OTA 写入失败: " + otaError);
                }
            } else if (up.status == UPLOAD_FILE_END) {
                if (otaError.length()) return;
                if (Update.end(true)) {
                    Serial.printf("✅ OTA 写入 %u 字节\n", up.totalSize);
                } else {
                    otaError = String(Update.errorString());
                    Serial.println("❌ Update.end 失败: " + otaError);
                }
            } else if (up.status == UPLOAD_FILE_ABORTED) {
                Update.abort();
                if (otaError.length() == 0) otaError = "上传被中断";
                Serial.println("❌ OTA 中断");
            }
        });
}

// =====================================================
// 配置热点 / WiFi / NTP
// =====================================================

void startConfigAP() {
    if (apActive) return;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD);
    registerAPRoutes();
    webServer.begin();
    apActive = true;
    current_status = STATUS_CONFIGAP;
    Serial.println("📶 配置热点已开启: " + String(SETUP_AP_SSID)
                   + "  密码: " + String(SETUP_AP_PASSWORD));
    Serial.println("   浏览器访问: http://192.168.4.1  （仅填 WiFi 账号密码）");
}

void stopConfigAP() {
    if (!apActive) return;
    WiFi.softAPdisconnect(true);
    apActive = false;
    Serial.println("📴 配置热点已关闭");
}

void connectWiFi() {
    current_status = STATUS_WIFI_CONNECTING;
    Serial.print("📶 连接 WiFi: " + String(wifi_ssid) + " ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            Serial.println("\n❌ WiFi 连接超时，启动配置热点");
            WiFi.disconnect();
            startConfigAP();
            return;
        }
        delay(500);
        esp_task_wdt_reset();
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi 已连接: " + WiFi.localIP().toString());
    current_status = STATUS_CONNECTED;
}

// 涂鸦签名依赖时间戳；其他平台也用它在状态页显示时间
void syncNTP() {
    configTime(8 * 3600, 0, "ntp.aliyun.com", "pool.ntp.org");
    Serial.print("🕐 同步 NTP 时间");
    time_t now = 0;
    int retry = 0;
    while (now < 1000000000L && retry < NTP_TIMEOUT_S * 2) {
        delay(500);
        esp_task_wdt_reset();
        Serial.print(".");
        time(&now);
        retry++;
    }
    if (now > 1000000000L) {
        Serial.println(" ✅ 时间戳: " + String((long)now));
    } else {
        Serial.println(" ⚠️ 同步超时，涂鸦平台可能无法连接");
    }
}

// =====================================================
// MQTT
// =====================================================

void applyCommand(bool on, const String& source) {
    switchState    = on;
    lastCommandLog = source + " = " + (on ? "ON" : "OFF");
    pendingReport  = true;   // 回报状态放到主循环，避免在回调里发布

    Serial.println("📨 指令 [" + source + "]: " + String(on ? "ON" : "OFF"));
    if (on) {
        pendingBLEStart = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
        pendingBLEStop = true;
        digitalWrite(STATUS_LED_PIN, LOW);
    }
}

// 把一段文本解释成开/关。识别不出来返回 false
static bool textToBool(const String& raw, const char* onText, const char* offText, bool& out) {
    String v = raw;
    v.trim();
    v.toLowerCase();
    if (v.length() == 0) return false;

    String on  = String(onText);  on.toLowerCase();
    String off = String(offText); off.toLowerCase();

    if (v == on  || v == "on"  || v == "1" || v == "true")  { out = true;  return true; }
    if (v == off || v == "off" || v == "0" || v == "false") { out = false; return true; }
    return false;
}

// 通用模式：按点号路径从 JSON 里取值，再解释成开/关
static bool jsonToBool(const byte* payload, unsigned int len, const char* path, bool& out) {
    JsonDocument doc;
    if (deserializeJson(doc, payload, len)) return false;

    JsonVariantConst cur = doc.as<JsonVariantConst>();
    String p = String(path);
    int start = 0;
    while (true) {
        int dot = p.indexOf('.', start);
        String seg = (dot < 0) ? p.substring(start) : p.substring(start, dot);
        if (seg.length() == 0) return false;
        cur = cur[seg.c_str()];
        if (dot < 0) break;
        start = dot + 1;
    }

    if (cur.isNull()) return false;
    if (cur.is<bool>())        { out = cur.as<bool>();        return true; }
    if (cur.is<int>())         { out = cur.as<int>() != 0;    return true; }
    if (cur.is<float>())       { out = cur.as<float>() != 0;  return true; }
    if (cur.is<const char*>()) {
        return textToBool(String(cur.as<const char*>()), gen_on_text, gen_off_text, out);
    }
    return false;
}

// 涂鸦：{"data":{"switch_1":{"value":true}}} 或 {"data":{"5346":true}}
static void handleTuyaMessage(const byte* payload, unsigned int len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, len);
    if (err) {
        Serial.println("⚠️ JSON 解析失败: " + String(err.c_str()));
        return;
    }

    String msgId = doc["msgId"] | "";
    JsonObject data = doc["data"].as<JsonObject>();
    if (data.isNull()) return;

    auto extractBool = [](JsonVariant v) -> bool {
        if (v.is<JsonObject>()) return v["value"].as<bool>();
        return v.as<bool>();
    };

    // 优先匹配已知标识符，找不到时遍历所有 DP 取第一个布尔值
    bool   found = false, switchOn = false;
    String matchedKey = "";

    if (strlen(learned_dp_key) > 0 && !data[learned_dp_key].isNull()) {
        switchOn   = extractBool(data[learned_dp_key]);
        matchedKey = String(learned_dp_key);
        found      = true;
    } else {
        for (JsonPair kv : data) {
            JsonVariant v = kv.value();
            if (v.is<bool>() || (v.is<JsonObject>() && !v["value"].isNull())) {
                switchOn   = extractBool(v);
                matchedKey = kv.key().c_str();
                found      = true;
                break;
            }
        }
    }
    if (!found) return;

    // 记住云端实际使用的标识符，后续上报沿用它
    if (matchedKey.length() > 0 && matchedKey != String(learned_dp_key)) {
        strncpy(learned_dp_key, matchedKey.c_str(), sizeof(learned_dp_key) - 1);
        learned_dp_key[sizeof(learned_dp_key) - 1] = '\0';
        Serial.println("🔎 已识别 DP 标识符: " + matchedKey);
    }

    applyCommand(switchOn, matchedKey);
    if (msgId.length() > 0) sendTuyaSetResponse(msgId);
}

void mqttCallback(char* topicStr, byte* payload, unsigned int len) {
    String msg = "";
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];

    // 上报到同一个主题时 broker 可能把消息推回来。只有「内容和刚发出去的完全一致
    // 且在时间窗内」才当作回环丢弃，免得误吞掉窗口期里的真实指令。
    if (echoSuppressUntil > millis() && lastPublished.length() > 0 && msg == lastPublished) {
        Serial.println("↩️ 忽略自身上报的回环消息");
        return;
    }

    if (platform == PLATFORM_TUYA) {
        handleTuyaMessage(payload, len);
        return;
    }

    bool on = false;
    if (platform == PLATFORM_GENERIC && strlen(gen_json_key) > 0) {
        if (!jsonToBool(payload, len, gen_json_key, on)) {
            Serial.println("⚠️ 按路径 " + String(gen_json_key) + " 取不到开关值");
            return;
        }
        applyCommand(on, String(gen_json_key));
        return;
    }

    const char* onText  = (platform == PLATFORM_GENERIC) ? gen_on_text  : "on";
    const char* offText = (platform == PLATFORM_GENERIC) ? gen_off_text : "off";
    if (!textToBool(msg, onText, offText, on)) {
        Serial.println("⚠️ 无法识别的消息内容: " + msg);
        return;
    }
    applyCommand(on, String(topicStr));
}

void sendTuyaSetResponse(const String& msgId) {
    String topic = "tylink/" + String(tuya_device_id) + "/thing/property/set_response";
    JsonDocument doc;
    doc["msgId"] = msgId;
    doc["time"]  = (long)time(nullptr) * 1000;
    doc["code"]  = 0;
    String payload;
    serializeJson(doc, payload);
    mqtt.publish(topic.c_str(), payload.c_str());
}

void reportState(bool on) {
    if (!mqtt.connected()) return;

    switch (platform) {
        case PLATFORM_BEMFA: {
            if (strlen(bemfa_topic) == 0) return;
            // 巴法云收发同一个主题，记下内容以便识别回环
            lastPublished     = String(on ? "on" : "off");
            echoSuppressUntil = millis() + ECHO_SUPPRESS_MS;
            mqtt.publish(bemfa_topic, lastPublished.c_str());
            break;
        }
        case PLATFORM_TUYA: {
            // 标识符未知时跳过——涂鸦要求 data 的 key 与物模型一致
            if (strlen(learned_dp_key) == 0) return;
            String topic = "tylink/" + String(tuya_device_id) + "/thing/property/report";
            JsonDocument doc;
            doc["msgId"] = String(millis());
            doc["time"]  = (long)time(nullptr) * 1000;
            JsonObject data = doc["data"].to<JsonObject>();
            data[learned_dp_key]["value"] = on;
            String payload;
            serializeJson(doc, payload);
            if (!mqtt.publish(topic.c_str(), payload.c_str())) {
                Serial.println("⚠️ 属性上报失败，可能连接已断开");
            }
            break;
        }
        case PLATFORM_GENERIC: {
            if (strlen(gen_pub_topic) == 0) return;
            if (strcmp(gen_pub_topic, gen_sub_topic) == 0) {
                lastPublished     = String(on ? gen_on_text : gen_off_text);
                echoSuppressUntil = millis() + ECHO_SUPPRESS_MS;
            }
            mqtt.publish(gen_pub_topic, on ? gen_on_text : gen_off_text);
            break;
        }
    }
}

bool mqttConnect() {
    if (!platformConfigured())            return false;
    if (WiFi.status() != WL_CONNECTED)    return false;

    const char* host   = "";
    uint16_t    port   = 1883;
    bool        useTls = false;
    String      clientId, user, pass, subTopic;

    switch (platform) {
        case PLATFORM_BEMFA:
            host = BEMFA_HOST; port = BEMFA_PORT; useTls = false;
            clientId = String(bemfa_uid);   // 巴法云：ClientId = 私钥，无需用户名密码
            subTopic = String(bemfa_topic);
            break;

        case PLATFORM_TUYA: {
            time_t now = time(nullptr);
            if (now < 1000000000L) {
                Serial.println("⚠️ NTP 未同步，跳过涂鸦连接");
                return false;
            }
            // 涂鸦要求 10 位秒级时间戳，设备时间偏差过大会认证失败
            String T = String((long)now);
            host = tuya_host; port = TUYA_PORT; useTls = true;
            clientId = "tuyalink_" + String(tuya_device_id);
            user     = String(tuya_device_id)
                       + "|signMethod=hmacSha256,timestamp=" + T
                       + ",secureMode=1,accessType=1";
            pass     = hmacSha256(tuya_device_secret,
                       "deviceId=" + String(tuya_device_id)
                       + ",timestamp=" + T + ",secureMode=1,accessType=1");
            subTopic = "tylink/" + String(tuya_device_id) + "/thing/property/set";
            break;
        }

        case PLATFORM_GENERIC:
            host = gen_host; port = (uint16_t)gen_port; useTls = gen_tls;
            clientId = strlen(gen_client_id)
                       ? String(gen_client_id)
                       : "esp32c3-wake-" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF), HEX);
            user     = String(gen_user);
            pass     = String(gen_pass);
            subTopic = String(gen_sub_topic);
            break;

        default:
            return false;
    }

    if (useTls) {
        tlsClient.setInsecure();   // 跳过证书校验（DIY 场景适用）
        mqtt.setClient(tlsClient);
    } else {
        mqtt.setClient(plainClient);
    }
    mqtt.setServer(host, port);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    mqtt.setBufferSize(1024);

    Serial.printf("🔗 连接 %s  %s:%u  TLS=%s  剩余堆=%u B\n",
                  platformName(platform), host, port,
                  useTls ? "是" : "否", ESP.getFreeHeap());

    bool ok = (user.length() > 0 || pass.length() > 0)
              ? mqtt.connect(clientId.c_str(), user.c_str(), pass.c_str())
              : mqtt.connect(clientId.c_str());
    lastMqttState = mqtt.state();

    if (!ok) {
        mqttFailCount++;
        Serial.printf("   ❌ 失败: %s  剩余堆=%u B\n",
                      mqttStateText(lastMqttState), ESP.getFreeHeap());
        return false;
    }

    mqttConnectCount++;
    mqtt.subscribe(subTopic.c_str());
    Serial.println("   ✅ 已连接，订阅: " + subTopic);

    // 上报当前状态，让云端把设备标记为在线
    reportState(switchState);
    lastHeartbeat = millis();
    return true;
}

void handleMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiAttempt > WIFI_RECONNECT_MS) {
            lastWifiAttempt = millis();
            Serial.println("📶 WiFi 断线，尝试重连...");
            WiFi.reconnect();
        }
        return;
    }

    if (!mqtt.connected()) {
        int state = mqtt.state();
        if (state != lastMqttState) {
            lastMqttState = state;
            Serial.printf("📴 MQTT 断开: %s  剩余堆=%u B\n",
                          mqttStateText(state), ESP.getFreeHeap());
        }
        if (millis() - lastMqttAttempt > MQTT_RECONNECT_MS) {
            lastMqttAttempt = millis();
            mqttConnect();
        }
        return;
    }

    mqtt.loop();

    // 定期上报：既是保活，也让云端持续看到设备活跃
    if (millis() - lastHeartbeat > MQTT_HEARTBEAT_MS) {
        lastHeartbeat = millis();
        reportState(switchState);
    }
}

// =====================================================
// BLE 唤醒
// =====================================================

void initWakeupBLE() {
    if (bleInitialized) return;

    uint8_t mac[6];
    if (strlen(ble_mac) > 0) {
        sscanf(ble_mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    } else {
        memcpy(mac, baseMAC, 6);
    }
    mac[5] -= 2;
    esp_base_mac_addr_set(mac);

    uint8_t rb[6];
    esp_read_mac(rb, ESP_MAC_BT);
    Serial.printf("🔵 唤醒 BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  rb[0], rb[1], rb[2], rb[3], rb[4], rb[5]);

    size_t hexLen = strlen(ble_data);
    if (hexLen > 0 && hexLen % 2 == 0) {
        size_t len = hexLen / 2;
        if (len > sizeof(wake_adv_raw)) len = sizeof(wake_adv_raw);
        for (size_t i = 0; i < len; i++) {
            sscanf(ble_data + i * 2, "%2hhx", &wake_adv_raw[i]);
        }
        wake_adv_len = len;
    }

    BLEDevice::init(BLE_DEVICE_NAME);
    pWakeAdv = BLEDevice::getAdvertising();
    pWakeAdv->setMinInterval(0x0020);
    pWakeAdv->setMaxInterval(0x0040);
    pWakeAdv->setAdvertisementType(ADV_TYPE_NONCONN_IND);

    bleInitialized = true;
    Serial.println("✅ 唤醒 BLE 初始化完成");
}

void startBLEAdvertising() {
    if (!bleInitialized) initWakeupBLE();
    if (!pWakeAdv) return;

    pWakeAdv->stop();
    BLEAdvertisementData advData;
    String raw(reinterpret_cast<char*>(wake_adv_raw), wake_adv_len);
    advData.addData(raw);
    pWakeAdv->setAdvertisementData(advData);
    pWakeAdv->start();
    bleAdvStart = millis();
    Serial.println("📡 BLE 唤醒广播已开始 (1秒)...");
}

void stopBLEAdvertising() {
    if (bleInitialized && pWakeAdv) {
        pWakeAdv->stop();
        bleAdvStart = 0;
        Serial.println("🔇 BLE 唤醒广播已停止");
    }
}

void handleBLEAdvertising() {
    if (bleAdvStart > 0 && millis() - bleAdvStart >= BLE_WAKE_DURATION_MS) {
        stopBLEAdvertising();
    }
}

// =====================================================
// 按键 / LED
// =====================================================

void updateStatusLED() {
    unsigned long interval;
    switch (current_status) {
        case STATUS_CONFIGAP:        interval = 300;  break;
        case STATUS_WIFI_CONNECTING: interval = 500;  break;
        case STATUS_CONNECTED:       interval = 2000; break;
        default:                     interval = 1000; break;
    }
    if (millis() - lastLedToggle >= interval) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastLedToggle = millis();
    }
}

void checkButton() {
    static unsigned long lastPress  = 0;
    static bool          btnPressed = false;

    bool pressed = (digitalRead(TRIGGER_PIN) == LOW);

    if (pressed && !btnPressed) {
        if (millis() - lastPress > BUTTON_DEBOUNCE_MS) {
            btnPressed = true;
            lastPress  = millis();
        }
    } else if (!pressed && btnPressed) {
        unsigned long duration = millis() - lastPress;
        btnPressed = false;

        if (duration > LONG_PRESS_MS) {
            Serial.println("🔄 长按(>3s): 出厂重置，清除所有配置...");
            if (prefs.begin("config", false)) {
                prefs.clear();
                prefs.end();
            }
            safeRestart("出厂重置完成");
        } else {
            if (current_status == STATUS_CONFIGAP) {
                Serial.println("🔘 短按: 重启配置热点");
                stopConfigAP();
                startConfigAP();
            } else {
                Serial.println("🔘 短按: 手动触发 BLE 唤醒广播 (测试)");
                startBLEAdvertising();
            }
        }
    }
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
    esp_base_mac_addr_set(baseMAC);

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  ESP32C3  BLE 唤醒小爱音箱 v" FIRMWARE_VERSION);
    Serial.println("  统一固件：多平台 MQTT + 网页 OTA");
    Serial.println("========================================");

    pinMode(TRIGGER_PIN,    INPUT_PULLUP);
    pinMode(LED_PIN,        OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(LED_PIN,        LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms     = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    loadConfig();

    if (strlen(wifi_ssid) == 0) {
        Serial.println("⚠️ 无 WiFi 配置，启动配置热点");
        startConfigAP();
    } else {
        connectWiFi();
        if (current_status == STATUS_CONNECTED) {
            syncNTP();
            if (MDNS.begin("esp32c3-wake")) {
                Serial.println("🌐 mDNS: http://esp32c3-wake.local");
            }
            registerLANRoutes();
            webServer.begin();
            String base = "http://" + WiFi.localIP().toString();
            Serial.println("🌐 配置页: " + base);
            Serial.println("🩺 状态页: " + base + "/status");
            Serial.println("⬆️ 升级页: " + base + "/update");

            // 先建立 MQTT 连接，再初始化 BLE：
            // 蓝牙协议栈会占用几十 KB 堆，握手阶段把内存留给 TLS 更稳
            if (platformConfigured()) {
                mqttConnect();
            } else {
                Serial.printf("⚠️ %s 参数不完整，请到配置页填写\n", platformName(platform));
            }
            Serial.printf("🧠 BLE 初始化前剩余堆: %u B\n", ESP.getFreeHeap());
            initWakeupBLE();
            Serial.printf("🧠 BLE 初始化后剩余堆: %u B\n", ESP.getFreeHeap());
        }
    }

    Serial.println("🚀 启动完成\n");
}

void loop() {
    esp_task_wdt_reset();
    webServer.handleClient();
    checkButton();
    updateStatusLED();
    handleBLEAdvertising();

    if (current_status == STATUS_CONNECTED) {
        handleMQTT();
    }

    if (pendingBLEStart) {
        pendingBLEStart = false;
        startBLEAdvertising();   // 未初始化时在内部惰性初始化 BLE
    }
    if (pendingBLEStop) {
        pendingBLEStop = false;
        stopBLEAdvertising();
    }
    if (pendingReport) {
        pendingReport = false;
        reportState(switchState);
    }

    delay(10);
}
