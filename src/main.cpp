/**
 * BLE 唤醒小爱音箱 (ESP32-C3)
 * 版本: 6.1 - 涂鸦(Tuya) IoT 平台 MQTT 方案（TuyaLink 标准协议）
 *
 * 首次配置流程：
 *   1. 上电后 LED 快闪，设备开启配置热点 "ESP32_BLE_Wake"（密码 12345678）
 *   2. 手机连接热点 → 浏览器访问 192.168.4.1，填写 WiFi 账号密码
 *   3. 设备连上 WiFi 后，访问 http://esp32c3-wake.local 填写涂鸦参数
 *
 * 涂鸦参数获取：
 *   iot.tuya.com → 产品开发 → 设备管理 → 设备详情 → 设备证书
 *                 （DeviceID + DeviceSecret）
 *   新版设备详情页不再展示 DP 标识符，因此本版本的「DP 标识符」为选填：
 *   留空时固件会从云端下发的第一条指令中自动学习，并用它上报状态。
 *   数据中心必须与项目所在区域一致，可在配置页选择：
 *     中国 m1.tuyacn.com / 美西 m1.tuyaus.com / 欧洲 m1.tuyaeu.com / 印度 m1.tuyain.com
 *
 * 诊断：连上 WiFi 后访问 http://esp32c3-wake.local/status 查看
 *       WiFi / NTP / MQTT 状态、剩余堆内存和最近一条云端指令。
 *
 * 接入涂鸦智能 App / 米家：
 *   在涂鸦 IoT 平台配置产品后，用涂鸦智能 App 即可控制；
 *   也可通过 Home Assistant Tuya 插件或 Matter Bridge 接入米家。
 *
 * 日常使用：
 *   App"开" → MQTT set → BLE 广播唤醒小爱音箱
 *   App"关" → MQTT set → 停止广播
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
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <esp_mac.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "mbedtls/md.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "time.h"

// ==================== 引脚定义 ====================
#define TRIGGER_PIN    9
#define LED_PIN        12
#define STATUS_LED_PIN 13

// ==================== 时间常量 ====================
#define WDT_TIMEOUT_SECONDS  180
#define BUTTON_DEBOUNCE_MS   50
#define LONG_PRESS_MS        3000
#define BLE_WAKE_DURATION_MS 1000
#define MQTT_RECONNECT_MS    8000
#define WIFI_TIMEOUT_MS      30000
#define WIFI_RECONNECT_MS    15000
#define NTP_TIMEOUT_S        20
#define MQTT_SOCKET_TIMEOUT_S 20
#define MQTT_HEARTBEAT_MS    45000   // 定期上报属性，帮助云端维持在线判定

// ==================== 配置热点 ====================
#define SETUP_AP_SSID     "ESP32_BLE_Wake"
#define SETUP_AP_PASSWORD "12345678"

// ==================== 涂鸦云 MQTT ====================
// 接入地址由项目所在数据中心决定，选错会认证失败（state=5）
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
char tuya_device_id[25]     = "";   // 涂鸦设备 ID
char tuya_device_secret[33] = "";   // 涂鸦设备密钥
char tuya_dp_key[33]        = "";   // 开关 DP 标识符（留空 = 自动识别）
char tuya_host[41]          = DEFAULT_TUYA_HOST; // 接入地址（数据中心）
char ble_mac[19]            = "";
char ble_data[65]           = "";

// 运行时状态：从云端下发指令中学到的 DP 标识符，用于上报
char          learned_dp_key[33] = "";
String        lastCommandLog     = "";   // 最近一条云端指令，供 /status 查看
int           lastMqttState      = -100; // 最近一次 MQTT 连接返回码
unsigned long lastHeartbeat      = 0;
uint32_t      mqttConnectCount   = 0;
uint32_t      mqttFailCount      = 0;

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

// ==================== 跨任务标志 ====================
volatile bool pendingBLEStart = false;
volatile bool pendingBLEStop  = false;
volatile bool pendingReport   = false;
bool          switchState     = false;   // 云端最后一次下发的开关状态

// ==================== 重连计时 ====================
unsigned long lastMqttAttempt = 0;
unsigned long lastWifiAttempt = 0;

// ==================== 对象 ====================
Preferences       prefs;
WebServer         webServer(80);
WiFiClientSecure  wifiClient;
PubSubClient      mqtt(wifiClient);
bool              apActive = false;

// ==================== 函数声明 ====================
void loadConfig();
void saveWiFiConfig(const String& ssid, const String& pass);
void saveServiceConfig(const String& deviceId, const String& secret,
                       const String& dpKey,     const String& host,
                       const String& mac,       const String& data);
void startConfigAP();
void stopConfigAP();
void connectWiFi();
void syncNTP();
String hmacSha256(const char* key, const String& data);
bool mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void sendSetResponse(const String& msgId);
void reportSwitchState(bool on);
void handleMQTT();
void initWakeupBLE();
void startBLEAdvertising();
void stopBLEAdvertising();
void handleBLEAdvertising();
void checkButton();
void updateStatusLED();
void safeRestart(const char* reason);
String buildWiFiPage();
String buildServicePage();
String buildStatusPage();
const char* mqttStateText(int state);
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
    prefs.getString("wifi_ssid",    "").toCharArray(wifi_ssid,          sizeof(wifi_ssid));
    prefs.getString("wifi_pass",    "").toCharArray(wifi_pass,          sizeof(wifi_pass));
    prefs.getString("tuya_did",     "").toCharArray(tuya_device_id,     sizeof(tuya_device_id));
    prefs.getString("tuya_secret",  "").toCharArray(tuya_device_secret, sizeof(tuya_device_secret));
    prefs.getString("tuya_dp",      "").toCharArray(tuya_dp_key,        sizeof(tuya_dp_key));
    prefs.getString("tuya_host", DEFAULT_TUYA_HOST).toCharArray(tuya_host, sizeof(tuya_host));
    prefs.getString("ble_mac",  DEFAULT_BLE_MAC).toCharArray(ble_mac,   sizeof(ble_mac));
    prefs.getString("ble_data", DEFAULT_BLE_DATA).toCharArray(ble_data, sizeof(ble_data));
    prefs.end();

    if (strlen(tuya_host) == 0) strcpy(tuya_host, DEFAULT_TUYA_HOST);
    strncpy(learned_dp_key, tuya_dp_key, sizeof(learned_dp_key) - 1);

    Serial.println("✅ 配置已加载 - WiFi: " + String(wifi_ssid)
                   + "  涂鸦设备ID: " + String(tuya_device_id)
                   + "  接入地址: " + String(tuya_host)
                   + "  DP: " + (strlen(tuya_dp_key) ? String(tuya_dp_key) : String("自动识别")));
}

void saveWiFiConfig(const String& ssid, const String& pass) {
    if (!prefs.begin("config", false)) return;
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    Serial.println("💾 WiFi 配置已保存");
}

void saveServiceConfig(const String& deviceId, const String& secret,
                       const String& dpKey,     const String& host,
                       const String& mac,       const String& data) {
    if (!prefs.begin("config", false)) return;
    prefs.putString("tuya_did",    deviceId);
    prefs.putString("tuya_secret", secret);
    prefs.putString("tuya_dp",     dpKey);   // 允许留空，由固件自动识别
    prefs.putString("tuya_host",   host.length() > 0 ? host : DEFAULT_TUYA_HOST);
    prefs.putString("ble_mac",     mac.length()  > 0 ? mac   : DEFAULT_BLE_MAC);
    prefs.putString("ble_data",    data.length() > 0 ? data  : DEFAULT_BLE_DATA);
    prefs.end();
    Serial.println("💾 服务配置已保存");
}

// =====================================================
// Web 配置页
// =====================================================

// ---- AP 热点页：仅 WiFi 配置 ----
String buildWiFiPage() {
    return
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;padding:20px;max-width:420px;margin:auto;color:#333}"
        "h2{margin-bottom:4px}"
        ".sub{color:#888;font-size:.9em;margin:0 0 20px}"
        "label{display:block;margin-top:14px;font-weight:bold;font-size:.9em}"
        "input[type=text],input[type=password]{"
        "  width:100%;box-sizing:border-box;padding:9px;"
        "  border:1px solid #ddd;border-radius:6px;font-size:.95em;margin-top:4px}"
        "input[type=submit]{width:100%;background:#1989fa;color:#fff;border:none;"
        "  padding:12px;border-radius:8px;font-size:1em;margin-top:20px;cursor:pointer}"
        ".tip{font-size:.82em;color:#aaa;margin-top:6px}"
        "</style></head><body>"
        "<h2>WiFi 配置</h2>"
        "<p class='sub'>连接成功后，可通过局域网地址继续配置涂鸦云和 BLE 参数</p>"
        "<form method='POST' action='/save-wifi'>"
        "<label>WiFi 名称 (SSID)</label>"
        "<input type='text' name='ssid' value='" + String(wifi_ssid) + "' required "
        "placeholder='家庭 WiFi 名称（仅支持 2.4GHz）'>"
        "<label>WiFi 密码</label>"
        "<input type='password' name='pass' value='' "
        "placeholder='WiFi 密码（无密码留空）'>"
        "<p class='tip'>保存后设备将自动连接 WiFi，LED 由快闪变为慢闪即为成功</p>"
        "<input type='submit' value='保存并连接'>"
        "</form>"
        "</body></html>";
}

// ---- 局域网页：涂鸦云 + BLE 配置 ----
String buildServicePage() {
    bool tuyaOk = (strlen(tuya_device_id) > 0 && strlen(tuya_device_secret) > 0);

    String statusBar;
    if (!tuyaOk) {
        statusBar = "<div class='warn'>涂鸦云尚未配置，设备暂无法接收控制指令</div>";
    } else if (mqtt.connected()) {
        statusBar = "<div class='ok'>涂鸦云已连接 &nbsp; 设备ID: <b>"
                    + String(tuya_device_id) + "</b></div>";
    } else {
        statusBar = "<div class='warn'>涂鸦云未连接（" + String(mqttStateText(lastMqttState))
                    + "） &nbsp; 设备ID: <b>" + String(tuya_device_id) + "</b></div>";
    }

    String hostOptions = "";
    bool matched = false;
    for (size_t i = 0; i < TUYA_REGION_COUNT; i++) {
        bool sel = (strcmp(tuya_host, TUYA_REGIONS[i].host) == 0);
        if (sel) matched = true;
        hostOptions += "<option value='" + String(TUYA_REGIONS[i].host) + "'"
                       + (sel ? " selected" : "") + ">"
                       + String(TUYA_REGIONS[i].label) + " - "
                       + String(TUYA_REGIONS[i].host) + "</option>";
    }
    if (!matched) {
        hostOptions = "<option value='" + String(tuya_host) + "' selected>"
                      + String(tuya_host) + "</option>" + hostOptions;
    }

    return
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;padding:20px;max-width:460px;margin:auto;color:#333}"
        "h2{margin-bottom:4px}"
        "h3{margin:0 0 6px;font-size:1em;color:#555}"
        "label{display:block;margin-top:14px;font-weight:bold;font-size:.9em}"
        "input[type=text],select{"
        "  width:100%;box-sizing:border-box;padding:9px;background:#fff;"
        "  border:1px solid #ddd;border-radius:6px;font-size:.95em;margin-top:4px}"
        "input[type=submit]{width:100%;background:#1989fa;color:#fff;border:none;"
        "  padding:12px;border-radius:8px;font-size:1em;margin-top:20px;cursor:pointer}"
        "hr{border:none;border-top:1px solid #eee;margin:20px 0}"
        "a{color:#1989fa}"
        ".tip{font-size:.82em;color:#aaa;margin-top:5px;line-height:1.5}"
        ".ok{background:#f0f9eb;border:1px solid #b3e19d;border-radius:8px;"
        "  padding:10px 14px;margin-bottom:16px;font-size:.9em}"
        ".warn{background:#fff7e6;border:1px solid #ffd591;border-radius:8px;"
        "  padding:10px 14px;margin-bottom:16px;font-size:.9em;color:#874d00}"
        "</style></head><body>"
        "<h2>BLE 唤醒配置</h2>"
        + statusBar +
        "<form method='POST' action='/save'>"

        "<hr><h3>涂鸦云 IoT</h3>"
        "<p class='tip'>"
        "iot.tuya.com → 产品开发 → 设备管理 → 设备详情 → <b>设备证书</b><br>"
        "<b>数据中心必须与项目所在区域一致</b>，选错会认证失败"
        "</p>"
        "<label>设备 ID（Device ID）</label>"
        "<input type='text' name='did' value='" + String(tuya_device_id) + "' required "
        "placeholder='26c24759c1344038xxxxxx'>"
        "<label>设备密钥（Device Secret）</label>"
        "<input type='text' name='secret' value='" + String(tuya_device_secret) + "' required "
        "placeholder='XXXXXXXXXXXXXXXX'>"
        "<label>数据中心</label>"
        "<select name='host'>" + hostOptions + "</select>"
        "<label>开关 DP 标识符（选填）</label>"
        "<input type='text' name='dp' value='" + String(tuya_dp_key) + "' "
        "placeholder='留空则自动识别'>"
        "<p class='tip'>"
        "新版设备详情页不再显示标识符，留空即可——固件会从云端下发的第一条指令里"
        "自动学到它，并用同一个标识符回报状态。<br>"
        "如果设备详情里「绑定用户 / 绑定 APP」为空，说明还没在涂鸦智能 App 里绑定这台设备，"
        "此时 App 无法下发指令。"
        "</p>"

        "<hr><h3>BLE 唤醒目标</h3>"
        "<p class='tip'>留空使用默认值；如需唤醒特定音箱，填写其 BLE MAC 和广播数据</p>"
        "<label>目标设备 BLE MAC</label>"
        "<input type='text' name='mac' value='" + String(ble_mac) + "' "
        "placeholder='78:81:8c:06:9a:c4' "
        "pattern='^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$'>"
        "<label>BLE 广播数据（HEX，不含空格）</label>"
        "<input type='text' name='data' value='" + String(ble_data) + "' "
        "placeholder='0201061BFF...' pattern='^[0-9a-fA-F]*$'>"

        "<input type='submit' value='保存并重启'>"
        "</form>"
        "</body></html>";
}

// ---- MQTT 返回码释义（PubSubClient state()）----
const char* mqttStateText(int state) {
    switch (state) {
        case -100: return "尚未尝试连接";
        case -4: return "-4 服务器无响应，可能是地址/端口不通";
        case -3: return "-3 连接被断开";
        case -2: return "-2 TCP/TLS 建连失败，检查网络与数据中心地址";
        case -1: return "-1 已主动断开";
        case  0: return "0 已连接";
        case  1: return "1 协议版本不被接受";
        case  2: return "2 ClientID 被拒绝";
        case  3: return "3 服务不可用";
        case  4: return "4 用户名或密码错误，检查 DeviceSecret 与设备时间";
        case  5: return "5 认证未通过，检查 DeviceID / Secret / 数据中心是否匹配";
        default: return "未知状态";
    }
}

// ---- 局域网诊断页 /status ----
String buildStatusPage() {
    time_t now = time(nullptr);
    struct tm tmv;
    char timeStr[32] = "未同步";
    if (now > 1000000000L) {
        localtime_r(&now, &tmv);
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tmv);
    }

    String dpShown = strlen(learned_dp_key) ? String(learned_dp_key) : String("尚未识别");

    String rows = "";
    auto row = [&rows](const String& k, const String& v) {
        rows += "<tr><td>" + k + "</td><td>" + v + "</td></tr>";
    };

    row("WiFi", WiFi.status() == WL_CONNECTED
                ? "已连接 " + WiFi.SSID() + " (" + WiFi.localIP().toString()
                  + ", RSSI " + String(WiFi.RSSI()) + " dBm)"
                : "未连接");
    row("NTP 时间", String(timeStr));
    row("涂鸦地址", String(tuya_host) + ":" + String(TUYA_PORT));
    row("设备 ID", String(tuya_device_id));
    row("MQTT", mqtt.connected() ? "已连接" : "未连接");
    row("最近返回码", String(mqttStateText(lastMqttState)));
    row("连接成功 / 失败次数", String(mqttConnectCount) + " / " + String(mqttFailCount));
    row("DP 标识符", dpShown);
    row("最近一条指令", lastCommandLog.length() ? lastCommandLog : "无");
    row("BLE", bleInitialized ? "已初始化" : "未初始化");
    row("剩余堆内存", String(ESP.getFreeHeap()) + " B（最低 "
                     + String(ESP.getMinFreeHeap()) + " B）");
    row("运行时长", String(millis() / 1000) + " s");

    return
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='5'>"
        "<style>"
        "body{font-family:sans-serif;padding:20px;max-width:520px;margin:auto;color:#333}"
        "table{width:100%;border-collapse:collapse;font-size:.9em}"
        "td{padding:8px 6px;border-bottom:1px solid #eee;vertical-align:top}"
        "td:first-child{color:#888;width:38%}"
        "a{color:#1989fa;margin-right:14px}"
        "</style></head><body>"
        "<h2>运行状态</h2>"
        "<table>" + rows + "</table>"
        "<p><a href='/'>配置页</a><a href='/wake'>手动触发 BLE 唤醒</a></p>"
        "<p style='color:#aaa;font-size:.8em'>本页每 5 秒自动刷新</p>"
        "</body></html>";
}

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
            "<small>连接成功后请访问 <b>http://esp32c3-wake.local</b> 配置涂鸦云参数</small>"
            "</p>");
        delay(800);
        ESP.restart();
    });
}

void registerLANRoutes() {
    webServer.on("/", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildServicePage());
    });

    webServer.on("/status", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildStatusPage());
    });

    // 手动触发一次 BLE 唤醒广播，用于不依赖涂鸦云的联调
    webServer.on("/wake", HTTP_GET, []() {
        pendingBLEStart = true;
        webServer.send(200, "text/html; charset=utf-8",
            "<meta charset='UTF-8'><p style='font-family:sans-serif;padding:20px'>"
            "📡 已触发 BLE 唤醒广播，<a href='/status'>返回状态页</a></p>");
    });

    webServer.on("/save", HTTP_POST, []() {
        String did    = webServer.arg("did");
        String secret = webServer.arg("secret");
        String dp     = webServer.arg("dp");
        String host   = webServer.arg("host");
        String mac    = webServer.arg("mac");
        String data   = webServer.arg("data");

        if (did.length() == 0 || secret.length() == 0) {
            webServer.send(400, "text/html; charset=utf-8",
                "<meta charset='UTF-8'>"
                "<p style='font-family:sans-serif;padding:20px'>❌ 设备 ID 和设备密钥为必填项</p>");
            return;
        }
        saveServiceConfig(did, secret, dp, host, mac, data);
        webServer.send(200, "text/html; charset=utf-8",
            "<meta charset='UTF-8'>"
            "<p style='font-family:sans-serif;padding:20px'>✅ 已保存，设备重启中...</p>");
        delay(800);
        ESP.restart();
    });
}

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

// =====================================================
// WiFi 连接
// =====================================================

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

// =====================================================
// NTP 时间同步（涂鸦 HMAC 签名需要时间戳）
// =====================================================

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
        Serial.println(" ⚠️ 同步超时，MQTT 可能无法连接");
    }
}

// =====================================================
// HMAC-SHA256（涂鸦签名认证）
// =====================================================

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

// =====================================================
// MQTT（涂鸦云）
// =====================================================

void mqttCallback(char* topicStr, byte* payload, unsigned int len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, len);
    if (err) {
        Serial.println("⚠️ JSON 解析失败: " + String(err.c_str()));
        return;
    }

    String msgId = doc["msgId"] | "";
    JsonObject data = doc["data"].as<JsonObject>();
    if (data.isNull()) return;

    // 从 DP 值中提取布尔值：兼容两种格式
    //   标准物模型: {"switch_1": {"value": true}}
    //   简单格式:   {"5346": true}
    auto extractBool = [](JsonVariant v) -> bool {
        if (v.is<JsonObject>()) return v["value"].as<bool>();
        return v.as<bool>();
    };

    // 优先匹配配置的 DP 标识符，找不到时遍历所有 DP 取第一个布尔值
    bool found = false;
    bool switchOn = false;
    String matchedKey = "";

    if (!data[tuya_dp_key].isNull()) {
        switchOn  = extractBool(data[tuya_dp_key]);
        matchedKey = String(tuya_dp_key);
        found = true;
    } else {
        for (JsonPair kv : data) {
            JsonVariant v = kv.value();
            if (v.is<bool>() || (v.is<JsonObject>() && !v["value"].isNull())) {
                switchOn   = extractBool(v);
                matchedKey = kv.key().c_str();
                found = true;
                break;
            }
        }
    }

    if (found) {
        // 记住云端实际使用的标识符，后续上报沿用它
        if (matchedKey.length() > 0 && matchedKey != String(learned_dp_key)) {
            strncpy(learned_dp_key, matchedKey.c_str(), sizeof(learned_dp_key) - 1);
            learned_dp_key[sizeof(learned_dp_key) - 1] = '\0';
            Serial.println("🔎 已识别 DP 标识符: " + matchedKey);
        }
        lastCommandLog = matchedKey + " = " + String(switchOn ? "ON" : "OFF");
        switchState    = switchOn;
        pendingReport  = true;   // 回报状态放到主循环，避免在回调里发布

        Serial.println("📨 涂鸦指令 [" + matchedKey + "]: "
                       + String(switchOn ? "ON" : "OFF"));
        if (switchOn) {
            pendingBLEStart = true;
            digitalWrite(STATUS_LED_PIN, HIGH);
        } else {
            pendingBLEStop = true;
            digitalWrite(STATUS_LED_PIN, LOW);
        }
        if (msgId.length() > 0) sendSetResponse(msgId);
    }
}

void sendSetResponse(const String& msgId) {
    String topic = "tylink/" + String(tuya_device_id)
                   + "/thing/property/set_response";
    JsonDocument doc;
    doc["msgId"] = msgId;
    doc["time"]  = (long)time(nullptr) * 1000;
    doc["code"]  = 0;
    String payload;
    serializeJson(doc, payload);
    mqtt.publish(topic.c_str(), payload.c_str());
}

// 上报开关状态。标识符未知时跳过——涂鸦要求 data 的 key 与物模型一致
void reportSwitchState(bool on) {
    if (!mqtt.connected() || strlen(learned_dp_key) == 0) return;

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
}

bool mqttConnect() {
    if (strlen(tuya_device_id) == 0 || strlen(tuya_device_secret) == 0) return false;
    if (WiFi.status() != WL_CONNECTED) return false;

    time_t now = time(nullptr);
    if (now < 1000000000L) {
        Serial.println("⚠️ NTP 未同步，跳过 MQTT 连接");
        return false;
    }

    // 涂鸦要求 10 位秒级时间戳，设备时间偏差过大会认证失败
    String T = String((long)now);

    // 涂鸦 MQTT 凭据生成
    String clientId = "tuyalink_" + String(tuya_device_id);
    String username = String(tuya_device_id)
                      + "|signMethod=hmacSha256,timestamp=" + T
                      + ",secureMode=1,accessType=1";
    String signData = "deviceId=" + String(tuya_device_id)
                      + ",timestamp=" + T
                      + ",secureMode=1,accessType=1";
    String password = hmacSha256(tuya_device_secret, signData);

    wifiClient.setInsecure(); // 跳过 TLS 证书验证（DIY 场景适用）
    mqtt.setServer(tuya_host, TUYA_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    mqtt.setBufferSize(1024);

    Serial.printf("🔗 连接涂鸦云 %s:%d  时间戳=%s  剩余堆=%u B\n",
                  tuya_host, TUYA_PORT, T.c_str(), ESP.getFreeHeap());

    bool ok = mqtt.connect(clientId.c_str(), username.c_str(), password.c_str());
    lastMqttState = mqtt.state();

    if (!ok) {
        mqttFailCount++;
        Serial.printf("   ❌ 失败: %s  剩余堆=%u B\n",
                      mqttStateText(lastMqttState), ESP.getFreeHeap());
        return false;
    }

    mqttConnectCount++;
    String subTopic = "tylink/" + String(tuya_device_id) + "/thing/property/set";
    mqtt.subscribe(subTopic.c_str());
    Serial.println("   ✅ 已连接，订阅: " + subTopic);

    // 上报当前属性，让涂鸦云把设备标记为在线（米家同步依赖此步骤）
    if (strlen(learned_dp_key) > 0) {
        reportSwitchState(switchState);
        Serial.println("   📤 已上报初始状态 [" + String(learned_dp_key) + "]");
    } else {
        Serial.println("   ℹ️ DP 标识符未知，等待云端下发第一条指令后自动识别");
    }
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

    // 定期上报属性：既是保活，也让云端持续看到设备活跃
    if (millis() - lastHeartbeat > MQTT_HEARTBEAT_MS) {
        lastHeartbeat = millis();
        reportSwitchState(switchState);
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
// 工具函数
// =====================================================

void safeRestart(const char* reason) {
    Serial.println("🔄 重启: " + String(reason));
    prefs.end();
    delay(500);
    ESP.restart();
}

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
    Serial.println("  ESP32C3  BLE 唤醒小爱音箱 v6.1");
    Serial.println("  方案: 涂鸦 IoT TuyaLink MQTT");
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
            Serial.println("🌐 配置页: http://" + WiFi.localIP().toString());
            Serial.println("🩺 状态页: http://" + WiFi.localIP().toString() + "/status");

            // 先建立 TLS/MQTT 连接，再初始化 BLE：
            // 蓝牙协议栈会占用几十 KB 堆，握手阶段把内存留给 TLS 更稳
            if (strlen(tuya_device_id) > 0 && strlen(tuya_device_secret) > 0) {
                mqttConnect();
            } else {
                Serial.println("⚠️ 涂鸦云未配置，请访问配置页填写设备 ID 和密钥");
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
        reportSwitchState(switchState);
    }

    delay(10);
}
