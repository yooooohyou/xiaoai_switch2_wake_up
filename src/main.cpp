/**
 * BLE 唤醒小爱音箱 (ESP32-C3)
 * 版本: 5.0 - 巴法云 MQTT 方案（无需 Matter 认证）
 *
 * 首次配置流程：
 *   1. 上电后 LED 快闪，设备开启配置热点 "ESP32_BLE_Wake"（密码 12345678）
 *   2. 手机连接热点 → 浏览器访问 192.168.4.1
 *   3. 填写：家庭 WiFi 账号/密码、巴法云 UID（私钥）、主题名、BLE 参数
 *   4. 点击"保存并重启"，设备自动接入家庭 WiFi 并连接巴法云
 *
 * 接入米家方法：
 *   1. 登录 bemfa.com → 控制台 → 新建主题（类型选"灯"或"开关"）
 *   2. 在巴法云 App 或米家"巴法云"插件中授权，即可用米家/小爱控制
 *
 * 日常使用：
 *   米家"开" / 小爱"打开" → MQTT "on" → BLE 广播唤醒小爱音箱
 *   米家"关" / 小爱"关闭" → MQTT "off" → 停止广播
 *
 * 按键操作：
 *   短按 → 配置模式：重启热点；正常模式：手动触发 BLE 广播测试
 *   长按(>3s) → 出厂重置，清除所有配置
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <esp_mac.h>
#include <PubSubClient.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// ==================== 引脚定义 ====================
#define TRIGGER_PIN    9
#define LED_PIN        12
#define STATUS_LED_PIN 13

// ==================== 时间常量 ====================
#define WDT_TIMEOUT_SECONDS  180
#define BUTTON_DEBOUNCE_MS   50
#define LONG_PRESS_MS        3000
#define BLE_WAKE_DURATION_MS 1000
#define MQTT_RECONNECT_MS    5000
#define WIFI_TIMEOUT_MS      30000
#define WIFI_RECONNECT_MS    15000

// ==================== 配置热点 ====================
#define SETUP_AP_SSID     "ESP32_BLE_Wake"
#define SETUP_AP_PASSWORD "12345678"

// ==================== 巴法云 MQTT ====================
#define BEMFA_HOST "bemfa.com"
#define BEMFA_PORT 9501

// ==================== BLE ====================
#define BLE_DEVICE_NAME "ESP32C3_BLE_Beacon"

// ==================== 默认 BLE 参数 ====================
const char* DEFAULT_BLE_MAC  = "78:81:8c:06:9a:c4";
const char* DEFAULT_BLE_DATA = "0201061BFF53050100037E0566200001816D60168C81780F00000000000000";

// ==================== 配置缓冲区（从 NVS 加载）====================
char wifi_ssid[33]    = "";
char wifi_pass[65]    = "";
char bemfa_uid[65]    = "";   // 巴法云私钥
char bemfa_topic[33]  = "";   // 主题名，如 "light001"
char ble_mac[19]      = "";
char ble_data[65]     = "";

// ==================== BLE 广播数据 ====================
uint8_t baseMAC[6] = {0x78, 0x81, 0x8c, 0x06, 0x9a, 0xc4};

// 最大 32 字节，默认 28 字节（可由用户配置覆盖）
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
    STATUS_CONFIGAP,       // 等待 Web 配置
    STATUS_WIFI_CONNECTING,// 正在连接 WiFi
    STATUS_CONNECTED       // WiFi + MQTT 正常运行
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

// ==================== 重连计时 ====================
unsigned long lastMqttAttempt = 0;
unsigned long lastWifiAttempt = 0;

// ==================== 对象 ====================
Preferences  prefs;
WebServer    webServer(80);
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
bool         apActive = false;

// ==================== 函数声明 ====================
void loadConfig();
void saveConfig(const String& ssid, const String& pass,
                const String& uid,  const String& topic,
                const String& mac,  const String& data);
void startConfigAP();
void stopConfigAP();
void connectWiFi();
bool mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int len);
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
void registerAPRoutes();
void registerLANRoutes();

// =====================================================
// 配置加载 / 保存
// =====================================================

void loadConfig() {
    if (!prefs.begin("config", true)) {
        prefs.end();
        strcpy(ble_mac,  DEFAULT_BLE_MAC);
        strcpy(ble_data, DEFAULT_BLE_DATA);
        return;
    }
    prefs.getString("wifi_ssid",    "").toCharArray(wifi_ssid,   sizeof(wifi_ssid));
    prefs.getString("wifi_pass",    "").toCharArray(wifi_pass,   sizeof(wifi_pass));
    prefs.getString("bemfa_uid",    "").toCharArray(bemfa_uid,   sizeof(bemfa_uid));
    prefs.getString("bemfa_topic",  "").toCharArray(bemfa_topic, sizeof(bemfa_topic));
    prefs.getString("ble_mac",  DEFAULT_BLE_MAC).toCharArray(ble_mac,   sizeof(ble_mac));
    prefs.getString("ble_data", DEFAULT_BLE_DATA).toCharArray(ble_data, sizeof(ble_data));
    prefs.end();

    Serial.println("✅ 配置已加载 - WiFi: " + String(wifi_ssid)
                   + "  Bemfa主题: " + String(bemfa_topic));
}

void saveWiFiConfig(const String& ssid, const String& pass) {
    if (!prefs.begin("config", false)) return;
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    Serial.println("💾 WiFi 配置已保存");
}

void saveServiceConfig(const String& uid,  const String& topic,
                       const String& mac,  const String& data) {
    if (!prefs.begin("config", false)) return;
    prefs.putString("bemfa_uid",   uid);
    prefs.putString("bemfa_topic", topic);
    prefs.putString("ble_mac",     mac.length()  > 0 ? mac  : DEFAULT_BLE_MAC);
    prefs.putString("ble_data",    data.length() > 0 ? data : DEFAULT_BLE_DATA);
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
        "<p class='sub'>连接成功后，可通过局域网地址继续配置巴法云和 BLE 参数</p>"
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

// ---- 局域网页：巴法云 + BLE 配置 ----
String buildServicePage() {
    bool mqttOk = (strlen(bemfa_uid) > 0 && strlen(bemfa_topic) > 0);

    String statusBar = mqttOk
        ? "<div class='ok'>巴法云已配置 &nbsp; 主题: <b>" + String(bemfa_topic) + "</b></div>"
        : "<div class='warn'>巴法云尚未配置，设备暂无法接收米家指令</div>";

    return
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;padding:20px;max-width:460px;margin:auto;color:#333}"
        "h2{margin-bottom:4px}"
        "h3{margin:0 0 6px;font-size:1em;color:#555}"
        "label{display:block;margin-top:14px;font-weight:bold;font-size:.9em}"
        "input[type=text]{"
        "  width:100%;box-sizing:border-box;padding:9px;"
        "  border:1px solid #ddd;border-radius:6px;font-size:.95em;margin-top:4px}"
        "input[type=submit]{width:100%;background:#1989fa;color:#fff;border:none;"
        "  padding:12px;border-radius:8px;font-size:1em;margin-top:20px;cursor:pointer}"
        "hr{border:none;border-top:1px solid #eee;margin:20px 0}"
        ".tip{font-size:.82em;color:#aaa;margin-top:5px;line-height:1.5}"
        ".ok{background:#f0f9eb;border:1px solid #b3e19d;border-radius:8px;"
        "  padding:10px 14px;margin-bottom:16px;font-size:.9em}"
        ".warn{background:#fff7e6;border:1px solid #ffd591;border-radius:8px;"
        "  padding:10px 14px;margin-bottom:16px;font-size:.9em;color:#874d00}"
        "</style></head><body>"
        "<h2>BLE 唤醒配置</h2>"
        + statusBar +
        "<form method='POST' action='/save'>"

        "<hr><h3>巴法云 MQTT</h3>"
        "<p class='tip'>"
        "登录 <b>bemfa.com</b> → 个人中心复制私钥；控制台新建主题（类型选灯/开关）<br>"
        "在巴法云 App 内接入米家，即可用米家/小爱控制本设备"
        "</p>"
        "<label>巴法云私钥（UID）</label>"
        "<input type='text' name='uid' value='" + String(bemfa_uid) + "' required "
        "placeholder='请输入巴法云账号私钥'>"
        "<label>主题名称</label>"
        "<input type='text' name='topic' value='" + String(bemfa_topic) + "' required "
        "placeholder='例如 light001'>"

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

void registerAPRoutes() {
    // AP 模式：仅 WiFi 配置
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
            "<small>连接成功后请访问 <b>http://esp32c3-wake.local</b> 配置巴法云参数</small>"
            "</p>");
        delay(800);
        ESP.restart();
    });
}

void registerLANRoutes() {
    // 局域网模式：巴法云 + BLE 配置
    webServer.on("/", HTTP_GET, []() {
        webServer.send(200, "text/html; charset=utf-8", buildServicePage());
    });

    webServer.on("/save", HTTP_POST, []() {
        String uid   = webServer.arg("uid");
        String topic = webServer.arg("topic");
        String mac   = webServer.arg("mac");
        String data  = webServer.arg("data");

        if (uid.length() == 0 || topic.length() == 0) {
            webServer.send(400, "text/html; charset=utf-8",
                "<meta charset='UTF-8'>"
                "<p style='font-family:sans-serif;padding:20px'>❌ 巴法云私钥和主题名为必填项</p>");
            return;
        }
        saveServiceConfig(uid, topic, mac, data);
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
// MQTT（巴法云）
// =====================================================

void mqttCallback(char* topicStr, byte* payload, unsigned int len) {
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    msg.trim();
    msg.toLowerCase();

    Serial.println("📨 MQTT [" + String(topicStr) + "]: " + msg);

    if (msg == "on" || msg == "1") {
        pendingBLEStart = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
    } else if (msg == "off" || msg == "0") {
        pendingBLEStop = true;
        digitalWrite(STATUS_LED_PIN, LOW);
    }
}

bool mqttConnect() {
    if (strlen(bemfa_uid) == 0 || strlen(bemfa_topic) == 0) return false;
    if (WiFi.status() != WL_CONNECTED) return false;

    mqtt.setServer(BEMFA_HOST, BEMFA_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);

    Serial.print("🔗 连接巴法云 MQTT...");
    // 巴法云：ClientID = 私钥，无需用户名/密码
    if (mqtt.connect(bemfa_uid)) {
        mqtt.subscribe(bemfa_topic);
        Serial.println(" ✅ 已连接，订阅: " + String(bemfa_topic));
        return true;
    }
    Serial.println(" ❌ 失败 (state=" + String(mqtt.state()) + ")");
    return false;
}

void handleMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        // WiFi 断线，定期重连
        if (millis() - lastWifiAttempt > WIFI_RECONNECT_MS) {
            lastWifiAttempt = millis();
            Serial.println("📶 WiFi 断线，尝试重连...");
            WiFi.reconnect();
        }
        return;
    }

    if (!mqtt.connected()) {
        if (millis() - lastMqttAttempt > MQTT_RECONNECT_MS) {
            lastMqttAttempt = millis();
            mqttConnect();
        }
    } else {
        mqtt.loop();
    }
}

// =====================================================
// BLE 唤醒
// =====================================================

void initWakeupBLE() {
    if (bleInitialized) return;

    // 解析目标 MAC，BLE MAC = 目标MAC - 2
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

    // 解析 HEX 字符串广播数据
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
        case STATUS_CONFIGAP:        interval = 300;  break; // 快闪 - 等待配置
        case STATUS_WIFI_CONNECTING: interval = 500;  break; // 中闪 - 连接中
        case STATUS_CONNECTED:       interval = 2000; break; // 慢闪 - 正常运行
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
            // 长按：清除所有配置
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
                if (bleInitialized) startBLEAdvertising();
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
    Serial.println("  ESP32C3  BLE 唤醒小爱音箱 v5.0");
    Serial.println("  方案: 巴法云 MQTT（无需设备认证）");
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

    bool needConfig = (strlen(wifi_ssid) == 0 ||
                       strlen(bemfa_uid) == 0 ||
                       strlen(bemfa_topic) == 0);

    if (strlen(wifi_ssid) == 0) {
        Serial.println("⚠️ 无 WiFi 配置，启动配置热点");
        startConfigAP();
    } else {
        connectWiFi();
        if (current_status == STATUS_CONNECTED) {
            if (MDNS.begin("esp32c3-wake")) {
                Serial.println("🌐 mDNS: http://esp32c3-wake.local");
            }
            registerLANRoutes();
            webServer.begin();
            Serial.println("🌐 配置页: http://" + WiFi.localIP().toString());
            initWakeupBLE();
            // 巴法云参数齐全才连接 MQTT
            if (strlen(bemfa_uid) > 0 && strlen(bemfa_topic) > 0) {
                mqttConnect();
            } else {
                Serial.println("⚠️ 巴法云未配置，请访问配置页填写私钥和主题名");
            }
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
        if (bleInitialized) startBLEAdvertising();
    }
    if (pendingBLEStop) {
        pendingBLEStop = false;
        stopBLEAdvertising();
    }

    delay(10);
}
