/**
 * BLE Provisioning + 巴法云 + BLE唤醒 (ESP32-C3)
 * 版本: 3.0 - 用 BLE GATT 替代 WiFiManager 进行网络配置
 *
 * 配网流程：
 *   1. 首次启动（或出厂重置后），设备广播 "ESP32C3_Config"
 *   2. 手机用 nRF Connect 等 BLE 工具连接
 *   3. 依次写入各特征值（SSID/密码/巴法云UID等）
 *   4. 向 CMD 特征写入 "commit" 触发保存并重启
 *
 * 按键操作：
 *   短按 → 重新进入 BLE 配网模式
 *   长按(>3s) → 出厂重置，清除所有配置
 */

#include <WiFi.h>
#include <Preferences.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <esp_mac.h>

// ==================== 引脚定义 ====================
#define TRIGGER_PIN   9
#define LED_PIN       12
#define BAFA_LED_PIN  13

// ==================== 时间常量 ====================
#define WDT_TIMEOUT_SECONDS     180
#define BUTTON_DEBOUNCE_MS      50
#define LONG_PRESS_MS           3000
#define BLE_WAKE_DURATION_MS    1000    // 唤醒广播持续1秒
#define WIFI_CONNECT_TIMEOUT_MS 30000   // WiFi连接超时30秒

// ==================== 设备名称 ====================
#define DEVICE_NAME      "ESP32C3_BLE_Beacon"  // 唤醒模式BLE名
#define PROV_DEVICE_NAME "ESP32C3_Config"       // 配网模式BLE名

// ==================== BLE 配网 GATT UUIDs ====================
#define PROV_SVC_UUID   "12345678-1234-1234-1234-123456789abc"
#define CHR_SSID_UUID   "12345678-1234-1234-1234-123456789001"  // 写: WiFi SSID
#define CHR_PASS_UUID   "12345678-1234-1234-1234-123456789002"  // 写: WiFi 密码
#define CHR_UID_UUID    "12345678-1234-1234-1234-123456789003"  // 写: 巴法云 UID
#define CHR_TOPIC_UUID  "12345678-1234-1234-1234-123456789004"  // 写: 巴法云 主题
#define CHR_BMAC_UUID   "12345678-1234-1234-1234-123456789005"  // 写: BLE MAC
#define CHR_BDATA_UUID  "12345678-1234-1234-1234-123456789006"  // 写: BLE 广播数据(Hex)
#define CHR_CMD_UUID    "12345678-1234-1234-1234-123456789007"  // 写: "commit" 提交
#define CHR_STATUS_UUID "12345678-1234-1234-1234-123456789008"  // 读/通知: 状态

// ==================== 全局配置缓冲区 ====================
char wifi_ssid_buf[33]  = "";
char wifi_pass_buf[65]  = "";
char bafa_uid_buf[65]   = "";
char bafa_topic_buf[33] = "";
char ble_mac_buf[19]    = "";
char ble_data_buf[65]   = "";

// ==================== 默认值 ====================
const char* DEFAULT_BAFA_UID   = "your_bafa_uid_here";
const char* DEFAULT_BAFA_TOPIC = "your_bafa_topic_here";
const char* DEFAULT_BLE_MAC    = "78:81:8c:06:9a:c4";
const char* DEFAULT_BLE_DATA   = "0201061BFF53050100037E0566200001816D60168C81780F00000000000000";

// ==================== 唤醒BLE广播数据 ====================
uint8_t newMAC[6] = {0x78, 0x81, 0x8c, 0x06, 0x9a, 0xc4};

static uint8_t wake_adv_data[] = {
    0x02, 0x01, 0x06,
    0x1B, 0xFF,
    0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05, 0x66, 0x20, 0x00, 0x01, 0x81,
    0x6D, 0x60, 0x16, 0x8C, 0x81, 0x78,
    0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// ==================== 系统状态 ====================
enum SystemStatus {
    STATUS_BOOT,
    STATUS_PROV_MODE,   // BLE配网模式
    STATUS_CONNECTING,
    STATUS_CONNECTED,
    STATUS_ERROR
};
SystemStatus current_status = STATUS_BOOT;

enum BLEMode { BLE_NONE, BLE_PROVISIONING, BLE_WAKEUP };
BLEMode currentBLEMode = BLE_NONE;

unsigned long last_led_toggle = 0;
bool led_state = false;
bool ledState  = false;
unsigned long bleAdvertisingStart = 0;

// ==================== WiFi 重连状态 ====================
unsigned long lastReconnectAttempt = 0;
int           reconnectAttempts    = 0;
// 退避间隔：10s → 20s → 40s → 60s（上限）
#define RECONNECT_BASE_MS  10000UL
#define RECONNECT_MAX_MS   60000UL

// ==================== 配网状态 ====================
bool provClientConnected  = false;
volatile bool provCommitPending = false;
String prov_ssid, prov_pass, prov_uid, prov_topic, prov_mac, prov_data;

// ==================== 对象 ====================
Preferences prefs;
WiFiClient client;
BLECharacteristic* pStatusChar = nullptr;
BLEAdvertising*    pWakeAdv    = nullptr;

const char* host = "bemfa.com";
const int   port = 8344;

// ==================== 函数声明 ====================
void loadSavedParams();
void saveAllParams();
void checkButton();
void updateStatusLED();
void safeRestart(const char* reason);
void printSystemInfo();
bool initializePreferences();
bool connectWiFi();
void connect_server();
void send_heartbeat();
void startProvBLE();
bool handleProvCommit();
void initWakeupBLE();
void startBLEAdvertising();
void stopBLEAdvertising();
void handleBLEAdvertising();
void handleWiFiReconnect();
void ensureBafaConnected();
std::string hexToBytes(const String& hex);

// =====================================================
// BLE 配网回调
// =====================================================

class ProvServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        provClientConnected = true;
        Serial.println("✅ [BLE配网] 手机已连接");
        if (pStatusChar) pStatusChar->setValue("connected");
    }
    void onDisconnect(BLEServer*) override {
        provClientConnected = false;
        Serial.println("📱 [BLE配网] 手机已断开");
        // 未提交则重新广播，等待下次连接
        if (!provCommitPending && currentBLEMode == BLE_PROVISIONING) {
            BLEDevice::startAdvertising();
        }
    }
};

class ProvWriteCallback : public BLECharacteristicCallbacks {
    String fieldName;
public:
    ProvWriteCallback(const char* name) : fieldName(name) {}

    void onWrite(BLECharacteristic* c) override {
        String val = c->getValue().c_str();
        Serial.println("📝 [BLE配网] " + fieldName + " = " + val);

        if (fieldName == "ssid")  prov_ssid  = val;
        if (fieldName == "pass")  prov_pass  = val;
        if (fieldName == "uid")   prov_uid   = val;
        if (fieldName == "topic") prov_topic = val;
        if (fieldName == "mac")   prov_mac   = val;
        if (fieldName == "data")  prov_data  = val;
        if (fieldName == "cmd" && val == "commit") {
            provCommitPending = true;
            Serial.println("🚀 [BLE配网] 收到 commit 指令");
            if (pStatusChar) pStatusChar->setValue("provisioning...");
        }
    }
};

// =====================================================
// Preferences
// =====================================================

bool initializePreferences() {
    if (!prefs.begin("config", true)) return false;
    size_t fe = prefs.freeEntries();
    prefs.end();
    Serial.println("✅ Preferences 就绪 (空闲条目: " + String(fe) + ")");
    return true;
}

void loadSavedParams() {
    Serial.println("📖 加载保存的参数...");
    if (!prefs.begin("config", true)) {
        Serial.println("❌ 无法打开 Preferences，使用默认值");
        strcpy(bafa_uid_buf,   DEFAULT_BAFA_UID);
        strcpy(bafa_topic_buf, DEFAULT_BAFA_TOPIC);
        strcpy(ble_mac_buf,    DEFAULT_BLE_MAC);
        strcpy(ble_data_buf,   DEFAULT_BLE_DATA);
        return;
    }

    String ssid  = prefs.getString("wifi_ssid",  "");
    String pass  = prefs.getString("wifi_pass",  "");
    String uid   = prefs.getString("bafa_uid",   DEFAULT_BAFA_UID);
    String topic = prefs.getString("bafa_topic", DEFAULT_BAFA_TOPIC);
    String mac   = prefs.getString("ble_mac",    DEFAULT_BLE_MAC);
    String data  = prefs.getString("ble_data",   DEFAULT_BLE_DATA);
    prefs.end();

    strncpy(wifi_ssid_buf,  ssid.c_str(),   sizeof(wifi_ssid_buf) - 1);
    strncpy(wifi_pass_buf,  pass.c_str(),   sizeof(wifi_pass_buf) - 1);
    strncpy(bafa_uid_buf,   uid.c_str(),    sizeof(bafa_uid_buf) - 1);
    strncpy(bafa_topic_buf, topic.c_str(),  sizeof(bafa_topic_buf) - 1);
    strncpy(ble_mac_buf,    mac.c_str(),    sizeof(ble_mac_buf) - 1);
    strncpy(ble_data_buf,   data.c_str(),   sizeof(ble_data_buf) - 1);

    Serial.println("✅ 参数加载完成:");
    Serial.println("   WiFi SSID:  " + String(wifi_ssid_buf));
    Serial.println("   Bafa UID:   " + String(bafa_uid_buf));
    Serial.println("   Bafa Topic: " + String(bafa_topic_buf));
    Serial.println("   BLE MAC:    " + String(ble_mac_buf));
}

void saveAllParams() {
    if (!prefs.begin("config", false)) {
        Serial.println("❌ 无法打开 Preferences 写入");
        return;
    }
    prefs.putString("wifi_ssid",  wifi_ssid_buf);
    prefs.putString("wifi_pass",  wifi_pass_buf);
    prefs.putString("bafa_uid",   bafa_uid_buf);
    prefs.putString("bafa_topic", bafa_topic_buf);
    prefs.putString("ble_mac",    ble_mac_buf);
    prefs.putString("ble_data",   ble_data_buf);
    prefs.end();
    Serial.println("✅ 配置已保存到 Flash");
}

// =====================================================
// BLE 配网
// =====================================================

void startProvBLE() {
    if (currentBLEMode == BLE_PROVISIONING) return;

    // 唤醒BLE已初始化则先注销
    if (currentBLEMode == BLE_WAKEUP) {
        BLEDevice::deinit(true);
        delay(100);
    }

    Serial.println("📡 [BLE配网] 启动中...");
    current_status    = STATUS_PROV_MODE;
    currentBLEMode    = BLE_PROVISIONING;
    provCommitPending = false;
    prov_ssid = prov_pass = prov_uid = prov_topic = prov_mac = prov_data = "";

    BLEDevice::init(PROV_DEVICE_NAME);
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ProvServerCallbacks());

    BLEService* pSvc = pServer->createService(PROV_SVC_UUID);

    // 创建可写特征的辅助 lambda
    auto mkW = [&](const char* uuid, const char* field) {
        auto* c = pSvc->createCharacteristic(uuid,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        c->setCallbacks(new ProvWriteCallback(field));
        return c;
    };

    mkW(CHR_SSID_UUID,  "ssid");
    mkW(CHR_PASS_UUID,  "pass");
    mkW(CHR_UID_UUID,   "uid");
    mkW(CHR_TOPIC_UUID, "topic");
    mkW(CHR_BMAC_UUID,  "mac");
    mkW(CHR_BDATA_UUID, "data");
    mkW(CHR_CMD_UUID,   "cmd");

    pStatusChar = pSvc->createCharacteristic(CHR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pStatusChar->addDescriptor(new BLE2902());
    pStatusChar->setValue("waiting");

    pSvc->start();

    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(PROV_SVC_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("✅ [BLE配网] 已启动，设备名: " + String(PROV_DEVICE_NAME));
    Serial.println("   Service UUID: " + String(PROV_SVC_UUID));
    Serial.println("   请用手机 BLE 工具连接，依次写入各字段，最后 cmd='commit'");
}

bool handleProvCommit() {
    if (!provCommitPending) return false;
    provCommitPending = false;

    Serial.println("🔧 [BLE配网] 处理配置提交...");

    if (prov_ssid.length() == 0) {
        Serial.println("❌ WiFi SSID 不能为空");
        if (pStatusChar) pStatusChar->setValue("error:empty_ssid");
        return false;
    }

    // 可选字段使用默认值
    if (prov_uid.length()   == 0) prov_uid   = DEFAULT_BAFA_UID;
    if (prov_topic.length() == 0) prov_topic = DEFAULT_BAFA_TOPIC;
    if (prov_mac.length()   == 0) prov_mac   = DEFAULT_BLE_MAC;
    if (prov_data.length()  == 0) prov_data  = DEFAULT_BLE_DATA;

    // 写入缓冲区
    strncpy(wifi_ssid_buf,  prov_ssid.c_str(),  sizeof(wifi_ssid_buf) - 1);
    strncpy(wifi_pass_buf,  prov_pass.c_str(),  sizeof(wifi_pass_buf) - 1);
    strncpy(bafa_uid_buf,   prov_uid.c_str(),   sizeof(bafa_uid_buf) - 1);
    strncpy(bafa_topic_buf, prov_topic.c_str(), sizeof(bafa_topic_buf) - 1);
    strncpy(ble_mac_buf,    prov_mac.c_str(),   sizeof(ble_mac_buf) - 1);
    strncpy(ble_data_buf,   prov_data.c_str(),  sizeof(ble_data_buf) - 1);

    saveAllParams();

    if (pStatusChar) {
        pStatusChar->setValue("saved,restarting...");
        pStatusChar->notify();
    }
    delay(800); // 让手机端收到通知

    safeRestart("配网完成");
    return true; // 不会执行到这里
}

// =====================================================
// WiFi
// =====================================================

bool connectWiFi() {
    if (strlen(wifi_ssid_buf) == 0) {
        Serial.println("⚠️  无 WiFi SSID 配置");
        return false;
    }

    Serial.println("🔄 连接 WiFi: " + String(wifi_ssid_buf));
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid_buf, wifi_pass_buf);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("\n❌ WiFi 连接超时");
            return false;
        }
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset();
    }

    Serial.println("\n✅ WiFi 已连接!");
    Serial.println("   IP:   " + WiFi.localIP().toString());
    Serial.println("   RSSI: " + String(WiFi.RSSI()) + " dBm");
    return true;
}

// =====================================================
// 唤醒BLE
// =====================================================

void initWakeupBLE() {
    if (currentBLEMode == BLE_WAKEUP) return;

    if (currentBLEMode == BLE_PROVISIONING) {
        BLEDevice::deinit(true);
        delay(100);
    }

    Serial.println("🔵 初始化唤醒 BLE...");

    uint8_t customMAC[6];
    if (strlen(ble_mac_buf) > 0) {
        sscanf(ble_mac_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &customMAC[0], &customMAC[1], &customMAC[2],
               &customMAC[3], &customMAC[4], &customMAC[5]);
        customMAC[5] -= 2;
        esp_base_mac_addr_set(customMAC);
    } else {
        newMAC[5] -= 2;
        esp_base_mac_addr_set(newMAC);
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    Serial.printf("   BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    BLEDevice::init(DEVICE_NAME);
    pWakeAdv = BLEDevice::getAdvertising();
    pWakeAdv->setMinInterval(0x0020);
    pWakeAdv->setMaxInterval(0x0040);
    pWakeAdv->setAdvertisementType(ADV_TYPE_NONCONN_IND);

    currentBLEMode = BLE_WAKEUP;
    Serial.println("✅ 唤醒 BLE 初始化完成");
}

void startBLEAdvertising() {
    if (currentBLEMode != BLE_WAKEUP) initWakeupBLE();

    Serial.println("📡 开始 BLE 唤醒广播 (1秒)...");
    pWakeAdv->stop();

    BLEAdvertisementData advData;
    std::string raw(reinterpret_cast<char*>(wake_adv_data), sizeof(wake_adv_data));
    advData.addData(raw);
    pWakeAdv->setAdvertisementData(advData);
    pWakeAdv->start();

    bleAdvertisingStart = millis();
}

void stopBLEAdvertising() {
    if (currentBLEMode == BLE_WAKEUP && pWakeAdv) {
        pWakeAdv->stop();
        bleAdvertisingStart = 0;
        Serial.println("🔇 BLE 唤醒广播已停止");
    }
}

void handleBLEAdvertising() {
    if (bleAdvertisingStart > 0 && millis() - bleAdvertisingStart >= BLE_WAKE_DURATION_MS) {
        stopBLEAdvertising();
    }
}

// =====================================================
// 巴法云
// =====================================================

void connect_server() {
    Serial.print("🔗 连接巴法云...");
    if (!client.connect(host, port)) {
        Serial.println(" 失败!");
        return;
    }
    Serial.println(" 成功!");
    String cmd = "cmd=1&uid=" + String(bafa_uid_buf) + "&topic=" + String(bafa_topic_buf) + "\r\n";
    client.print(cmd);
    Serial.println("✅ 已订阅主题: " + String(bafa_topic_buf));
}

void send_heartbeat() {
    client.print("cmd=0&msg=ping\r\n");
    Serial.println("💓 心跳已发送");
}

// =====================================================
// WiFi 自动重连（非阻塞，指数退避）
// =====================================================

void handleWiFiReconnect() {
    if (WiFi.status() == WL_CONNECTED) {
        if (current_status == STATUS_CONNECTING) {
            // 刚重连成功
            current_status     = STATUS_CONNECTED;
            reconnectAttempts  = 0;
            lastReconnectAttempt = 0;
            Serial.println("✅ WiFi 重连成功");
            Serial.println("   IP:   " + WiFi.localIP().toString());
            Serial.println("   RSSI: " + String(WiFi.RSSI()) + " dBm");
        }
        return;
    }

    // 刚断线：记录状态，断开巴法云
    if (current_status == STATUS_CONNECTED) {
        current_status       = STATUS_CONNECTING;
        reconnectAttempts    = 0;
        lastReconnectAttempt = 0;
        client.stop();
        Serial.println("⚠️  WiFi 断线，启动自动重连...");
    }

    // 计算退避间隔：10s * 2^attempt，上限60s
    unsigned long backoff = RECONNECT_BASE_MS * (1UL << min(reconnectAttempts, 3));
    if (backoff > RECONNECT_MAX_MS) backoff = RECONNECT_MAX_MS;

    if (millis() - lastReconnectAttempt < backoff) return;

    reconnectAttempts++;
    lastReconnectAttempt = millis();
    Serial.printf("🔄 WiFi 重连尝试 #%d (下次退避 %lus)...\n",
                  reconnectAttempts,
                  min(RECONNECT_BASE_MS * (1UL << min(reconnectAttempts, 3)),
                      RECONNECT_MAX_MS) / 1000);

    WiFi.disconnect(false);
    WiFi.begin(wifi_ssid_buf, wifi_pass_buf);
}

// =====================================================
// 确保巴法云 TCP 连接正常（WiFi 已连接时调用）
// =====================================================

void ensureBafaConnected() {
    if (!client.connected()) {
        Serial.println("🔗 巴法云连接断开，重新连接...");
        client.stop();
        connect_server();
    }
}

// =====================================================
// 工具函数
// =====================================================

void safeRestart(const char* reason) {
    Serial.println("🔄 重启: " + String(reason));
    prefs.end();
    WiFi.disconnect(true);
    delay(1000);
    ESP.restart();
}

void printSystemInfo() {
    Serial.println("📋 系统信息:");
    Serial.println("   芯片型号: " + String(ESP.getChipModel()));
    Serial.println("   Flash: "    + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB");
    Serial.println("   可用堆: "   + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("   SDK: "      + String(ESP.getSdkVersion()));
}

void updateStatusLED() {
    unsigned long interval;
    switch (current_status) {
        case STATUS_PROV_MODE:  interval = 150;  break; // 超快闪 - 配网
        case STATUS_CONNECTING: interval = 500;  break; // 中速闪 - 连接中
        case STATUS_CONNECTED:  interval = 2000; break; // 慢闪   - 已连接
        case STATUS_ERROR:
            digitalWrite(LED_PIN, HIGH);
            return;
        default: interval = 1000; break;
    }

    if (millis() - last_led_toggle >= interval) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
        last_led_toggle = millis();
    }
}

void checkButton() {
    static unsigned long last_press   = 0;
    static bool          button_pressed = false;

    bool pressed = (digitalRead(TRIGGER_PIN) == LOW);

    if (pressed && !button_pressed) {
        if (millis() - last_press > BUTTON_DEBOUNCE_MS) {
            button_pressed = true;
            last_press     = millis();
            Serial.println("🔘 按键按下");
        }
    } else if (!pressed && button_pressed) {
        unsigned long duration = millis() - last_press;
        button_pressed = false;

        if (duration > LONG_PRESS_MS) {
            // 长按：出厂重置
            Serial.println("🔄 长按(>3s): 出厂重置...");
            if (prefs.begin("config", false)) {
                prefs.clear();
                prefs.end();
                Serial.println("   ✅ Preferences 已清除");
            }
            WiFi.disconnect(true);
            safeRestart("出厂重置完成");
        } else {
            // 短按：进入BLE配网
            Serial.println("⚙️  短按: 启动 BLE 配网模式");
            stopBLEAdvertising();
            startProvBLE();
        }
    }
}

std::string hexToBytes(const String& hex) {
    std::string result;
    for (unsigned int i = 0; i < hex.length(); i += 2) {
        std::string byte = hex.substring(i, i + 2).c_str();
        result.push_back((char)strtol(byte.c_str(), nullptr, 16));
    }
    return result;
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
    esp_base_mac_addr_set(newMAC);
    WiFi.mode(WIFI_STA);
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("ESP32C3  BLE配网 + 巴法云 + BLE唤醒");
    Serial.println("版本: 3.0 - BLE Provisioning");
    Serial.println("========================================");

    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    pinMode(LED_PIN,      OUTPUT);
    pinMode(BAFA_LED_PIN, OUTPUT);
    digitalWrite(LED_PIN,      LOW);
    digitalWrite(BAFA_LED_PIN, LOW);

    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
    Serial.println("✅ 看门狗已初始化");

    printSystemInfo();

    if (!initializePreferences()) {
        Serial.println("❌ Preferences 初始化失败，使用默认值");
    }

    loadSavedParams();

    current_status = STATUS_CONNECTING;

    // 尝试连接 WiFi；若失败则进入 BLE 配网
    if (!connectWiFi()) {
        Serial.println("📡 WiFi 连接失败，启动 BLE 配网模式");
        startProvBLE();
        return;
    }

    current_status = STATUS_CONNECTED;

    // 初始化唤醒BLE
    initWakeupBLE();

    // 连接巴法云
    connect_server();

    Serial.println("🚀 启动完成，进入主循环");
}

void loop() {
    esp_task_wdt_reset();

    // ---- 配网模式 ----
    if (current_status == STATUS_PROV_MODE) {
        handleProvCommit(); // commit 后会重启，不会返回
        checkButton();
        updateStatusLED();
        delay(50);
        return;
    }

    // ---- 正常运行模式 ----
    checkButton();
    updateStatusLED();

    // WiFi 自动重连（非阻塞）
    handleWiFiReconnect();

    // WiFi 未连接时跳过后续网络操作
    if (current_status != STATUS_CONNECTED) {
        delay(100);
        return;
    }

    // 确保巴法云 TCP 连接正常
    ensureBafaConnected();

    // 处理巴法云消息
    if (client.available()) {
        String message = client.readStringUntil('\n');
        Serial.println("📨 收到: " + message);

        if (message.indexOf("on") != -1) {
            digitalWrite(BAFA_LED_PIN, HIGH);
            ledState = true;
            Serial.println("💡 LED 开");
            startBLEAdvertising();
        } else if (message.indexOf("off") != -1) {
            digitalWrite(BAFA_LED_PIN, LOW);
            ledState = false;
            Serial.println("💡 LED 关");
            stopBLEAdvertising();
        }
    }

    handleBLEAdvertising();

    // 每50秒发送心跳
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 50000) {
        send_heartbeat();
        lastHeartbeat = millis();
    }

    delay(100);
}
