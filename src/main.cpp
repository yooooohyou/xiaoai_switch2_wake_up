/**
 * Matter + BLE 唤醒小爱音箱 (ESP32-C3)
 * 版本: 4.0 - 接入米家 Matter 协议
 *
 * 配网流程（首次使用）：
 *   1. 上电后 LED 快闪，串口打印配对码
 *   2. 打开米家 App → 添加设备 → 扫码 / 手动输入配对码
 *   3. Matter 自动完成 WiFi 配置，无需手动操作
 *   4. 配对成功后 LED 慢闪，设备出现在米家 App
 *
 * 日常使用：
 *   - 米家 App / 小爱音箱发出"开"指令 → 发送 BLE 广播唤醒目标设备
 *   - 米家 App 发出"关"指令 → 停止 BLE 广播
 *
 * 按键操作：
 *   短按 → 未配对：重新打印配对码；已配对：手动触发 BLE 广播测试
 *   长按(>3s) → 出厂重置（Matter 解配对 + 清除所有配置）
 */

#include <Matter.h>
#include <MatterOnOffLight.h>
#include <Preferences.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <esp_mac.h>

// ==================== 引脚定义 ====================
#define TRIGGER_PIN    9
#define LED_PIN        12
#define STATUS_LED_PIN 13

// ==================== 时间常量 ====================
#define WDT_TIMEOUT_SECONDS  180
#define BUTTON_DEBOUNCE_MS   50
#define LONG_PRESS_MS        3000
#define BLE_WAKE_DURATION_MS 1000

// ==================== BLE 设备名（唤醒模式）====================
#define BLE_DEVICE_NAME "ESP32C3_BLE_Beacon"

// ==================== 配置默认值 ====================
const char* DEFAULT_BLE_MAC  = "78:81:8c:06:9a:c4";
const char* DEFAULT_BLE_DATA = "0201061BFF53050100037E0566200001816D60168C81780F00000000000000";

// ==================== 配置缓冲区（仅 BLE 唤醒参数）====================
// WiFi 配置由 Matter 自动管理，无需手动保存
char ble_mac_buf[19]  = "";
char ble_data_buf[65] = "";

// ==================== BLE 唤醒广播数据 ====================
// 设备基础 MAC，用于 Matter 配对时的设备识别
uint8_t baseMAC[6] = {0x78, 0x81, 0x8c, 0x06, 0x9a, 0xc4};

static uint8_t wake_adv_data[] = {
    0x02, 0x01, 0x06,
    0x1B, 0xFF,
    0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05, 0x66, 0x20, 0x00, 0x01, 0x81,
    0x6D, 0x60, 0x16, 0x8C, 0x81, 0x78,
    0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// ==================== 系统状态 ====================
enum SystemStatus { STATUS_BOOT, STATUS_COMMISSIONING, STATUS_CONNECTED };
SystemStatus current_status = STATUS_BOOT;

unsigned long last_led_toggle = 0;
bool          led_state       = false;

// ==================== BLE 唤醒状态 ====================
bool            bleInitialized  = false;
unsigned long   bleAdvStart     = 0;
BLEAdvertising* pWakeAdv        = nullptr;

// ==================== Matter 回调 → loop 通信标志 ====================
// Matter 回调运行在独立任务中，用 volatile flag 安全传递到 loop
volatile bool pendingBLEStart = false;
volatile bool pendingBLEStop  = false;

// ==================== 对象 ====================
Preferences      prefs;
MatterOnOffLight MatterLight;

// ==================== 函数声明 ====================
void loadConfig();
void saveConfig(const String& mac, const String& data);
void initWakeupBLE();
void startBLEAdvertising();
void stopBLEAdvertising();
void handleBLEAdvertising();
void checkButton();
void updateStatusLED();
void safeRestart(const char* reason);

// =====================================================
// Matter 开关回调（运行在 Matter 任务）
// =====================================================

bool onLightChange(bool state) {
    // 只设置 flag，不直接操作 BLE（避免跨任务竞争）
    if (state) {
        pendingBLEStart = true;
    } else {
        pendingBLEStop = true;
    }
    digitalWrite(STATUS_LED_PIN, state ? HIGH : LOW);
    Serial.println(String("📨 Matter 指令: ") + (state ? "ON → 触发唤醒广播" : "OFF → 停止广播"));
    return true;
}

// =====================================================
// 配置加载/保存（仅 BLE 唤醒参数）
// =====================================================

void loadConfig() {
    if (!prefs.begin("config", true)) {
        strcpy(ble_mac_buf,  DEFAULT_BLE_MAC);
        strcpy(ble_data_buf, DEFAULT_BLE_DATA);
        prefs.end();
        return;
    }
    String mac  = prefs.getString("ble_mac",  DEFAULT_BLE_MAC);
    String data = prefs.getString("ble_data", DEFAULT_BLE_DATA);
    prefs.end();

    strncpy(ble_mac_buf,  mac.c_str(),  sizeof(ble_mac_buf) - 1);
    strncpy(ble_data_buf, data.c_str(), sizeof(ble_data_buf) - 1);
    Serial.println("✅ 配置已加载 - BLE MAC: " + mac);
}

// =====================================================
// 唤醒 BLE 初始化（Matter 配对完成后调用）
// =====================================================

void initWakeupBLE() {
    if (bleInitialized) return;

    // 根据配置设置 BLE MAC（BLE MAC = 基础MAC - 2）
    uint8_t mac[6];
    if (strlen(ble_mac_buf) > 0) {
        sscanf(ble_mac_buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
        mac[5] -= 2;
    } else {
        memcpy(mac, baseMAC, 6);
        mac[5] -= 2;
    }
    esp_base_mac_addr_set(mac);

    uint8_t readback[6];
    esp_read_mac(readback, ESP_MAC_BT);
    Serial.printf("🔵 唤醒 BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  readback[0], readback[1], readback[2],
                  readback[3], readback[4], readback[5]);

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
    std::string raw(reinterpret_cast<char*>(wake_adv_data), sizeof(wake_adv_data));
    advData.addData(raw);
    pWakeAdv->setAdvertisementData(advData);
    pWakeAdv->start();
    bleAdvStart = millis();
    Serial.println("📡 BLE 唤醒广播已开始 (持续1秒)...");
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
        case STATUS_COMMISSIONING: interval = 300;  break; // 快闪 - 等待米家配对
        case STATUS_CONNECTED:     interval = 2000; break; // 慢闪 - 正常运行
        default:                   interval = 1000; break;
    }
    if (millis() - last_led_toggle >= interval) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
        last_led_toggle = millis();
    }
}

void checkButton() {
    static unsigned long last_press  = 0;
    static bool          btn_pressed = false;

    bool pressed = (digitalRead(TRIGGER_PIN) == LOW);

    if (pressed && !btn_pressed) {
        if (millis() - last_press > BUTTON_DEBOUNCE_MS) {
            btn_pressed = true;
            last_press  = millis();
        }
    } else if (!pressed && btn_pressed) {
        unsigned long duration = millis() - last_press;
        btn_pressed = false;

        if (duration > LONG_PRESS_MS) {
            // 长按：Matter 出厂重置
            Serial.println("🔄 长按(>3s): Matter 出厂重置...");
            Matter.decommission();
            if (prefs.begin("config", false)) {
                prefs.clear();
                prefs.end();
            }
            safeRestart("出厂重置完成");
        } else {
            // 短按：显示配对信息 或 手动测试
            if (!Matter.isDeviceCommissioned()) {
                Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                Serial.println("  手动配对码: " + Matter.getManualPairingCode());
                Serial.println("  二维码链接: " + Matter.getOnboardingQRCodeUrl());
                Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
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
    // 在 BT/WiFi 初始化前设置基础 MAC
    esp_base_mac_addr_set(baseMAC);

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  ESP32C3  Matter + BLE 唤醒小爱音箱");
    Serial.println("  版本: 4.0 - 米家 Matter 协议");
    Serial.println("========================================");

    pinMode(TRIGGER_PIN,    INPUT_PULLUP);
    pinMode(LED_PIN,        OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(LED_PIN,        LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);

    loadConfig();

    // 初始化 Matter 和 On/Off Light 端点
    Matter.begin();
    MatterLight.begin();
    MatterLight.onChangeOnOff(onLightChange);

    if (!Matter.isDeviceCommissioned()) {
        current_status = STATUS_COMMISSIONING;
        Serial.println("\n⚠️  Matter 尚未与米家配对！");
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.println("  1. 打开米家 App → 添加设备");
        Serial.println("  2. 扫描二维码 或 手动输入配对码");
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.println("  手动配对码: " + Matter.getManualPairingCode());
        Serial.println("  二维码链接: " + Matter.getOnboardingQRCodeUrl());
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.println("  LED 快闪 = 等待配对");
        Serial.println("  短按按钮 = 重新打印配对信息\n");
    } else {
        current_status = STATUS_CONNECTED;
        Serial.println("✅ Matter 已配对，设备正常运行");
        Serial.println("   在米家 App 中即可控制\n");
        // Matter 配对后 BLE 已由 Matter 释放，可安全初始化唤醒 BLE
        initWakeupBLE();
    }

    Serial.println("🚀 启动完成\n");
}

void loop() {
    esp_task_wdt_reset();
    checkButton();
    updateStatusLED();
    handleBLEAdvertising();

    // 处理 Matter 回调传来的 BLE 请求（保证在 loop 任务执行，避免跨任务竞争）
    if (pendingBLEStart) {
        pendingBLEStart = false;
        if (bleInitialized) startBLEAdvertising();
    }
    if (pendingBLEStop) {
        pendingBLEStop = false;
        stopBLEAdvertising();
    }

    // 检测 Matter 是否刚完成配对（首次配对后初始化唤醒 BLE）
    static bool prevCommissioned = false;
    bool nowCommissioned = Matter.isDeviceCommissioned();
    if (!prevCommissioned && nowCommissioned) {
        prevCommissioned = true;
        current_status   = STATUS_CONNECTED;
        Serial.println("🎉 Matter 配对成功！设备已加入米家");
        delay(1000); // 等待 Matter BLE 栈完全释放
        initWakeupBLE();
    }
    prevCommissioned = nowCommissioned;

    delay(50);
}
