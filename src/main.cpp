/**
 * Matter + BLE 唤醒小爱音箱 (ESP32-C3)
 * 版本: 4.1 - 接入米家 Matter 协议
 *
 * 配网流程（首次使用）：
 *   1. 上电后 LED 快闪，设备自动开启配对引导 WiFi
 *   2. 手机连接 WiFi "ESP32_Matter_Setup"（密码见 SETUP_AP_PASSWORD）
 *   3. 浏览器访问 192.168.4.1，查看 Matter 配对码 / 二维码链接
 *   4. 断开引导 WiFi，打开米家 App → 添加设备 → 扫码 / 手动输入配对码
 *   5. Matter 自动完成 WiFi 配置，配对成功后引导 WiFi 自动关闭
 *   6. 配对成功后 LED 慢闪，设备出现在米家 App
 *
 * 日常使用：
 *   - 米家 App / 小爱音箱发出"开"指令 → 发送 BLE 广播唤醒目标设备
 *   - 米家 App 发出"关"指令 → 停止 BLE 广播
 *
 * 按键操作：
 *   短按 → 未配对：重启引导 WiFi；已配对：手动触发 BLE 广播测试
 *   长按(>3s) → 出厂重置（Matter 解配对 + 清除所有配置）
 */

#include <Matter.h>
#include <MatterEndpoints/MatterOnOffLight.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
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

// ==================== 配对引导 WiFi AP ====================
#define SETUP_AP_SSID     "ESP32_Matter_Setup"
#define SETUP_AP_PASSWORD "12345678"

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

// ==================== Matter 配对参数（固定值 → 可预先生成 QR 码贴到设备上）====================
// passcode: 8位数字，不能是 00000000/11111111 等连号；discriminator: 0-4095
// 修改这两个值后，用 Matter.getOnboardingQRCodeUrl() 重新生成 QR 码
#define MATTER_PASSCODE     20202021
#define MATTER_DISCRIMINATOR 3840

// ==================== 对象 ====================
Preferences      prefs;
MatterOnOffLight MatterLight;
WebServer        setupServer(80);
bool             apActive = false;

// ==================== 函数声明 ====================
void loadConfig();
void saveConfig(const String& mac, const String& data);
void startSetupAP();
void stopSetupAP();
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
// Web 配置页（AP 模式 + 配对后 LAN 模式复用同一套路由）
// =====================================================

// 返回完整 HTML 页面（commissioned 决定是否显示 Matter 配对区）
String buildConfigPage(bool commissioned) {
    String code  = commissioned ? "" : Matter.getManualPairingCode();
    String qrUrl = commissioned ? "" : Matter.getOnboardingQRCodeUrl();
    String ip    = commissioned ? WiFi.localIP().toString() : "192.168.4.1";

    String html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;padding:20px;max-width:460px;margin:auto;color:#333}"
        "h2{margin-bottom:2px} .sub{color:#888;font-size:.9em;margin:0 0 16px}"
        ".code{font-size:1.8em;font-weight:bold;color:#e74c3c;letter-spacing:3px;"
        "background:#fff5f5;padding:10px;border-radius:8px;text-align:center;margin:12px 0}"
        ".btn{display:block;background:#07c160;color:#fff;padding:12px;text-align:center;"
        "text-decoration:none;border-radius:8px;font-size:1em;margin:10px 0}"
        "label{display:block;margin-top:14px;font-weight:bold;font-size:.9em}"
        "input[type=text]{width:100%;box-sizing:border-box;padding:9px;border:1px solid #ddd;"
        "border-radius:6px;font-size:.95em;margin-top:4px}"
        "input[type=submit]{width:100%;background:#1989fa;color:#fff;border:none;padding:12px;"
        "border-radius:8px;font-size:1em;margin-top:16px;cursor:pointer}"
        "hr{border:none;border-top:1px solid #eee;margin:20px 0}"
        ".ok{color:#07c160;font-weight:bold}"
        "</style></head><body>";

    // ---- Matter 配对区（仅未配对时显示）----
    if (!commissioned) {
        html += "<h2>Matter 配对</h2>"
                "<p class='sub'>先填写 BLE 配置，再去米家扫码</p>"
                "<p style='margin-bottom:4px'>手动配对码：</p>"
                "<div class='code'>" + code + "</div>"
                "<a href='" + qrUrl + "' class='btn'>点击生成二维码 →</a>"
                "<hr>";
    } else {
        html += "<h2>BLE 唤醒配置</h2>"
                "<p class='sub'>设备已接入米家，可随时修改唤醒目标</p>";
    }

    // ---- BLE 配置表单 ----
    html += "<form method='POST' action='/save'>"
            "<label>目标设备 BLE MAC</label>"
            "<input type='text' name='mac' value='" + String(ble_mac_buf) + "' "
            "placeholder='78:81:8c:06:9a:c4' pattern='^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$' required>"
            "<label>BLE 广播数据（HEX，不含空格）</label>"
            "<input type='text' name='data' value='" + String(ble_data_buf) + "' "
            "placeholder='0201061BFF...' pattern='^[0-9a-fA-F]+$' required>"
            "<input type='submit' value='保存并重启'>"
            "</form>";

    if (!commissioned) {
        html += "<hr><p style='font-size:.85em;color:#aaa'>"
                "配对成功后可通过 <b>http://" + ip + "</b> 或 "
                "<b>http://esp32c3-wake.local</b> 继续访问此页面</p>";
    }

    html += "</body></html>";
    return html;
}

void registerWebRoutes() {
    setupServer.on("/", HTTP_GET, []() {
        setupServer.send(200, "text/html; charset=utf-8",
                         buildConfigPage(Matter.isDeviceCommissioned()));
    });

    setupServer.on("/save", HTTP_POST, []() {
        String mac  = setupServer.arg("mac");
        String data = setupServer.arg("data");

        if (mac.length() > 0 && data.length() > 0) {
            if (prefs.begin("config", false)) {
                prefs.putString("ble_mac",  mac);
                prefs.putString("ble_data", data);
                prefs.end();
            }
            setupServer.send(200, "text/html; charset=utf-8",
                "<meta charset='UTF-8'>"
                "<p style='font-family:sans-serif;padding:20px'>"
                "✅ 已保存，设备重启中...</p>");
            delay(800);
            ESP.restart();
        } else {
            setupServer.send(400, "text/plain", "参数缺失");
        }
    });
}

void startSetupAP() {
    if (apActive) return;
    WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD);
    registerWebRoutes();
    setupServer.begin();
    apActive = true;
    Serial.println("📶 引导 WiFi: " + String(SETUP_AP_SSID)
                   + "  密码: " + String(SETUP_AP_PASSWORD));
    Serial.println("   浏览器访问: http://192.168.4.1");
}

void stopSetupAP() {
    if (!apActive) return;
    WiFi.softAPdisconnect(true);
    apActive = false;
    Serial.println("📴 引导 WiFi 已关闭");
}

void startLANServer() {
    // Matter 配对后在局域网继续提供配置页
    if (MDNS.begin("esp32c3-wake")) {
        Serial.println("🌐 mDNS: http://esp32c3-wake.local");
    }
    // 路由已在 startSetupAP 注册过，直接 begin 即可；
    // 若出厂重置后直接进入已配对状态，需重新注册
    if (!apActive) {
        registerWebRoutes();
        setupServer.begin();
    }
    Serial.println("🌐 配置页: http://" + WiFi.localIP().toString());
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
            // 短按：重启引导 WiFi 或 手动测试 BLE
            if (!Matter.isDeviceCommissioned()) {
                Serial.println("🔘 短按: 重启引导 WiFi");
                stopSetupAP();
                startSetupAP();
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
    // 固定 passcode/discriminator，使 QR 码每次相同
    Matter.setPasscode(MATTER_PASSCODE);
    Matter.setDiscriminator(MATTER_DISCRIMINATOR);
    Matter.begin();
    MatterLight.begin();
    MatterLight.onChangeOnOff(onLightChange);

    if (!Matter.isDeviceCommissioned()) {
        current_status = STATUS_COMMISSIONING;
        startSetupAP();
        Serial.println("  LED 快闪 = 等待配对");
        Serial.println("  短按按钮 = 重启引导 WiFi\n");
    } else {
        current_status = STATUS_CONNECTED;
        Serial.println("✅ Matter 已配对，设备正常运行");
        startLANServer();
        initWakeupBLE();
    }

    Serial.println("🚀 启动完成\n");
}

void loop() {
    esp_task_wdt_reset();
    setupServer.handleClient();
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
        stopSetupAP();
        delay(1000); // 等待 Matter BLE 栈完全释放
        startLANServer();
        initWakeupBLE();
    }
    prevCommissioned = nowCommissioned;

    delay(50);
}
