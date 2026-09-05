/*
 * LoRa Device - Zephyr RTOS Version
 * Heltec WiFi LoRa 32 V3
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/arch/cpu.h>
#include <soc.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// LOG_MODULE_REGISTER(lora_device, LOG_LEVEL_INF);
LOG_MODULE_REGISTER(LOG_LEVEL_INF);

/* ==================== DEFINES ==================== */
#define WDT_TIMEOUT_SEC 15
#define LORA_FREQUENCY 433000000

#define MAX_PRIORITY_STORE 100
#define MAX_NORMAL_STORE 5
#define MAX_Retry 10
#define DEBOUNCE_TIME 10000    // 10 seconds
#define NO_SIGNAL_TIMEOUT 300000 // 5 minutes
#define HEART_INTERVAL 120000   // 2 minutes

/* ==================== GPIO PINS ==================== */
#define S1_PIN 32
#define S2_PIN 33
#define S3_PIN 25
#define S4_PIN 27
#define S5_PIN 14
#define S6_PIN 13

/* ==================== NVS STORAGE ==================== */
#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET PARTITION_OFFSET(NVS_PARTITION)

/* ==================== GLOBAL VARIABLES ==================== */
static const struct device *gpio_dev;
static const struct device *lora_dev;
static struct nvs_fs fs;

static const char *serialNo = "20260002";
static const char *deviceID = "C002";
static const char *prefix = "";
static const char *prefix_ack = "ACK";

static uint32_t global_uid = 5000;
static uint32_t msg_id_S1 = 5000;
static uint32_t msg_id_S2 = 5000;
static uint32_t msg_id_S3 = 5000;
static uint32_t msg_id_S4 = 5000;
static uint32_t msg_id_S5 = 5000;
static uint32_t msg_id_S6 = 5000;

static bool S1_State = false, S2_State = false, S3_State = false;
static bool S4_State = false, S5_State = false, S6_State = false;

static uint64_t startS1 = 0, startS2 = 0, startS3 = 0;
static uint64_t startS4 = 0, startS5 = 0, startS6 = 0;

static uint64_t noSignalStartTime = 0;
static uint64_t lastHeartbeat = 0;
static uint64_t nextRetryTime = 0;

/* ==================== QUEUE STRUCTURES ==================== */
struct StoredMsg {
    uint32_t uid;
    int retryCount;
    int maxRetries;
    char msg[128];
};

static struct StoredMsg priorityQueue[MAX_PRIORITY_STORE];
static int priorityCount = 0;
static struct StoredMsg normalQueue[MAX_NORMAL_STORE];
static int normalCount = 0;

/* ==================== NVS FUNCTIONS ==================== */
static int nvs_init(void)
{
    int rc;

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }

    fs.offset = NVS_PARTITION_OFFSET;
    fs.sector_size = 4096;
    fs.sector_count = 4;

    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("NVS mount failed: %d", rc);
        return rc;
    }

    return 0;
}

static void save_uid(void)
{
    nvs_write(&fs, 1, &global_uid, sizeof(global_uid));
    nvs_write(&fs, 2, &msg_id_S1, sizeof(msg_id_S1));
    nvs_write(&fs, 3, &msg_id_S2, sizeof(msg_id_S2));
    nvs_write(&fs, 4, &msg_id_S3, sizeof(msg_id_S3));
    nvs_write(&fs, 5, &msg_id_S4, sizeof(msg_id_S4));
    nvs_write(&fs, 6, &msg_id_S5, sizeof(msg_id_S5));
    nvs_write(&fs, 7, &msg_id_S6, sizeof(msg_id_S6));
}

static void load_uid(void)
{
    nvs_read(&fs, 1, &global_uid, sizeof(global_uid));
    nvs_read(&fs, 2, &msg_id_S1, sizeof(msg_id_S1));
    nvs_read(&fs, 3, &msg_id_S2, sizeof(msg_id_S2));
    nvs_read(&fs, 4, &msg_id_S3, sizeof(msg_id_S3));
    nvs_read(&fs, 5, &msg_id_S4, sizeof(msg_id_S4));
    nvs_read(&fs, 6, &msg_id_S5, sizeof(msg_id_S5));
    nvs_read(&fs, 7, &msg_id_S6, sizeof(msg_id_S6));
}

/* ==================== RESET REASON ==================== */
static const char *get_reset_reason(void)
{
    return "Power On";
}

/* ==================== QUEUE FUNCTIONS ==================== */
static void store_message(uint32_t uid, const char *msg, int maxRetries)
{
    if (maxRetries == -1) {
        for (int i = 0; i < priorityCount; i++) {
            if (priorityQueue[i].uid == uid) return;
        }
        if (priorityCount >= MAX_PRIORITY_STORE) {
            LOG_WRN("Priority queue full, removing oldest");
            for (int i = 1; i < MAX_PRIORITY_STORE; i++) {
                priorityQueue[i - 1] = priorityQueue[i];
            }
            priorityCount--;
        }
        priorityQueue[priorityCount].uid = uid;
        strcpy(priorityQueue[priorityCount].msg, msg);
        priorityQueue[priorityCount].retryCount = 0;
        priorityQueue[priorityCount].maxRetries = -1;
        priorityCount++;
        //LOG_INF("Stored [Priority]: %s", msg);
        printk("Stored [Priority]: %s\n", msg);
    } else {
        for (int i = 0; i < normalCount; i++) {
            if (normalQueue[i].uid == uid) return;
        }
        if (normalCount >= MAX_NORMAL_STORE) {
            LOG_WRN("Normal queue full, removing oldest");
            for (int i = 1; i < MAX_NORMAL_STORE; i++) {
                normalQueue[i - 1] = normalQueue[i];
            }
            normalCount--;
        }
        normalQueue[normalCount].uid = uid;
        strcpy(normalQueue[normalCount].msg, msg);
        normalQueue[normalCount].retryCount = 0;
        normalQueue[normalCount].maxRetries = maxRetries;
        normalCount++;
        //LOG_INF("Stored [Normal]: %s", msg);
        printk("Stored [Priority]: %s\n", msg);
    }
}

static void delete_message(uint32_t uid)
{
    for (int i = 0; i < priorityCount; i++) {
        if (priorityQueue[i].uid == uid) {
            LOG_INF("Deleting priority UID: %d", uid);
            for (int j = i + 1; j < priorityCount; j++) {
                priorityQueue[j - 1] = priorityQueue[j];
            }
            priorityCount--;
            return;
        }
    }

    for (int i = 0; i < normalCount; i++) {
        if (normalQueue[i].uid == uid) {
            LOG_INF("Deleting normal UID: %d", uid);
            for (int j = i + 1; j < normalCount; j++) {
                normalQueue[j - 1] = normalQueue[j];
            }
            normalCount--;
            return;
        }
    }
}

/* ==================== SEND LORA EVENT ==================== */
static void send_lora_event(const char *dev, const char *ser, 
                           const char *eventType, const char *color,
                           const char *duration, const char *reason,
                           const char *pallet, int maxRetries)
{
    char message[128];
    char msg_id[16];
    
    if (strcmp(color, "S1") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S1);
    else if (strcmp(color, "S2") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S2);
    else if (strcmp(color, "S3") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S3);
    else if (strcmp(color, "S4") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S4);
    else if (strcmp(color, "S5") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S5);
    else if (strcmp(color, "S6") == 0) snprintf(msg_id, sizeof(msg_id), "%d", msg_id_S6);
    else if (strcmp(color, "DWN") == 0) strcpy(msg_id, "1");
    else strcpy(msg_id, "0");
    
    uint32_t uid = global_uid++;
    save_uid();
    
    snprintf(message, sizeof(message), "%d,%s,%s,%s,%s,%s,%s,%s,%s",
             uid, dev, msg_id, eventType, ser, color, duration, reason, pallet);
    
    store_message(uid, message, maxRetries);
}

static void send_heartbeat(void)
{
    send_lora_event(deviceID, serialNo, "STA", "DWN", "no_duration", "Power Down", "no_pallet", MAX_Retry);
}

static void send_power_on_sol(void)
{
    const char *reason = get_reset_reason();
    send_lora_event(deviceID, serialNo, "SOL", "DWN", "no_duration", reason, "no_pallet", -1);
}

/* ==================== PROCESS STORED MESSAGES ==================== */
static void process_stored(void)
{
    if (priorityCount == 0 && normalCount == 0) return;
    
    uint64_t now = k_uptime_get();
    if (now < nextRetryTime) return;
    
    struct StoredMsg *msgToSend = NULL;
    
    if (priorityCount > 0) {
        msgToSend = &priorityQueue[0];
    } else if (normalCount > 0) {
        msgToSend = &normalQueue[0];
        if (msgToSend->maxRetries != -1 && msgToSend->retryCount >= msgToSend->maxRetries) {
            LOG_INF("Retry limit reached, removing UID: %d", msgToSend->uid);
            delete_message(msgToSend->uid);
            return;
        }
    }
    
    if (msgToSend != NULL) {
        char full_msg[140];
        snprintf(full_msg, sizeof(full_msg), "%s%s", prefix, msgToSend->msg);
        
        int ret = lora_send(lora_dev, (uint8_t *)full_msg, strlen(full_msg));
        if (ret < 0) {
            LOG_ERR("LoRa send failed: %d", ret);
        } else {
            LOG_INF("Sent: %s [Attempt: %d]", full_msg, msgToSend->retryCount + 1);
        }
        
        msgToSend->retryCount++;
        nextRetryTime = k_uptime_get() + 1500 + (sys_rand32_get() % 2000);
    }
}

/* ==================== CHECK ACKNOWLEDGMENT ==================== */
static void check_ack(void)
{
    uint8_t buf[64];
    int size;
    int16_t rssi;
    int8_t snr;

    /* FIXED: lora_recv takes 6 arguments - dev, data, size, k_timeout_t timeout, rssi, snr */
    size = lora_recv(lora_dev, buf, sizeof(buf), K_MSEC(10), &rssi, &snr);
    
    if (size > 0) {
        char ack[64];
        memcpy(ack, buf, size);
        ack[size] = '\0';
        
        if (strncmp(ack, prefix_ack, strlen(prefix_ack)) != 0) return;
        
        char *p1 = strchr(ack, ',');
        if (!p1) return;
        char *p2 = strchr(p1 + 1, ',');
        if (!p2) return;
        
        char serial[16];
        char uid_str[16];
        strncpy(serial, p1 + 1, p2 - p1 - 1);
        serial[p2 - p1 - 1] = '\0';
        strcpy(uid_str, p2 + 1);
        
        uint32_t uid = (uint32_t)atoi(uid_str);
        
        if (strcmp(serial, serialNo) != 0) return;
        
        bool found = false;
        for (int i = 0; i < priorityCount; i++) {
            if (priorityQueue[i].uid == uid) { found = true; break; }
        }
        if (!found) {
            for (int i = 0; i < normalCount; i++) {
                if (normalQueue[i].uid == uid) { found = true; break; }
            }
        }
        
        if (found) {
            LOG_INF("ACK matched, stopping send");
            delete_message(uid);
        }
    }
}

/* ==================== MONITOR PIN ==================== */
static void monitor_pin(int pin, bool *state, uint64_t *startT,
                        const char *color, const char *dev, 
                        const char *ser, const char *pallet)
{
    int activeReading = gpio_pin_get(gpio_dev, pin);
    
    if (activeReading > 0) {
        if (!*state) {
            if (*startT == 0) {
                *startT = k_uptime_get();
            } else if (k_uptime_get() - *startT >= DEBOUNCE_TIME) {
                *state = true;
                LOG_INF("%s START (Validated)", color);
                send_lora_event(dev, ser, "STA", color, "", "", pallet, -1);
            }
        }
    } else {
        if (*state) {
            *state = false;
            uint32_t sec = (k_uptime_get() - *startT) / 1000;
            LOG_INF("%s STOP", color);
            if (sec > 0) {
                char duration[16];
                snprintf(duration, sizeof(duration), "%d", sec);
                send_lora_event(dev, ser, "SOL", color, duration, "", pallet, -1);
            }
            
            if (strcmp(color, "S1") == 0) msg_id_S1++;
            else if (strcmp(color, "S2") == 0) msg_id_S2++;
            else if (strcmp(color, "S3") == 0) msg_id_S3++;
            else if (strcmp(color, "S4") == 0) msg_id_S4++;
            else if (strcmp(color, "S5") == 0) msg_id_S5++;
            else if (strcmp(color, "S6") == 0) msg_id_S6++;
            save_uid();
        }
        *startT = 0;
    }
}

/* ==================== MAIN ==================== */
void main(void)
{
    int ret;
    
    LOG_INF("Starting LoRa Device - Zephyr RTOS");
    LOG_INF("Heltec WiFi LoRa 32 V3");
    
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return;
    }
    
    gpio_pin_configure(gpio_dev, S1_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, S2_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, S3_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, S4_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, S5_PIN, GPIO_INPUT);
    gpio_pin_configure(gpio_dev, S6_PIN, GPIO_INPUT);
    
    ret = nvs_init();
    if (ret < 0) {
        LOG_ERR("NVS init failed: %d", ret);
        return;
    }
    load_uid();
    
    lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
        return;
    }
    
    struct lora_modem_config config = {
        .frequency = LORA_FREQUENCY,
        .bandwidth = BW_125_KHZ,
        .datarate = SF_12,
        .coding_rate = CR_4_5,
        .preamble_len = 8,
        .tx_power = 17,
        .tx = true,
    };
    
    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed: %d", ret);
        return;
    }
    
    LOG_INF("LoRa initialized at %d Hz", LORA_FREQUENCY);
    LOG_INF("System ready. Reset Reason: %s", get_reset_reason());
    
    noSignalStartTime = k_uptime_get();
    send_power_on_sol();
    
    while (1) {
        monitor_pin(S1_PIN, &S1_State, &startS1, "S1", deviceID, serialNo, "P1");
        monitor_pin(S2_PIN, &S2_State, &startS2, "S2", deviceID, serialNo, "P1");
        monitor_pin(S3_PIN, &S3_State, &startS3, "S3", deviceID, serialNo, "P1");
        monitor_pin(S4_PIN, &S4_State, &startS4, "S4", deviceID, serialNo, "P2");
        monitor_pin(S5_PIN, &S5_State, &startS5, "S5", deviceID, serialNo, "P2");
        monitor_pin(S6_PIN, &S6_State, &startS6, "S6", deviceID, serialNo, "P2");
        
        if (k_uptime_get() - lastHeartbeat >= HEART_INTERVAL) {
            lastHeartbeat = k_uptime_get();
            send_heartbeat();
        }
        
        bool pinActive = (gpio_pin_get(gpio_dev, S1_PIN) ||
                         gpio_pin_get(gpio_dev, S2_PIN) ||
                         gpio_pin_get(gpio_dev, S3_PIN) ||
                         gpio_pin_get(gpio_dev, S4_PIN) ||
                         gpio_pin_get(gpio_dev, S5_PIN) ||
                         gpio_pin_get(gpio_dev, S6_PIN));
        
        if (pinActive || priorityCount > 0) {
            noSignalStartTime = k_uptime_get();
        } else {
            if (k_uptime_get() - noSignalStartTime >= NO_SIGNAL_TIMEOUT) {
                LOG_INF("No activity for 5 mins, restarting...");
                k_sleep(K_MSEC(100));
                /* Force hardware reset */
                /* NVIC_SystemReset(); */
            }
        }
        
        process_stored();
        check_ack();
        
        k_sleep(K_MSEC(10));
    }
}
