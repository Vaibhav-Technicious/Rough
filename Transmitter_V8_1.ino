#include <SPI.h>
#include <LoRa.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

// Watchdog Timeout in Seconds
#define WDT_TIMEOUT_SEC 15

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 26

// Default values (will be overridden by NVS)
float LORA_FREQUENCY = 433E6;

const int S1 = 32;
const int S2 = 33;
const int S3 = 25;
const int S4 = 27;
const int S5 = 14;
const int S6 = 13;

Preferences prefs;

String serialNo = "20262003";
String deviceID = "C003";
String prefix = "";
String prefix_ack = "ACK";

unsigned long noSignalStartTime = 0;
const unsigned long NO_SIGNAL_TIMEOUT = 300000; // 5 Minutes

unsigned long DEBOUNCE_TIME = 10000;
unsigned long lastChangeTime_S1 = 0, lastChangeTime_S2 = 0, lastChangeTime_S3 = 0;
unsigned long lastChangeTime_S4 = 0, lastChangeTime_S5 = 0, lastChangeTime_S6 = 0;

unsigned long msg_id_S1 = 5000;
unsigned long msg_id_S2 = 5000;
unsigned long msg_id_S3 = 5000;
unsigned long msg_id_S4 = 5000;
unsigned long msg_id_S5 = 5000;
unsigned long msg_id_S6 = 5000;

unsigned long global_uid = 5000;

bool S1_State = false, S2_State = false, S3_State = false;
bool S4_State = false, S5_State = false, S6_State = false;

unsigned long startS1 = 0, startS2 = 0, startS3 = 0;
unsigned long startS4 = 0, startS5 = 0, startS6 = 0;

/* ================= QUEUE MANAGEMENT ================= */
int MAX_PRIORITY_STORE = 100;
int MAX_NORMAL_STORE = 5;
#define MAX_Retry 10

struct StoredMsg {
  unsigned long uid;
  int retryCount;
  int maxRetries;
  String msg;
};

// Dynamic queues - use pointers to arrays of StoredMsg
StoredMsg* priorityQueue = nullptr;
StoredMsg* normalQueue = nullptr;
int priorityCount = 0;
int normalCount = 0;

unsigned long nextRetryTime = 0;
unsigned long lastHeartbeat = 0;
unsigned long HEART_INTERVAL = 120000;

/* ================= AP MODE VARIABLES ================= */
bool apModeActive = false;
bool loraActive = true;
bool rebootRequested = false;  // Flag to trigger reboot from web handler
WebServer server(80);
unsigned long s6HighStartTime = 0;
const unsigned long S6_AP_TRIGGER_TIME = 10000; // 10 seconds

/* ================= RESET REASON HELPER ================= */
String getResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "Power On";
    case ESP_RST_EXT:       return "External Reset";
    case ESP_RST_SW:        return "Software Restart";
    case ESP_RST_PANIC:     return "Exception Panic";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog";
    case ESP_RST_TASK_WDT:  return "Task Watchdog";
    case ESP_RST_WDT:       return "Other Watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep";
    case ESP_RST_BROWNOUT:  return "Brownout Reset";
    case ESP_RST_SDIO:      return "SDIO Reset";
    default:                return "Unknown";
  }
}

/* ================= NVS SAVE/LOAD FUNCTIONS ================= */
void saveAllSettings() {
    prefs.putULong("uid", global_uid);
    prefs.putULong("s1", msg_id_S1);
    prefs.putULong("s2", msg_id_S2);
    prefs.putULong("s3", msg_id_S3);
    prefs.putULong("s4", msg_id_S4);
    prefs.putULong("s5", msg_id_S5);
    prefs.putULong("s6", msg_id_S6);
    
    prefs.putFloat("loraFreq", LORA_FREQUENCY);
    prefs.putString("serialNo", serialNo);
    prefs.putString("deviceID", deviceID);
    prefs.putULong("debounce", DEBOUNCE_TIME);
    prefs.putInt("maxPriority", MAX_PRIORITY_STORE);
    prefs.putInt("maxNormal", MAX_NORMAL_STORE);
    prefs.putULong("heartInt", HEART_INTERVAL);
}

void loadAllSettings() {
    global_uid = prefs.getULong("uid", 5000);
    msg_id_S1 = prefs.getULong("s1", 5000);
    msg_id_S2 = prefs.getULong("s2", 5000);
    msg_id_S3 = prefs.getULong("s3", 5000);
    msg_id_S4 = prefs.getULong("s4", 5000);
    msg_id_S5 = prefs.getULong("s5", 5000);
    msg_id_S6 = prefs.getULong("s6", 5000);
    
    LORA_FREQUENCY = prefs.getFloat("loraFreq", 433E6);
    serialNo = prefs.getString("serialNo", "20262003");
    deviceID = prefs.getString("deviceID", "C003");
    DEBOUNCE_TIME = prefs.getULong("debounce", 10000);
    MAX_PRIORITY_STORE = prefs.getInt("maxPriority", 100);
    MAX_NORMAL_STORE = prefs.getInt("maxNormal", 5);
    HEART_INTERVAL = prefs.getULong("heartInt", 120000);
}

/* ================= QUEUE INITIALIZATION ================= */
void initQueues() {
    // Delete old arrays using delete[] (properly calls String destructors)
    if (priorityQueue != nullptr) {
        delete[] priorityQueue;
        priorityQueue = nullptr;
    }
    if (normalQueue != nullptr) {
        delete[] normalQueue;
        normalQueue = nullptr;
    }
    
    // Allocate using new[] so String constructors are called properly
    priorityQueue = new StoredMsg[MAX_PRIORITY_STORE];
    normalQueue = new StoredMsg[MAX_NORMAL_STORE];
    
    if (priorityQueue == nullptr || normalQueue == nullptr) {
        Serial.println("FATAL: Queue allocation failed!");
        ESP.restart();
    }
    
    priorityCount = 0;
    normalCount = 0;
}

/* ================= WEB PAGE ================= */
String getWebPage() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Device Configuration</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            padding: 20px;
            color: #e0e0e0;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
            background: rgba(30, 30, 50, 0.95);
            border-radius: 16px;
            padding: 30px;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
        }
        h1 {
            text-align: center;
            color: #00d4ff;
            margin-bottom: 8px;
            font-size: 24px;
        }
        .subtitle {
            text-align: center;
            color: #888;
            font-size: 12px;
            margin-bottom: 25px;
        }
        .form-group { margin-bottom: 18px; }
        label {
            display: block;
            margin-bottom: 6px;
            color: #aaa;
            font-size: 13px;
            font-weight: 500;
        }
        .unit { color: #666; font-size: 11px; }
        input[type="text"], input[type="number"] {
            width: 100%;
            padding: 12px 14px;
            border: 2px solid #333;
            border-radius: 8px;
            background: #0d0d1a;
            color: #00d4ff;
            font-size: 15px;
            transition: border-color 0.3s;
        }
        input:focus { outline: none; border-color: #00d4ff; }
        .btn-container { display: flex; gap: 12px; margin-top: 25px; }
        button {
            flex: 1;
            padding: 14px;
            border: none;
            border-radius: 8px;
            font-size: 15px;
            font-weight: 600;
            cursor: pointer;
            transition: transform 0.2s, opacity 0.2s;
        }
        button:active { transform: scale(0.98); }
        .btn-save {
            background: linear-gradient(135deg, #00d4ff, #0099cc);
            color: #000;
        }
        .btn-reboot {
            background: linear-gradient(135deg, #ff6b6b, #cc4444);
            color: #fff;
        }
        .status {
            text-align: center;
            padding: 12px;
            border-radius: 8px;
            margin-top: 15px;
            font-weight: 600;
            display: none;
        }
        .status.success {
            background: rgba(0, 212, 100, 0.2);
            color: #00d464;
            display: block;
        }
        .info-box {
            background: rgba(0, 212, 255, 0.1);
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 8px;
            padding: 12px;
            margin-bottom: 20px;
            font-size: 12px;
            color: #00d4ff;
            text-align: center;
        }
        .note {
            text-align: center;
            color: #ffc800;
            font-size: 11px;
            margin-top: 15px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>⚙️ Device Configuration</h1>
        <p class="subtitle">AP Mode Active - Save will reboot device</p>
        
        <div class="info-box">
            Device ID: <strong>)rawliteral" + deviceID + R"rawliteral(</strong>
        </div>
        
        <form id="configForm">
            <div class="form-group">
                <label>LoRa Frequency <span class="unit">(Hz, e.g., 433000000)</span></label>
                <input type="number" name="loraFreq" value=")rawliteral" + String((unsigned long)LORA_FREQUENCY) + R"rawliteral(" step="1000000">
            </div>
            
            <div class="form-group">
                <label>Serial Number</label>
                <input type="text" name="serialNo" value=")rawliteral" + serialNo + R"rawliteral(">
            </div>
            
            <div class="form-group">
                <label>Device ID</label>
                <input type="text" name="deviceID" value=")rawliteral" + deviceID + R"rawliteral(">
            </div>
            
            <div class="form-group">
                <label>Debounce Time <span class="unit">(milliseconds)</span></label>
                <input type="number" name="debounce" value=")rawliteral" + String(DEBOUNCE_TIME) + R"rawliteral(" min="0">
            </div>
            
            <div class="form-group">
                <label>Max Priority Store</label>
                <input type="number" name="maxPriority" value=")rawliteral" + String(MAX_PRIORITY_STORE) + R"rawliteral(" min="1" max="500">
            </div>
            
            <div class="form-group">
                <label>Max Normal Store</label>
                <input type="number" name="maxNormal" value=")rawliteral" + String(MAX_NORMAL_STORE) + R"rawliteral(" min="1" max="100">
            </div>
            
            <div class="form-group">
                <label>Heartbeat Interval <span class="unit">(milliseconds)</span></label>
                <input type="number" name="heartInt" value=")rawliteral" + String(HEART_INTERVAL) + R"rawliteral(" min="1000">
            </div>
            
            <div class="btn-container">
                <button type="submit" class="btn-save">💾 Save & Reboot</button>
                <button type="button" class="btn-reboot" onclick="rebootDevice()">🔄 Reboot</button>
            </div>
        </form>
        
        <p class="note">⚠ Saving will store settings and reboot the device</p>
        
        <div id="status" class="status"></div>
    </div>
    
    <script>
        document.getElementById('configForm').addEventListener('submit', async function(e) {
            e.preventDefault();
            const formData = new FormData(this);
            const params = new URLSearchParams();
            for (const [key, value] of formData) {
                params.append(key, value);
            }
            
            const status = document.getElementById('status');
            status.innerHTML = '⏳ Saving settings...';
            status.className = 'status success';
            
            try {
                await fetch('/save', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: params.toString()
                });
            } catch (err) {
                // Expected - device is rebooting
            }
            
            // Show reboot message regardless
            let countdown = 8;
            const interval = setInterval(() => {
                countdown--;
                if (countdown <= 0) {
                    clearInterval(interval);
                    document.body.innerHTML = '<div style="text-align:center;padding:50px;font-family:Arial,sans-serif;">' +
                        '<h2 style="color:#00d4ff;">✓ Settings Saved - Device Rebooting</h2>' +
                        '<p style="color:#888;margin-top:20px;">The device is restarting with new settings.</p>' +
                        '<p style="color:#888;margin-top:10px;font-size:12px;">You may disconnect from the WiFi network.</p>' +
                        '</div>';
                } else {
                    status.innerHTML = '✓ Settings saved!<br>Device rebooting in ' + countdown + ' seconds...';
                }
            }, 1000);
        });
        
        function rebootDevice() {
            if (confirm('Are you sure you want to reboot the device?')) {
                fetch('/reboot').catch(() => {});
                setTimeout(() => {
                    document.body.innerHTML = '<div style="text-align:center;padding:50px;color:#00d4ff;font-size:20px;">Device rebooting...</div>';
                }, 1000);
            }
        }
    </script>
</body>
</html>
)rawliteral";
    return html;
}

/* ================= WEB SERVER HANDLERS ================= */
void handleRoot() {
    server.send(200, "text/html", getWebPage());
}

void handleSave() {
    // Read and apply all form values
    if (server.hasArg("loraFreq")) {
        float newFreq = server.arg("loraFreq").toFloat();
        if (newFreq > 0) LORA_FREQUENCY = newFreq;
    }
    if (server.hasArg("serialNo")) {
        serialNo = server.arg("serialNo");
    }
    if (server.hasArg("deviceID")) {
        deviceID = server.arg("deviceID");
    }
    if (server.hasArg("debounce")) {
        DEBOUNCE_TIME = server.arg("debounce").toInt();
    }
    if (server.hasArg("maxPriority")) {
        int newVal = server.arg("maxPriority").toInt();
        if (newVal > 0) MAX_PRIORITY_STORE = newVal;
    }
    if (server.hasArg("maxNormal")) {
        int newVal = server.arg("maxNormal").toInt();
        if (newVal > 0) MAX_NORMAL_STORE = newVal;
    }
    if (server.hasArg("heartInt")) {
        HEART_INTERVAL = server.arg("heartInt").toInt();
    }
    
    // Save all settings to NVS
    saveAllSettings();
    Serial.println("Settings saved via web interface");
    
    // Send response to client
    server.send(200, "text/plain", "OK");
    server.client().flush();
    
    // Signal that reboot is requested (handled in main loop)
    // This avoids calling ESP.restart() from within the web handler
    // which can cause issues with the HTTP response not being sent
    rebootRequested = true;
}

void handleReboot() {
    server.send(200, "text/plain", "Rebooting...");
    server.client().flush();
    rebootRequested = true;
}

void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

/* ================= AP MODE FUNCTIONS ================= */
void startAPMode() {
    if (apModeActive) return;
    
    Serial.println("Starting AP Mode...");
    
    // Turn off LoRa
    if (loraActive) {
        LoRa.sleep();
        loraActive = false;
        Serial.println("LoRa module disabled");
    }
    
    // Start WiFi AP
    String apName = deviceID;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str());
    
    Serial.print("AP SSID: ");
    Serial.println(apName);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    
    // Start mDNS
    if (MDNS.begin(deviceID.c_str())) {
        Serial.print("mDNS started: http://");
        Serial.print(deviceID);
        Serial.println(".local");
    } else {
        Serial.println("mDNS failed to start");
    }
    
    // Setup web server routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/reboot", HTTP_GET, handleReboot);
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("Web server started");
    
    apModeActive = true;
}

/* ================= SAVING FUNCTIONS ================= */
void saveUID() {
    prefs.putULong("uid", global_uid);
    prefs.putULong("s1", msg_id_S1);
    prefs.putULong("s2", msg_id_S2);
    prefs.putULong("s3", msg_id_S3);
    prefs.putULong("s4", msg_id_S4);
    prefs.putULong("s5", msg_id_S5);
    prefs.putULong("s6", msg_id_S6);
}

void loadUID() {
    global_uid = prefs.getULong("uid", 5000);
    msg_id_S1 = prefs.getULong("s1", 5000);
    msg_id_S2 = prefs.getULong("s2", 5000);
    msg_id_S3 = prefs.getULong("s3", 5000);
    msg_id_S4 = prefs.getULong("s4", 5000);
    msg_id_S5 = prefs.getULong("s5", 5000);
    msg_id_S6 = prefs.getULong("s6", 5000);
}

/* ================= QUEUE STORAGE LOGIC ================= */
void storeMessage(unsigned long uid, String msg, int maxRetries) {
  if (maxRetries == -1) {
    for (int i = 0; i < priorityCount; i++) {
      if (priorityQueue[i].uid == uid) return;
    }
    if (priorityCount >= MAX_PRIORITY_STORE) {
      Serial.println("⚠ PRIORITY QUEUE FULL → Removing Oldest");
      for (int i = 1; i < MAX_PRIORITY_STORE; i++) {
        priorityQueue[i - 1] = priorityQueue[i];
      }
      priorityCount--;
    }
    priorityQueue[priorityCount].uid = uid;
    priorityQueue[priorityCount].msg = msg;
    priorityQueue[priorityCount].retryCount = 0;
    priorityQueue[priorityCount].maxRetries = -1;
    priorityCount++;
    Serial.println("STORED [PRIORITY] -> " + msg);
    Serial.println("Total Priority Stored -> " + String(priorityCount));
  } 
  else {
    for (int i = 0; i < normalCount; i++) {
      if (normalQueue[i].uid == uid) return;
    }
    if (normalCount >= MAX_NORMAL_STORE) {
      Serial.println("⚠ NORMAL QUEUE FULL → Removing Oldest");
      for (int i = 1; i < MAX_NORMAL_STORE; i++) {
        normalQueue[i - 1] = normalQueue[i];
      }
      normalCount--;
    }
    normalQueue[normalCount].uid = uid;
    normalQueue[normalCount].msg = msg;
    normalQueue[normalCount].retryCount = 0;
    normalQueue[normalCount].maxRetries = maxRetries;
    normalCount++;
    Serial.println("STORED [NORMAL] -> " + msg);
    Serial.println("Total Normal Stored -> " + String(normalCount));
  }
}

/* ================= DELETION LOGIC ================= */
void deleteMessage(unsigned long uid) {
  for (int i = 0; i < priorityCount; i++) {
    if (priorityQueue[i].uid == uid) {
      Serial.println("DELETING PRIORITY UID -> " + String(uid));
      for (int j = i + 1; j < priorityCount; j++) {
        priorityQueue[j - 1] = priorityQueue[j];
      }
      priorityCount--;
      Serial.println("Remaining Priority -> " + String(priorityCount));
      return;
    }
  }

  for (int i = 0; i < normalCount; i++) {
    if (normalQueue[i].uid == uid) {
      Serial.println("DELETING NORMAL UID -> " + String(uid));
      for (int j = i + 1; j < normalCount; j++) {
        normalQueue[j - 1] = normalQueue[j];
      }
      normalCount--;
      Serial.println("Remaining Normal -> " + String(normalCount));
      return;
    }
  }
}

/* ================= TRANSMISSION LOGIC ================= */
void processStored() {
  if (!loraActive) return;
  if (priorityCount == 0 && normalCount == 0) return;
  if (millis() < nextRetryTime) return;

  StoredMsg *msgToSend = NULL;

  if (priorityCount > 0) {
    msgToSend = &priorityQueue[0];
  } 
  else if (normalCount > 0) {
    msgToSend = &normalQueue[0];
    if (msgToSend->maxRetries != -1 && msgToSend->retryCount >= msgToSend->maxRetries) {
      Serial.println("RETRY LIMIT REACHED -> REMOVING NORMAL UID: " + String(msgToSend->uid));
      deleteMessage(msgToSend->uid);
      return;
    }
  }

  if (msgToSend != NULL) {
    String msg = prefix + msgToSend->msg;
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    LoRa.receive();

    msgToSend->retryCount++;
    nextRetryTime = millis() + random(1500, 3500);
    Serial.println("SENT -> " + msg + " [Attempt: " + String(msgToSend->retryCount) + "]");
  }
}

/* ================= ACKNOWLEDGMENT LOGIC ================= */
void checkAck() {
  if (!loraActive) return;
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String ack = "";
    while (LoRa.available()) {
      ack += (char)LoRa.read();
    }
    ack.trim();

    if (!ack.startsWith(prefix_ack)) {
      return;
    }
    int p1 = ack.indexOf(',');
    int p2 = ack.indexOf(',', p1 + 1);
    if (p1 == -1 || p2 == -1) {
      return;
    }
    String serial = ack.substring(p1 + 1, p2);
    String uidStr = ack.substring(p2 + 1);
    serial.trim();
    uidStr.trim();
    unsigned long uid = strtoul(uidStr.c_str(), NULL, 10);

    if (serial != serialNo) {
      return;
    }

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
      Serial.println(" ACK MATCHED → STOP SENDING");
      deleteMessage(uid);
    } else {
      Serial.println(" UID NOT FOUND");
    }
  }
}

/* ================= EVENT HANDLERS ================= */
void sendLoRaEvent(String deviceID, String serialNo, String eventType, String color, String duration, String reason, String pallet, int maxRetries) {

  if (eventType == "STA") duration = "no_duration";
  if (color != "DWN") reason = "no_reason";
  if (color == "DWN" && duration == "") duration = "0";

  String msg_id = "";

  if (color == "S1") msg_id = String(msg_id_S1);
  else if (color == "S2") msg_id = String(msg_id_S2);
  else if (color == "S3") msg_id = String(msg_id_S3);
  else if (color == "S4") msg_id = String(msg_id_S4);
  else if (color == "S5") msg_id = String(msg_id_S5);
  else if (color == "S6") msg_id = String(msg_id_S6);
  else if (color == "DWN") msg_id = "1";

  unsigned long uid = global_uid++;
  saveUID();
  String message = String(uid) + "," + deviceID + "," + msg_id + "," + eventType + "," + serialNo + "," + color + "," + duration + "," + reason + "," + pallet;
  
  storeMessage(uid, message, maxRetries);
}

void sendHeartbeat() {
  sendLoRaEvent(deviceID, serialNo, "STA", "DWN", "no_duration", "Power Down", "no_pallet", MAX_Retry);
}

void sendPowerOnSOL() {
  String reason = getResetReason();
  sendLoRaEvent(deviceID, serialNo, "SOL", "DWN", "no_duration", reason, "no_pallet", -1);
}

void monitorPin(int pin, bool &state, unsigned long &startT,
                unsigned long &lastChangeTime,
                String color, String dev, String ser, String pallet)
{
  bool activeReading = (digitalRead(pin) == HIGH);

  if (activeReading) {
    if (!state) {
      if (startT == 0) {
        startT = millis(); 
      } 
      else if (millis() - startT >= DEBOUNCE_TIME) {
        state = true;
        Serial.println(color + " START (Validated)");
        sendLoRaEvent(dev, ser, "STA", color, "", "", pallet, -1);
      }
    }
  } 
  else {
    if (state) {
      state = false;
      unsigned long sec = (millis() - startT) / 1000;
      Serial.println(color + " STOP");
      if (sec > 0) {
        sendLoRaEvent(dev, ser, "SOL", color, String(sec), "", pallet, -1);
      }
      
      if (color == "S1") msg_id_S1++;
      else if (color == "S2") msg_id_S2++;
      else if (color == "S3") msg_id_S3++;
      else if (color == "S4") msg_id_S4++;
      else if (color == "S5") msg_id_S5++;
      else if (color == "S6") msg_id_S6++;
      saveUID();
    }
    startT = 0; 
  }
}

/* ================= S6 AP MODE HANDLER ================= */
void checkS6ForAPMode() {
  if (apModeActive || rebootRequested) return;
  
  bool s6Active = (digitalRead(S6) == HIGH);
  
  if (s6Active) {
    if (s6HighStartTime == 0) {
      s6HighStartTime = millis();
    } else if (millis() - s6HighStartTime >= S6_AP_TRIGGER_TIME) {
      startAPMode();
      s6HighStartTime = 0;
    }
  } else {
    s6HighStartTime = 0;
  }
}

bool isSystemActive() {
  bool pinActive = digitalRead(S1) || digitalRead(S2) || digitalRead(S3) || 
                   digitalRead(S4) || digitalRead(S5) || digitalRead(S6);
                   
  return pinActive || (priorityCount > 0) || apModeActive;
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(100);

  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  prefs.begin("lora", false);
  
  loadAllSettings();
  loadUID();
  initQueues();

  pinMode(S1, INPUT_PULLDOWN);
  pinMode(S2, INPUT_PULLDOWN);
  pinMode(S3, INPUT_PULLDOWN);
  pinMode(S4, INPUT_PULLDOWN);
  pinMode(S5, INPUT_PULLDOWN);
  pinMode(S6, INPUT_PULLDOWN);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa FAILED → RESTARTING ESP32...");
    delay(1000);
    ESP.restart();
  }
  
  LoRa.receive();

  randomSeed(esp_random());
  Serial.println("SYSTEM READY. Reset Reason: " + getResetReason());
  Serial.println("Device ID: " + deviceID);
  Serial.println("Serial No: " + serialNo);
  Serial.println("LoRa Frequency: " + String(LORA_FREQUENCY));
  Serial.println("Debounce Time: " + String(DEBOUNCE_TIME) + " ms");
  
  noSignalStartTime = millis(); 
  
  sendPowerOnSOL();
}

/* ================= MAIN LOOP ================= */
void loop() {
  esp_task_wdt_reset();

  // Handle reboot request from web (must be done outside web handler)
  if (rebootRequested) {
    Serial.println("Reboot requested - restarting in 1 second...");
    delay(1000);
    ESP.restart();
  }

  if (apModeActive) {
    server.handleClient();
  }

  checkS6ForAPMode();

  monitorPin(S1, S1_State, startS1, lastChangeTime_S1, "S1", deviceID, serialNo, "P1");
  monitorPin(S2, S2_State, startS2, lastChangeTime_S2, "S2", deviceID, serialNo, "P1");
  monitorPin(S3, S3_State, startS3, lastChangeTime_S3, "S3", deviceID, serialNo, "P1");
  monitorPin(S4, S4_State, startS4, lastChangeTime_S4, "S4", deviceID, serialNo, "P2");
  monitorPin(S5, S5_State, startS5, lastChangeTime_S5, "S5", deviceID, serialNo, "P2");

  if (millis() - lastHeartbeat >= HEART_INTERVAL) {
    lastHeartbeat = millis();
    if (!apModeActive) {
      sendHeartbeat();
    }
  }
  
  if (isSystemActive()) {
    noSignalStartTime = millis();
  } else {
    if (millis() - noSignalStartTime >= NO_SIGNAL_TIMEOUT) {
      Serial.println("No activity for 5 mins → Restarting...");
      delay(100);
      ESP.restart();
    }
  }
  
  processStored();
  checkAck();
}
