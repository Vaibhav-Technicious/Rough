#include <SPI.h>
#include <LoRa.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <WiFi.h>
#include <WebServer.h>

// ================= CONFIGURATION (DEFAULTS) =================
unsigned long cfg_WDT_TIMEOUT_SEC   = 20;
unsigned long cfg_LORA_FREQUENCY    = 433000000UL; // stored as Hz
unsigned long cfg_DEBOUNCE_TIME     = 10000;
unsigned long cfg_HEART_INTERVAL    = 120000;
unsigned long cfg_NO_SIGNAL_TIMEOUT = 300000;
int           cfg_MAX_PRIORITY_STORE = 100;
int           cfg_MAX_NORMAL_STORE   = 5;
int           cfg_MAX_Retry          = 10;

// ================= NEW: CONFIG_PIN debounce state =================
// Add these near the other CONFIG_PIN globals
unsigned long configPinLastDebounceTime = 0;
const unsigned long CONFIG_PIN_DEBOUNCE_MS = 200;   // 50 ms debounce
bool configPinLastReading = HIGH;
bool configPinStableState = HIGH;                    // debounced state


String cfg_serialNo = "20260004";
String cfg_deviceID = "C004";

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 26

const int S1 = 32;
const int S2 = 33;
const int S3 = 25;
const int S4 = 27;
const int S5 = 14;
const int S6 = 13;
const int CONFIG_PIN = 15; // GPIO15 - pull-up, short to GND to enter config mode

// NEW: how long the config pin must be held continuously to force a restart
const unsigned long CONFIG_PIN_LONG_PRESS_RESTART_MS = 5000;

Preferences prefs;

String serialNo = "";
String deviceID = "";
String prefix = "";
String prefix_ack = "ACK";

unsigned long noSignalStartTime = 0;

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

// NEW: track continuous LOW duration on CONFIG_PIN
unsigned long configPinPressStart = 0;
bool configPinLongPressTriggered = false;

// ================= QUEUE MANAGEMENT =================
#define HARD_MAX_PRIORITY 200
#define HARD_MAX_NORMAL   20

struct StoredMsg {
  unsigned long uid;
  int retryCount;
  int maxRetries;
  String msg;
};

StoredMsg priorityQueue[HARD_MAX_PRIORITY];
int priorityCount = 0;

StoredMsg normalQueue[HARD_MAX_NORMAL];
int normalCount = 0;

unsigned long nextRetryTime = 0;
unsigned long lastHeartbeat = 0;

// ================= WIFI / WEB SERVER =================
WebServer server(80);
bool configMode = false;

// ================= RESET REASON HELPER =================
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

// ================= CONFIG LOAD / SAVE =================
void loadConfig() {
  cfg_deviceID          = prefs.getString("deviceID", "C004");
  cfg_serialNo          = prefs.getString("serialNo", "20260004");
  cfg_LORA_FREQUENCY    = prefs.getULong("loraFreq", 433000000UL);
  cfg_DEBOUNCE_TIME     = prefs.getULong("debounce", 10000);
  cfg_HEART_INTERVAL    = prefs.getULong("heartInt", 120000);
  cfg_NO_SIGNAL_TIMEOUT = prefs.getULong("noSigTO", 300000);
  cfg_MAX_PRIORITY_STORE= prefs.getInt("maxPrio", 100);
  cfg_MAX_NORMAL_STORE  = prefs.getInt("maxNorm", 5);
  cfg_MAX_Retry         = prefs.getInt("maxRetry", 10);
  cfg_WDT_TIMEOUT_SEC   = prefs.getULong("wdtTO", 15);

  if (cfg_MAX_PRIORITY_STORE > HARD_MAX_PRIORITY) cfg_MAX_PRIORITY_STORE = HARD_MAX_PRIORITY;
  if (cfg_MAX_PRIORITY_STORE < 1) cfg_MAX_PRIORITY_STORE = 1;
  if (cfg_MAX_NORMAL_STORE > HARD_MAX_NORMAL) cfg_MAX_NORMAL_STORE = HARD_MAX_NORMAL;
  if (cfg_MAX_NORMAL_STORE < 1) cfg_MAX_NORMAL_STORE = 1;
  if (cfg_WDT_TIMEOUT_SEC < 5) cfg_WDT_TIMEOUT_SEC = 5;

  serialNo = cfg_serialNo;
  deviceID = cfg_deviceID;
}

void saveConfig() {
  prefs.putString("deviceID", cfg_deviceID);
  prefs.putString("serialNo", cfg_serialNo);
  prefs.putULong("loraFreq", cfg_LORA_FREQUENCY);
  prefs.putULong("debounce", cfg_DEBOUNCE_TIME);
  prefs.putULong("heartInt", cfg_HEART_INTERVAL);
  prefs.putULong("noSigTO", cfg_NO_SIGNAL_TIMEOUT);
  prefs.putInt("maxPrio", cfg_MAX_PRIORITY_STORE);
  prefs.putInt("maxNorm", cfg_MAX_NORMAL_STORE);
  prefs.putInt("maxRetry", cfg_MAX_Retry);
  prefs.putULong("wdtTO", cfg_WDT_TIMEOUT_SEC);
}

// ================= SAVING FUNCTIONS (UIDs) =================
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

// ================= QUEUE STORAGE LOGIC =================
void storeMessage(unsigned long uid, String msg, int maxRetries) {
  if (maxRetries == -1) {
    for (int i = 0; i < priorityCount; i++) {
      if (priorityQueue[i].uid == uid) return;
    }
    if (priorityCount >= cfg_MAX_PRIORITY_STORE) {
      Serial.println("PRIORITY QUEUE FULL -> Removing Oldest");
      for (int i = 1; i < priorityCount; i++) {
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
    if (normalCount >= cfg_MAX_NORMAL_STORE) {
      Serial.println("NORMAL QUEUE FULL -> Removing Oldest");
      for (int i = 1; i < normalCount; i++) {
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

// ================= DELETION LOGIC =================
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

// ================= TRANSMISSION LOGIC =================
void processStored() {
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

// ================= ACKNOWLEDGMENT LOGIC =================
void checkAck() {
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
      Serial.println("ACK MATCHED -> STOP SENDING");
      deleteMessage(uid);
    } else {
      Serial.println("UID NOT FOUND");
    }
  }
}

// ================= EVENT HANDLERS =================
void sendLoRaEvent(String deviceID_, String serialNo_, String eventType, String color, String duration, String reason, String pallet, int maxRetries) {
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
  String message = String(uid) + "," + deviceID_ + "," + msg_id + "," + eventType + "," + serialNo_ + "," + color + "," + duration + "," + reason + "," + pallet;
  storeMessage(uid, message, maxRetries);
}

void sendHeartbeat() {
  sendLoRaEvent(deviceID, serialNo, "STA", "DWN", "no_duration", "Power Down", "no_pallet", cfg_MAX_Retry);
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
      else if (millis() - startT >= cfg_DEBOUNCE_TIME) {
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

bool isSystemActive() {
  bool pinActive = digitalRead(S1) || digitalRead(S2) || digitalRead(S3) ||
                   digitalRead(S4) || digitalRead(S5) || digitalRead(S6);
  return pinActive || (priorityCount > 0);
}

// ================= WEB PAGE =================
String htmlEscape(const String &s) {
  String out = s;
  out.replace("&", "&amp;");
  out.replace("<", "&lt;");
  out.replace(">", "&gt;");
  out.replace("\"", "&quot;");
  return out;
}

String buildConfigPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Device Config</title>";
  html += "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f4f4f4;}";
  html += "h1{color:#333;}form{background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,0.1);max-width:600px;margin:auto;}";
  html += "label{display:block;margin-top:12px;font-weight:bold;}";
  html += "input{width:100%;padding:8px;margin-top:4px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;}";
  html += ".btn{margin-top:20px;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;font-size:16px;}";
  html += ".save{background:#28a745;color:#fff;margin-right:10px;}";
  html += ".reset{background:#dc3545;color:#fff;}</style></head><body>";
  html += "<h1>Device Configuration</h1>";
  html += "<form method='POST' action='/save'>";

  html += "<label>Device ID</label><input name='deviceID' value='" + htmlEscape(cfg_deviceID) + "'>";
  html += "<label>Serial No</label><input name='serialNo' value='" + htmlEscape(cfg_serialNo) + "'>";
  html += "<label>LoRa Frequency (Hz)</label><input name='loraFreq' type='number' value='" + String(cfg_LORA_FREQUENCY) + "'>";
  html += "<label>Debounce Time (ms)</label><input name='debounce' type='number' value='" + String(cfg_DEBOUNCE_TIME) + "'>";
  html += "<label>Heartbeat Interval (ms)</label><input name='heartInt' type='number' value='" + String(cfg_HEART_INTERVAL) + "'>";
  html += "<label>No Signal Timeout (ms)</label><input name='noSigTO' type='number' value='" + String(cfg_NO_SIGNAL_TIMEOUT) + "'>";
  html += "<label>Max Priority Store</label><input name='maxPrio' type='number' value='" + String(cfg_MAX_PRIORITY_STORE) + "'>";
  html += "<label>Max Normal Store</label><input name='maxNorm' type='number' value='" + String(cfg_MAX_NORMAL_STORE) + "'>";
  html += "<label>Max Retry</label><input name='maxRetry' type='number' value='" + String(cfg_MAX_Retry) + "'>";
  html += "<label>Watchdog Timeout (sec)</label><input name='wdtTO' type='number' value='" + String(cfg_WDT_TIMEOUT_SEC) + "'>";

  html += "<div style='margin-top:25px;'>";
  html += "<button type='submit' class='btn save'>SAVE &amp; RESTART</button>";
  html += "<button type='button' class='btn reset' onclick='if(confirm(\"Restart ESP32?\")){window.location=\"/reset\";}'>RESET</button>";
  html += "</div>";
  html += "</form></body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", buildConfigPage());
}

void handleSave() {
  if (server.hasArg("deviceID"))    cfg_deviceID = server.arg("deviceID");
  if (server.hasArg("serialNo"))    cfg_serialNo = server.arg("serialNo");
  if (server.hasArg("loraFreq"))    cfg_LORA_FREQUENCY = strtoul(server.arg("loraFreq").c_str(), NULL, 10);
  if (server.hasArg("debounce"))    cfg_DEBOUNCE_TIME = strtoul(server.arg("debounce").c_str(), NULL, 10);
  if (server.hasArg("heartInt"))    cfg_HEART_INTERVAL = strtoul(server.arg("heartInt").c_str(), NULL, 10);
  if (server.hasArg("noSigTO"))     cfg_NO_SIGNAL_TIMEOUT = strtoul(server.arg("noSigTO").c_str(), NULL, 10);
  if (server.hasArg("maxPrio"))     cfg_MAX_PRIORITY_STORE = server.arg("maxPrio").toInt();
  if (server.hasArg("maxNorm"))     cfg_MAX_NORMAL_STORE = server.arg("maxNorm").toInt();
  if (server.hasArg("maxRetry"))    cfg_MAX_Retry = server.arg("maxRetry").toInt();
  if (server.hasArg("wdtTO"))       cfg_WDT_TIMEOUT_SEC = strtoul(server.arg("wdtTO").c_str(), NULL, 10);

  if (cfg_MAX_PRIORITY_STORE > HARD_MAX_PRIORITY) cfg_MAX_PRIORITY_STORE = HARD_MAX_PRIORITY;
  if (cfg_MAX_PRIORITY_STORE < 1) cfg_MAX_PRIORITY_STORE = 1;
  if (cfg_MAX_NORMAL_STORE > HARD_MAX_NORMAL) cfg_MAX_NORMAL_STORE = HARD_MAX_NORMAL;
  if (cfg_MAX_NORMAL_STORE < 1) cfg_MAX_NORMAL_STORE = 1;
  if (cfg_WDT_TIMEOUT_SEC < 5) cfg_WDT_TIMEOUT_SEC = 5;

  saveConfig();

  server.send(200, "text/html", "<!DOCTYPE html><html><body style='font-family:Arial;text-align:center;margin-top:50px;'>"
                                 "<h2>Configuration Saved</h2><p>Device will restart in 3 seconds...</p></body></html>");
  delay(3000);
  ESP.restart();
}

void handleReset() {
  server.send(200, "text/html", "<!DOCTYPE html><html><body style='font-family:Arial;text-align:center;margin-top:50px;'>"
                                 "<h2>Restarting...</h2></body></html>");
  delay(1000);
  ESP.restart();
}

// ================= CONFIG MODE =================
void enterConfigMode() {
  configMode = true;
  // In enterConfigMode(), add:
  esp_task_wdt_delete(NULL);   // Unsubscribe current task
  Serial.println("=== ENTERING CONFIGURATION MODE ===");

  LoRa.sleep();
  Serial.println("LoRa module turned OFF.");

  deviceID = prefs.getString("deviceID", cfg_deviceID);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);

  WiFi.mode(WIFI_AP);
  String apName = deviceID;
  WiFi.softAP(apName.c_str(), "12345678");
  Serial.print("AP Started. SSID: ");
  Serial.println(apName);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", handleReset);
  server.begin();
  Serial.println("Web server started.");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Transmitter starting...");

  prefs.begin("lora", false);
  loadConfig();
  loadUID();

  pinMode(S1, INPUT_PULLDOWN);
  pinMode(S2, INPUT_PULLDOWN);
  pinMode(S3, INPUT_PULLDOWN);
  pinMode(S4, INPUT_PULLDOWN);
  pinMode(S5, INPUT_PULLDOWN);
  pinMode(S6, INPUT_PULLDOWN);
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  // Check config pin - if pulled low, enter config mode
  delay(100);
  if (digitalRead(CONFIG_PIN) == LOW) {
    // Seed the long-press timer so the 5-second restart logic
    // can also work if the button is held from boot.
    configPinPressStart = millis();
    enterConfigMode();
    return;
  }

  esp_task_wdt_init(cfg_WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(cfg_LORA_FREQUENCY)) {
    Serial.println("LoRa FAILED -> RESTARTING ESP32...");
    delay(1000);
    ESP.restart();
  }

  randomSeed(esp_random());
  Serial.println("SYSTEM READY. Reset Reason: " + getResetReason());
  Serial.print("Frequency: "); Serial.println(cfg_LORA_FREQUENCY);

  noSignalStartTime = millis();
  sendPowerOnSOL();
}

// ================= MAIN LOOP =================
// ================= MAIN LOOP =================
void loop() {
  // ---- Debounce CONFIG_PIN first ----
  bool rawReading = digitalRead(CONFIG_PIN);
  if (rawReading != configPinLastReading) {
    configPinLastDebounceTime = millis();
    configPinLastReading = rawReading;
  }
  if ((millis() - configPinLastDebounceTime) >= CONFIG_PIN_DEBOUNCE_MS) {
    configPinStableState = rawReading;
  }

  // ---- CONFIG_PIN press/release handling ----
  // We track press start on the debounced LOW transition,
  // and decide action only on the debounced release (HIGH transition).
  if (configPinStableState == LOW) {
    // Button is (stably) pressed
    if (configPinPressStart == 0) {
      configPinPressStart = millis();
      configPinLongPressTriggered = false;
    }
    // We do NOT act here. Wait for release.
  } else {
    // Button is (stably) released
    if (configPinPressStart != 0) {
      unsigned long pressDuration = millis() - configPinPressStart;

      if (pressDuration >= CONFIG_PIN_LONG_PRESS_RESTART_MS) {
        Serial.println("CONFIG_PIN long press (>5s) released -> RESTARTING ESP32");
        delay(100);
        ESP.restart();
        // Not reached
      } else {
        Serial.println("CONFIG_PIN short press released -> Entering config mode");
        configPinPressStart = 0;
        configPinLongPressTriggered = false;
        enterConfigMode();
        return;
      }
    }
    // Ensure clean state
    configPinPressStart = 0;
    configPinLongPressTriggered = false;
  }

  // ---- Config mode: only handle web clients ----
  if (configMode) {
    server.handleClient();
    delay(1); // yield to idle task
    return;
  }

  // ---- Normal operation ----
  esp_task_wdt_reset();

  monitorPin(S1, S1_State, startS1, lastChangeTime_S1, "S1", deviceID, serialNo, "P1");
  monitorPin(S2, S2_State, startS2, lastChangeTime_S2, "S2", deviceID, serialNo, "P1");
  monitorPin(S3, S3_State, startS3, lastChangeTime_S3, "S3", deviceID, serialNo, "P1");
  monitorPin(S4, S4_State, startS4, lastChangeTime_S4, "S4", deviceID, serialNo, "P2");
  monitorPin(S5, S5_State, startS5, lastChangeTime_S5, "S5", deviceID, serialNo, "P2");
  monitorPin(S6, S6_State, startS6, lastChangeTime_S6, "S6", deviceID, serialNo, "P2");

  if (millis() - lastHeartbeat >= cfg_HEART_INTERVAL) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  if (isSystemActive()) {
    noSignalStartTime = millis();
  } else {
    if (millis() - noSignalStartTime >= cfg_NO_SIGNAL_TIMEOUT) {
      Serial.println("No activity for timeout -> Restarting...");
      delay(100);
      ESP.restart();
    }
  }

  processStored();
  checkAck();
}