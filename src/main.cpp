#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#ifdef BOARD_TYPE_1
#include <RTClib.h>
#elif defined(BOARD_TYPE_2)
#include <time.h>
#endif
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AHTxx.h>

#ifdef BOARD_TYPE_1
#define SOIL_PIN GPIO_NUM_0     // ADC
#define SOLENOID_PIN GPIO_NUM_3 // Relay
#elif defined(BOARD_TYPE_2)
#define SOIL_PIN GPIO_NUM_12     // ADC
#define SOLENOID_PIN GPIO_NUM_13 // Relay
#endif

#ifdef BOARD_TYPE_1
AHTxx aht25(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
RTC_DS3231 rtc;
#endif

BLEServer *pServer = nullptr;
BLECharacteristic *pCharSensor;
BLECharacteristic *pCharControl;
BLECharacteristic *pCharSchedule;
BLECharacteristic *pCharLogs;
BLECharacteristic *pCharRTC;

const char *deviceName = "SmartPlant-ESP32";

const uint64_t SENSOR_NOTIFY_INTERVAL_MS = 5000; // 30s
uint64_t lastSensorMillis = 0;

// Logging
const char *LOG_PATH = "/logs.txt";
const size_t LOG_BUFFER_LIMIT = 512 * 1024; // 512KB rotate threshold

// Schedule struct
struct ScheduleSlot
{
  bool enabled;
  uint8_t hour;
  uint8_t minute;
  uint16_t duration_seconds;
  uint32_t last_run_day; // day counter to avoid duplicate runs
};

#define MAX_SLOTS 4
ScheduleSlot slots[MAX_SLOTS];

// For running solenoid
volatile bool solenoidRunning = false;
uint32_t solenoidStopAt = 0;
bool notifyEnabled = false;
const int MTU_SIZE = 75;
const int timezone = 7;

// utility: get epoch from RTC
#ifdef BOARD_TYPE_1
uint32_t rtc_epoch()
{
  DateTime now = rtc.now();
  return (uint32_t)now.unixtime();
}
#elif defined(BOARD_TYPE_2)
uint32_t rtc_epoch()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return 0;
  }
  return (uint32_t)mktime(&timeinfo);
}
#endif

#ifdef BOARD_TYPE_2
void setupTime()
{
struct tm tm;
  tm.tm_year = 2025 - 1900;  // years since 1900
  tm.tm_mon  = 9;            // month 0–11 (October = 9)
  tm.tm_mday = 23;           // day of month
  tm.tm_hour = 12;
  tm.tm_min  = 0;
  tm.tm_sec  = 0;
  time_t t = mktime(&tm);

  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);  // set system timeu
}
#endif

// helper to append log (JSON line)
void appendLog(JsonDocument &doc)
{
  File f = LittleFS.open(LOG_PATH, "a");
  if (!f)
    return;
  String out;
  serializeJson(doc, out);
  out += "\n";
  f.print(out);
  f.close();

  // rotate if file too big
  File check = LittleFS.open(LOG_PATH, "r");
  size_t sz = check.size();
  check.close();
  if (sz > LOG_BUFFER_LIMIT)
  {
    LittleFS.remove("/logs.old");
    LittleFS.rename(LOG_PATH, "/logs.old");
    // new logs.txt will be created next append
  }
}

String readSensorJson()
{
  float humidity = NAN, temp = NAN, soilRaw = NAN, battery = NAN;
  #ifdef BOARD_TYPE_1
  humidity = aht25.readHumidity();
  temp = aht25.readTemperature();
  soilRaw = analogRead(SOIL_PIN);        // raw 0-4095
  #elif defined(BOARD_TYPE_2)
  humidity = random(4000, 6000) / 100.0; // simulate
  temp = random(2000, 3000) / 100.0;     // simulate
  soilRaw = random(0, 4096);              // simulate
  #endif

  float soil = soilRaw / 4095.0 * 100.0; // percentage


  uint32_t ts = rtc_epoch();

  // Use smaller JSON document to reduce data size
  StaticJsonDocument<128> doc;
  doc["t"] = ts;
  if (!isnan(soilRaw))
    doc["s"] = (int)(soil * 100) / 100.0; // Round to 2 decimal places
  if (!isnan(humidity))
    doc["h"] = (int)(humidity * 100) / 100.0; // Round to 2 decimal places
  if (!isnan(temp))
    doc["tmp"] = (int)(temp * 100) / 100.0; // Round to 2 decimal places

  String out;
  serializeJson(doc, out);
  Serial.printf("Generated JSON: %s (length: %d)\n", out.c_str(), out.length());
  return out;
}

// BLE write callbacks
class ControlCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar)
  {
    Serial.println("ControlCallbacks::onWrite triggered");
    String val = pChar->getValue().c_str();
    Serial.printf("Received data size: %d\n", val.length());
    if (val.length() == 0)
      return;

    Serial.printf("Received data: %s\n", val.c_str());

    // parse JSON
    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, val);
    if (!err)
    {
      const char *cmd = doc["cmd"];
      Serial.printf("Parsed command: %s\n", cmd ? cmd : "null");
      if (cmd && strcmp(cmd, "trigger") == 0)
      {
        int dur = doc["duration"] | 10;
        // start solenoid for dur seconds
        Serial.printf("Trigger solenoid for %d seconds\n", dur);
        digitalWrite(SOLENOID_PIN, HIGH);
        solenoidRunning = true;
        solenoidStopAt = millis() + uint32_t(dur) * 1000;
      }
      else if (cmd && strcmp(cmd, "cancel") == 0)
      {
        Serial.println("Cancel solenoid");
        digitalWrite(SOLENOID_PIN, LOW);
        solenoidRunning = false;
      }
    }
    else
    {
      Serial.printf("JSON parse error: %s\n", err.c_str());
    }
  }
};

class ScheduleCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar)
  {
    Serial.println("ScheduleCallbacks::onWrite triggered");
    String val = pChar->getValue().c_str();
    Serial.printf("Schedule data size: %d\n", val.length());
    if (val.length() == 0)
      return;

    Serial.printf("Schedule data: %s\n", val.c_str());

    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, val);
    if (err)
    {
      Serial.printf("Schedule JSON parse error: %s\n", err.c_str());
      return;
    }

    // Expect {"slot":1,"time":"07:30","duration":15,"enabled":1}
    int slot = doc["slot"] | -1;
    if (slot < 0 || slot >= MAX_SLOTS)
    {
      Serial.printf("Invalid slot: %d\n", slot);
      return;
    }

    const char *timestr = doc["time"];
    int dur = doc["duration"] | -1; // Use -1 to detect if duration was provided
    int enabled = doc["enabled"] | 0;

    Serial.printf("Parsed - slot:%d, time:%s, duration:%d, enabled:%d\n",
                  slot, timestr ? timestr : "null", dur, enabled);

    // Always update enabled status
    slots[slot].enabled = enabled;

    // Only update time and duration if they are provided
    if (timestr)
    {
      int hh = 0, mm = 0;
      sscanf(timestr, "%d:%d", &hh, &mm);
      slots[slot].hour = hh;
      slots[slot].minute = mm;
    }

    if (dur >= 0) // Only update duration if it was explicitly provided
    {
      slots[slot].duration_seconds = dur;
    }

    Serial.printf("Updated slot %d: %02d:%02d, %ds, %s\n",
                  slot, slots[slot].hour, slots[slot].minute, slots[slot].duration_seconds, enabled ? "enabled" : "disabled");

    // save schedules
    File f = LittleFS.open("/schedules.json", "w");
    if (f)
    {
      DynamicJsonDocument root(512);
      JsonArray scheduleArray = root.to<JsonArray>();
      for (int i = 0; i < MAX_SLOTS; i++)
      {
        JsonObject so = scheduleArray.createNestedObject();
        so["enabled"] = slots[i].enabled;
        so["hour"] = slots[i].hour;
        so["minute"] = slots[i].minute;
        so["duration"] = slots[i].duration_seconds;
      }
      serializeJson(root, f);
      String out;
      serializeJson(root, out);
      Serial.printf("Saving schedules JSON: %s\n", out.c_str());

      f.close();
      Serial.println("Schedules saved to file");
    }
    else
    {
      Serial.println("Failed to save schedules");
    }
  }
  void onRead(BLECharacteristic *pChar)
  {
    Serial.println("ScheduleCallbacks::onRead triggered");
    // send current schedules as JSON array
    DynamicJsonDocument root(512);
    JsonArray scheduleArray = root.to<JsonArray>();

    for (int i = 0; i < MAX_SLOTS; i++)
    {
      JsonObject so = scheduleArray.createNestedObject();
      so["slot"] = i;
      so["enabled"] = slots[i].enabled;
      char timebuf[6];
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d", slots[i].hour, slots[i].minute);
      so["time"] = timebuf;
      so["duration"] = slots[i].duration_seconds;
    }
    String out;
    serializeJson(root, out);
    pChar->setValue(out.c_str());
    Serial.printf("Sent schedules JSON: %s\n", out.c_str());
  }
};

class RTCWriteCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar)
  {
    Serial.println("RTCWriteCallbacks::onWrite triggered");
    String val = pChar->getValue().c_str();
    Serial.printf("RTC data size: %d\n", val.length());
    if (val.length() == 0)
      return;

    Serial.printf("RTC data: %s\n", val.c_str());

    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, val);
    if (err)
    {
      Serial.printf("RTC JSON parse error: %s\n", err.c_str());
      return;
    }

    // expect {"epoch":1700000000}
    uint32_t e = doc["epoch"] | 0;
    Serial.printf("Parsed epoch: %u\n", e);
    if (e > 1000000000)
    {
      #ifdef BOARD_TYPE_1
      rtc.adjust(DateTime(e));
      #elif defined(BOARD_TYPE_2)
      struct timeval timeinfo;
      timeinfo.tv_sec = e;
      settimeofday(&timeinfo, nullptr);
      #endif
      Serial.println("RTC time updated");
    }
    else
    {
      Serial.println("Invalid epoch value");
    }
  }
};

class LogsCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar)
  {
    Serial.println("LogsCallbacks::onWrite triggered");
    String val = pChar->getValue().c_str();
    Serial.printf("Logs data size: %d\n", val.length());
    if (val.length() == 0)
      return;

    Serial.printf("Logs data: %s\n", val.c_str());

    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, val);
    if (err)
    {
      Serial.printf("Logs JSON parse error: %s\n", err.c_str());
      return;
    }

    const char *action = doc["action"];
    Serial.printf("Logs action: %s\n", action ? action : "null");

    if (action && strcmp(action, "get") == 0)
    {
      uint32_t start = doc["start"] | 0;
      Serial.printf("Starting log stream from line %u\n", start);

      // stream logs starting from start line index
      File f = LittleFS.open(LOG_PATH, "r");
      if (!f)
      {
        Serial.println("Failed to open log file");
        return;
      }

      // naive: read lines and notify from start index
      uint32_t idx = 0;
      uint32_t sent = 0;
      while (f.available())
      {
        String line = f.readStringUntil('\n');
        if (idx >= start)
        {
          // send line by notifications; chunk to MTU if needed
          pChar->setValue(line.c_str());
          pChar->notify();
          sent++;
          delay(20); // small delay to avoid BLE buffer overflow
        }
        idx++;
      }
      f.close();
      Serial.printf("Sent %u log lines\n", sent);
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Client connected");
    // Use conservative default for compatibility
  }

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("Client disconnected");
    notifyEnabled = false;
    // Restart advertising
    BLEDevice::startAdvertising();
  }
};

// Descriptor callback for notifications
class DescriptorCallbacks : public BLEDescriptorCallbacks
{
  void onWrite(BLEDescriptor *pDescriptor)
  {
    uint8_t *data = pDescriptor->getValue();
    if (data != nullptr)
    {
      Serial.printf("Descriptor written: ");
      for (int i = 0; i < pDescriptor->getLength(); i++)
      {
        Serial.printf("%02X ", data[i]);
      }
      Serial.println();

      // Check if notifications are enabled (0x0001) or disabled (0x0000)
      if (pDescriptor->getLength() >= 2)
      {
        uint16_t value = (data[1] << 8) | data[0]; // Little endian
        if (value == 0x0001)
        {
          notifyEnabled = true;
          Serial.println("Notifications ENABLED");
        }
        else if (value == 0x0000)
        {
          notifyEnabled = false;
          Serial.println("Notifications DISABLED");
        }
      }
    }
  }
};

class SensorCallbacks : public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pChar)
  {
    Serial.println("Sensor data read requested");
    String json = readSensorJson();
    pChar->setValue(json.c_str());
  }

  void onWrite(BLECharacteristic *pChar)
  {
    Serial.println("Sensor characteristic written to");
    // This usually won't be called for notifications, descriptor callback handles it
  }
};

void startBLE()
{
  Serial.println("Initializing BLE...");

  BLEDevice::init(deviceName);
  Serial.println("BLE device initialized");

  pServer = BLEDevice::createServer();
  Serial.println("BLE server created");
  BLEDevice::setMTU(MTU_SIZE);

  pServer->setCallbacks(new ServerCallbacks());
  Serial.println("Server callbacks set");

  BLEService *svc = pServer->createService("0000a000-0000-1000-8000-00805f9b34fb");
  Serial.println("BLE service created");

  pCharSensor = svc->createCharacteristic("0000a001-0000-1000-8000-00805f9b34fb",
                                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("Sensor characteristic created");

  pCharControl = svc->createCharacteristic("0000a002-0000-1000-8000-00805f9b34fb",
                                           BLECharacteristic::PROPERTY_WRITE);
  Serial.println("Control characteristic created");

  pCharSchedule = svc->createCharacteristic("0000a003-0000-1000-8000-00805f9b34fb",
                                            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  Serial.println("Schedule characteristic created");

  pCharLogs = svc->createCharacteristic("0000a004-0000-1000-8000-00805f9b34fb",
                                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("Logs characteristic created");

  pCharRTC = svc->createCharacteristic("0000a005-0000-1000-8000-00805f9b34fb",
                                       BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  Serial.println("RTC characteristic created");

  // Add descriptors for notify characteristics with callbacks
  BLE2902 *pSensorDescriptor = new BLE2902();
  pSensorDescriptor->setCallbacks(new DescriptorCallbacks());
  pCharSensor->addDescriptor(pSensorDescriptor);

  BLE2902 *pLogsDescriptor = new BLE2902();
  pLogsDescriptor->setCallbacks(new DescriptorCallbacks());
  pCharLogs->addDescriptor(pLogsDescriptor);

  // Set callbacks
  pCharControl->setCallbacks(new ControlCallbacks());
  Serial.println("Control callbacks set");

  pCharSchedule->setCallbacks(new ScheduleCallbacks());
  Serial.println("Schedule callbacks set");

  pCharLogs->setCallbacks(new LogsCallbacks());
  Serial.println("Logs callbacks set");

  pCharRTC->setCallbacks(new RTCWriteCallbacks());
  Serial.println("RTC callbacks set");

  pCharSensor->setCallbacks(new SensorCallbacks());
  Serial.println("Sensor callbacks set");

  svc->start();
  Serial.println("BLE service started");

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID("0000a000-0000-1000-8000-00805f9b34fb");
  pAdv->setScanResponse(false);
  pAdv->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
  Serial.printf("Device name: %s\n", deviceName);
  Serial.println("BLE initialization complete");
}

void loadSchedules()
{
  if (LittleFS.exists("/schedules.json"))
  {
    Serial.println("Loading schedules from /schedules.json");
    File f = LittleFS.open("/schedules.json", "r");
    if (!f)
    {
      Serial.println("Failed to open schedules file");
      return;
    }
    DynamicJsonDocument root(512);
    deserializeJson(root, f);
    JsonArray scheduleArray = root.as<JsonArray>();
    Serial.println(root.as<String>());
    for (int i = 0; i < MAX_SLOTS; i++)
    {
      JsonObject so = scheduleArray[i];
      slots[i].enabled = so["enabled"] | false;
      slots[i].hour = so["hour"] | 0;
      slots[i].minute = so["minute"] | 0;
      slots[i].duration_seconds = so["duration"] | 0;
      slots[i].last_run_day = 0;
    }
    f.close();
  }
  else
  {
    // init defaults
    for (int i = 0; i < MAX_SLOTS; i++)
    {
      slots[i].enabled = false;
      slots[i].hour = 0;
      slots[i].minute = 0;
      slots[i].duration_seconds = 0;
      slots[i].last_run_day = 0;
    }
  }
}

void setup()
{
  pinMode(SOLENOID_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting...");

  if (!LittleFS.begin(true))
  { // true = format automatically if mount fails
    Serial.println("LittleFS mount failed");
    while (true)
      delay(100);
  }
  Serial.println("LittleFS mounted successfully");

  Wire.begin(SDA, SCL);
  #ifdef BOARD_TYPE_1
  if (!rtc.begin())
  {
    Serial.println("RTC not found!");
  }
  aht25.begin(SDA, SCL);
  #elif defined(BOARD_TYPE_2)
  setupTime();
  #endif

  loadSchedules();
  startBLE();
  lastSensorMillis = millis();
}

void loop()
{
  uint32_t nowMs = millis();

  // periodic sensor read & notify or log
  if (nowMs - lastSensorMillis >= SENSOR_NOTIFY_INTERVAL_MS)
  {
    lastSensorMillis = nowMs;
    String json = readSensorJson();
    Serial.println("Sensor: " + json);

    // if client subscribed -> notify
    if (notifyEnabled && pServer->getConnectedCount() > 0)
    {
      Serial.printf("JSON data length: %d bytes\n", json.length());
      Serial.printf("Max notify size: %d bytes\n", MTU_SIZE - 3);

      // Data fits in single notification
      pCharSensor->setValue(json.c_str());
      pCharSensor->notify();
      Serial.printf("Sensor notification sent: %s\n", json.c_str());
    }
    else if (!notifyEnabled && pServer->getConnectedCount() > 0)
    {
      Serial.println("Client connected but notifications not enabled");
    }
    else if (notifyEnabled && pServer->getConnectedCount() == 0)
    {
      Serial.println("Notifications enabled but no client connected");
    }
    else
    {
      // save to log
      DynamicJsonDocument doc(256);
      deserializeJson(doc, json);
      appendLog(doc);
    }
  }

  // check solenoid stop
  if (solenoidRunning && millis() >= solenoidStopAt)
  {
    digitalWrite(SOLENOID_PIN, LOW);
    solenoidRunning = false;
  }

  // schedule check (run every 20s)
  static uint32_t lastSchedCheck = 0;
  if (millis() - lastSchedCheck > 20000)
  {
    Serial.println("Checking schedules...");
    lastSchedCheck = millis();
    #ifdef BOARD_TYPE_1
    DateTime utc = rtc.now();
    DateTime now = utc + TimeSpan(timezone * 3600);
    Serial.printf("Current time: %02d:%02d\n", now.hour(), now.minute());
    uint32_t dayCounter = now.day() + now.month() * 100 + now.year() * 10000; // simple unique per day value

    for (int i = 0; i < MAX_SLOTS; i++)
    {
      Serial.printf("Slot %d: enabled=%d, time=%02d:%02d, duration=%ds, last_run_day=%u\n",
                    i, slots[i].enabled, slots[i].hour, slots[i].minute,
                    slots[i].duration_seconds, slots[i].last_run_day);
      if (!slots[i].enabled)
        continue;
      if (slots[i].hour == now.hour() && slots[i].minute == now.minute())
      {
        if (slots[i].last_run_day != dayCounter)
        {
          // start valve
          digitalWrite(SOLENOID_PIN, HIGH);
          solenoidRunning = true;
          solenoidStopAt = millis() + uint32_t(slots[i].duration_seconds) * 1000;
          slots[i].last_run_day = dayCounter;
          // log event
          DynamicJsonDocument doc(256);
          doc["t"] = rtc_epoch();
          doc["event"] = "schedule_run";
          doc["slot"] = i;
          appendLog(doc);
        }
      }
    }
    #elif defined(BOARD_TYPE_2)
    for (int i = 0; i < MAX_SLOTS; i++)
    {
      Serial.printf("Slot %d: enabled=%d, time=%02d:%02d, duration=%ds, last_run_day=%u\n",
                    i, slots[i].enabled, slots[i].hour, slots[i].minute,
                    slots[i].duration_seconds, slots[i].last_run_day);
    }
    #endif
  }

  delay(20);
}