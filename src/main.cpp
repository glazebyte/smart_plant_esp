#include <Arduino.h>

#include <NimBLEDevice.h>
#include <Wire.h>
#include <RTClib.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AHTxx.h>

#define SOIL_PIN GPIO_NUM_0     // ADC
#define SOLENOID_PIN 27 // Relay

AHTxx aht25(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
RTC_DS3231 rtc;

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pCharSensor;
NimBLECharacteristic *pCharControl;
NimBLECharacteristic *pCharSchedule;
NimBLECharacteristic *pCharLogs;
NimBLECharacteristic *pCharRTC;

const char *deviceName = "SmartPlant-ESP32";

const uint64_t SENSOR_NOTIFY_INTERVAL_MS = 30000; // 30s
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

// utility: get epoch from RTC
uint32_t rtc_epoch()
{
  DateTime now = rtc.now();
  return (uint32_t)now.unixtime();
}

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
  humidity = aht25.readHumidity();
  temp = aht25.readTemperature();
  soilRaw = analogRead(SOIL_PIN); // raw 0-4095
  // assume divider and convert to volts; adjust dividerFactor
  const float dividerFactor = 2.0; // example

  uint32_t ts = rtc_epoch();

  StaticJsonDocument<256> doc;
  doc["t"] = ts;
  if (!isnan(soilRaw))
    doc["soil_raw"] = (int)soilRaw;
  if (!isnan(humidity))
    doc["hum"] = humidity;
  if (!isnan(temp))
    doc["temp"] = temp;

  String out;
  serializeJson(doc, out);
  return out;
}

// BLE write callbacks
class ControlCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar)
  {
    std::string val = pChar->getValue();
    if (val.size() == 0)
      return;
    // parse JSON
    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, val);
    if (!err)
    {
      const char *cmd = doc["cmd"];
      if (cmd && strcmp(cmd, "trigger") == 0)
      {
        int dur = doc["duration"] | 10;
        // start solenoid for dur seconds
        digitalWrite(SOLENOID_PIN, HIGH);
        solenoidRunning = true;
        solenoidStopAt = millis() + uint32_t(dur) * 1000;
      }
      else if (cmd && strcmp(cmd, "cancel") == 0)
      {
        digitalWrite(SOLENOID_PIN, LOW);
        solenoidRunning = false;
      }
    }
  }
};

class ScheduleCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar)
  {
    std::string val = pChar->getValue();
    if (val.size() == 0)
      return;
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, val))
      return;
    // Expect {"slot":1,"time":"07:30","duration":15,"enabled":1}
    int slot = doc["slot"] | -1;
    if (slot < 0 || slot >= MAX_SLOTS)
      return;
    const char *timestr = doc["time"];
    int dur = doc["duration"] | 0;
    int enabled = doc["enabled"] | 0;
    if (timestr)
    {
      int hh = 0, mm = 0;
      sscanf(timestr, "%d:%d", &hh, &mm);
      slots[slot].hour = hh;
      slots[slot].minute = mm;
      slots[slot].duration_seconds = dur;
      slots[slot].enabled = enabled;
      // save schedules
      File f = LittleFS.open("/schedules.json", "w");
      if (f)
      {
        DynamicJsonDocument root(512);
        for (int i = 0; i < MAX_SLOTS; i++)
        {
          JsonObject so = root[String(i)];
          so["enabled"] = slots[i].enabled;
          so["hour"] = slots[i].hour;
          so["minute"] = slots[i].minute;
          so["duration"] = slots[i].duration_seconds;
        }
        serializeJson(root, f);
        f.close();
      }
    }
  }
};

class RTCWriteCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar)
  {
    std::string val = pChar->getValue();
    if (val.size() == 0)
      return;
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, val))
    {
      // expect {"epoch":1700000000}
      uint32_t e = doc["epoch"] | 0;
      if (e > 1000000000)
      {
        rtc.adjust(DateTime(e));
      }
    }
  }
};

class LogsCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar)
  {
    std::string val = pChar->getValue();
    if (val.size() == 0)
      return;
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, val))
    {
      const char *action = doc["action"];
      if (action && strcmp(action, "get") == 0)
      {
        uint32_t start = doc["start"] | 0;
        // stream logs starting from start line index
        File f = LittleFS.open(LOG_PATH, "r");
        if (!f)
          return;
        // naive: read lines and notify from start index
        uint32_t idx = 0;
        String chunk;
        while (f.available())
        {
          String line = f.readStringUntil('\n');
          if (idx >= start)
          {
            // send line by notifications; chunk to MTU if needed
            pChar->setValue((uint8_t *)line.c_str(), line.length());
            pChar->notify();
            delay(20); // small delay to avoid BLE buffer overflow
          }
          idx++;
        }
        f.close();
      }
    }
  }
};

class CCCDCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *desc, NimBLEConnInfo &connInfo) override
  {
    std::string val = desc->getValue();
    bool notifyEnabled = (val.length() > 0 && (val[0] & 0x01));
    Serial.printf("Notifications %s\n", notifyEnabled ? "ENABLED" : "DISABLED");
  }

  void onRead(NimBLEDescriptor *desc, NimBLEConnInfo &connInfo) override
  {
    Serial.println("Descriptor read");
  }
};

void startBLE()
{
  NimBLEDevice::init(deviceName);
  NimBLEServer *pServer = NimBLEDevice::createServer();

  NimBLEService *svc = pServer->createService("0000a000-0000-1000-8000-00805f9b34fb");

  pCharSensor = svc->createCharacteristic("0000a001-0000-1000-8000-00805f9b34fb",
                                          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pCharControl = svc->createCharacteristic("0000a002-0000-1000-8000-00805f9b34fb",
                                           NIMBLE_PROPERTY::WRITE);
  pCharSchedule = svc->createCharacteristic("0000a003-0000-1000-8000-00805f9b34fb",
                                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  pCharLogs = svc->createCharacteristic("0000a004-0000-1000-8000-00805f9b34fb",
                                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharRTC = svc->createCharacteristic("0000a005-0000-1000-8000-00805f9b34fb",
                                       NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);

  pCharControl->setCallbacks(new ControlCallbacks());
  pCharSchedule->setCallbacks(new ScheduleCallbacks());
  pCharLogs->setCallbacks(new LogsCallbacks());
  pCharRTC->setCallbacks(new RTCWriteCallbacks());

  NimBLEDescriptor *pCCCD = pCharSensor->createDescriptor(
      "2902",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE,
      2);
  
  pCCCD->setCallbacks(new CCCDCallbacks());

  svc->start();

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(svc->getUUID());
  pAdv->start();
  Serial.println("BLE started, advertising.");
}

void loadSchedules()
{
  if (LittleFS.exists("/schedules.json"))
  {
    File f = LittleFS.open("/schedules.json", "r");
    if (!f)
      return;
    DynamicJsonDocument root(512);
    deserializeJson(root, f);
    for (int i = 0; i < MAX_SLOTS; i++)
    {
      JsonObject so = root[String(i)];
      slots[i].enabled = so["enabled"] | 0;
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
  Serial.begin(115200);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  LittleFS.begin();

  Wire.begin();
  if (!rtc.begin())
  {
    Serial.println("RTC not found!");
  }
  aht25.begin();

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

    // if client subscribed -> notify
    if (notifyEnabled)
    {
      pCharSensor->setValue((uint8_t *)json.c_str(), json.length());
      pCharSensor->notify();
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
    lastSchedCheck = millis();
    DateTime now = rtc.now();
    uint32_t dayCounter = now.day() + now.month() * 100 + now.year() * 10000; // simple unique per day value

    for (int i = 0; i < MAX_SLOTS; i++)
    {
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
  }

  delay(20);
}