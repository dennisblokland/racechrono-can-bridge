#include <ESP32CAN.h>
#include <RaceChrono.h>
#include <Arduino.h>
#include <CAN_config.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>

#include "config.h"

// Check if WiFi configuration exists
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#define WIFI_CONFIGURED
#else
#warning "WiFi configuration not found. Copy wifi_config.h.template to wifi_config.h and set your WiFi credentials for OTA functionality."
#endif
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;

bool isCanBusReaderActive = false;
long lastCanMessageReceivedMs;

struct PidExtra
{
  // Only send one out of |updateRateDivider| packets per PID.
  uint8_t updateRateDivider = DEFAULT_UPDATE_RATE_DIVIDER;

  // Varies between 0 and |updateRateDivider - 1|.
  uint8_t skippedUpdates = 0;
};
RaceChronoPidMap<PidExtra> pidMap;

uint32_t loop_iteration = 0;

uint32_t last_time_num_can_bus_timeouts_sent_ms = 0;
uint16_t num_can_bus_timeouts = 0;

#ifdef WIFI_CONFIGURED
WebServer server(OTA_PORT);
#endif

// Forward declarations to help put code in a natural reading order.
void waitForConnection();
void bufferNewPacket(uint32_t pid, uint8_t *data, uint8_t data_length);
void handleOneBufferedPacket();
void flushBufferedPackets();
void sendNumCanBusTimeouts();
void resetSkippedUpdatesCounters();
#ifdef WIFI_CONFIGURED
void setupWiFi();
void setupOTA();
#endif

void dumpMapToSerial()
{
  Serial.println("Current state of the PID map:");

  uint16_t updateIntervalForAllEntries;
  bool areAllPidsAllowed =
      pidMap.areAllPidsAllowed(&updateIntervalForAllEntries);
  if (areAllPidsAllowed)
  {
    Serial.println("  All PIDs are allowed.");
  }

  if (pidMap.isEmpty())
  {
    if (areAllPidsAllowed)
    {
      Serial.println("  Map is empty.");
      Serial.println("");
    }
    else
    {
      Serial.println("  No PIDs are allowed.");
      Serial.println("");
    }
    return;
  }

  struct
  {
    void operator()(void *entry)
    {
      uint32_t pid = pidMap.getPid(entry);
      const PidExtra *extra = pidMap.getExtra(entry);

      Serial.print("  ");
      Serial.print(pid);
      Serial.print(" (0x");
      Serial.print(pid, HEX);
      Serial.print("), sending 1 out of ");
      Serial.print(extra->updateRateDivider);
      Serial.println(" messages.");
    }
  } dumpEntry;
  pidMap.forEach(dumpEntry);

  Serial.println("");
}

class UpdateMapOnRaceChronoCommands : public RaceChronoBleCanHandler
{
public:
  void allowAllPids(uint16_t updateIntervalMs)
  {
    Serial.print("Command: ALLOW ALL PIDS, update interval: ");
    Serial.print(updateIntervalMs);
    Serial.println(" ms.");

    pidMap.allowAllPids(updateIntervalMs);

    dumpMapToSerial();
  }

  void denyAllPids()
  {
    Serial.println("Command: DENY ALL PIDS.");

    pidMap.reset();

    dumpMapToSerial();
  }

  void allowPid(uint32_t pid, uint16_t updateIntervalMs)
  {
    Serial.print("Command: ALLOW PID ");
    Serial.print(pid);
    Serial.print(" (0x");
    Serial.print(pid, HEX);
    Serial.print("), requested update interval: ");
    Serial.print(updateIntervalMs);
    Serial.println(" ms.");

    if (!pidMap.allowOnePid(pid, updateIntervalMs))
    {
      Serial.println("WARNING: unable to handle this request!");
    }

    void *entry = pidMap.getEntryId(pid);
    if (entry != nullptr)
    {
      PidExtra *pidExtra = pidMap.getExtra(entry);
      pidExtra->skippedUpdates = 0;
      pidExtra->updateRateDivider = getUpdateRateDivider(pid);
    }

    dumpMapToSerial();
  }

  void handleDisconnect()
  {
    Serial.println("Resetting the map.");

    pidMap.reset();

    dumpMapToSerial();
  }
} raceChronoHandler;

void setup()
{
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);
  pinMode(CAN_SE_PIN, OUTPUT);
  digitalWrite(CAN_SE_PIN, LOW);

  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  uint32_t startTimeMs = millis();
  Serial.begin(115200);
  while (!Serial && millis() - startTimeMs < 5000)
  {
  }

  Serial.println("ESP32 RaceChrono CAN Bridge starting...");

#ifdef WIFI_CONFIGURED
  setupWiFi();
  setupOTA();
#else
  Serial.println("WiFi not configured. OTA updates disabled.");
  Serial.println("To enable OTA: copy wifi_config.h.template to wifi_config.h and set your WiFi credentials.");
#endif

  Serial.println("Setting up BLE...");
  RaceChronoBle.setUp(DEVICE_NAME, &raceChronoHandler);
  RaceChronoBle.startAdvertising();

  Serial.println("BLE is set up, waiting for an incoming connection.");
  waitForConnection();
}

void waitForConnection()
{
  uint32_t iteration = 0;
  bool lastPrintHadNewline = false;
  while (!RaceChronoBle.waitForConnection(1000))
  {
    Serial.print(".");
    if ((++iteration) % 10 == 0)
    {
      lastPrintHadNewline = true;
      Serial.println();
    }
    else
    {
      lastPrintHadNewline = false;
    }
  }

  if (!lastPrintHadNewline)
  {
    Serial.println();
  }

  Serial.println("Connected.");
}

void loop()
{
  loop_iteration++;

#ifdef WIFI_CONFIGURED
  // Handle OTA updates
  server.handleClient();
#endif

  // First, verify that we have both Bluetooth and CAN up and running.

  // Not clear how heavy is the isConnected() call. Only check the connectivity
  // every 100 iterations to avoid stalling the CAN bus loop.
  if ((loop_iteration % 100) == 0 && !RaceChronoBle.isConnected())
  {
    Serial.println("RaceChrono disconnected!");
    raceChronoHandler.handleDisconnect();

    Serial.println("Waiting for a new connection.");
    waitForConnection();
    sendNumCanBusTimeouts();
  }

  uint32_t timeNowMs = millis();

  // Sending data over Bluetooth takes time, and MCP2515 only has two buffers
  // for received messages. Once a buffer is full, further messages are dropped.
  // Instead of relying on the MCP2515 buffering, we aggressively read messages
  // from the MCP2515 and put them into our memory.
  //
  // TODO: It might be more efficient to use interrupts to read data from
  // MCP2515 as soon as it's available instead of polling all the time. The
  // interrupts don't work out of the box with nRF52 Adafruit boards though, and
  // need to be handled carefully wrt synchronization between the interrupt
  // handling and processing of received data.

  CAN_frame_t rx_frame;
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  {
    uint32_t pid = rx_frame.MsgID;
    int data_length = rx_frame.FIR.B.DLC;
 

    bufferNewPacket(pid, rx_frame.data.u8, data_length);
    lastCanMessageReceivedMs = millis();
  }

  handleOneBufferedPacket();

  if (millis() - last_time_num_can_bus_timeouts_sent_ms > 2000)
  {
    sendNumCanBusTimeouts();
  }
}

struct BufferedMessage
{
  uint32_t pid;
  uint8_t data[8];
  uint8_t length;
};

// Circular buffer to put received messages, used to buffer messages in memory
// (which is relatively abundant) instead of relying on the very limited
// buffering ability of the MCP2515.
const uint8_t NUM_BUFFERS = 16; // Must be a power of 2, but less than 256.
BufferedMessage buffers[NUM_BUFFERS];
uint8_t bufferToWriteTo = 0;
uint8_t bufferToReadFrom = 0;

void bufferNewPacket(uint32_t pid, uint8_t *data, uint8_t data_length)
{
  if (bufferToWriteTo - bufferToReadFrom == NUM_BUFFERS)
  {
    Serial.println("WARNING: Receive buffer overflow, dropping one message.");

    // In case of a buffer overflow, drop the oldest message in the buffer, as
    // it's likely less useful than the newest one.
    bufferToReadFrom++;
  }

  BufferedMessage *message = &buffers[bufferToWriteTo % NUM_BUFFERS];
  message->pid = pid;
  memcpy(message->data, data, data_length);
  message->length = data_length;
  bufferToWriteTo++;
}

void handleOneBufferedPacket()
{
  if (bufferToReadFrom == bufferToWriteTo)
  {
    // No buffered messages.
    return;
  }

  BufferedMessage *message = &buffers[bufferToReadFrom % NUM_BUFFERS];
  uint32_t pid = message->pid;
  void *entry = pidMap.getEntryId(pid);
  if (entry != nullptr)
  {
    // TODO: we could do something smart here. For example, if there are more
    // messages pending with the same PID, we could count them towards
    // |skippedUpdates| in a way that we only send the latest one, but maintain
    // roughly the desired rate.
    PidExtra *extra = pidMap.getExtra(entry);
    if (extra->skippedUpdates == 0)
    {
      RaceChronoBle.sendCanData(pid, message->data, message->length);
    }

    extra->skippedUpdates++;
    if (extra->skippedUpdates >= extra->updateRateDivider)
    {
      // The next message with this PID will be sent.
      extra->skippedUpdates = 0;
    }
  }

  bufferToReadFrom++;
}

void flushBufferedPackets()
{
  bufferToWriteTo = 0;
  bufferToReadFrom = 0;
}

void sendNumCanBusTimeouts()
{
  // Send the count of timeouts to RaceChrono in a "fake" PID=0x777 message.
  uint8_t data[2];
  data[0] = num_can_bus_timeouts & 0xff;
  data[1] = num_can_bus_timeouts >> 8;
  RaceChronoBle.sendCanData(0x777, data, 2);

  last_time_num_can_bus_timeouts_sent_ms = millis();
}

void resetSkippedUpdatesCounters()
{
  struct
  {
    void operator()(void *entry)
    {
      PidExtra *extra = pidMap.getExtra(entry);
      extra->skippedUpdates = 0;
    }
  } resetSkippedUpdatesCounter;
  pidMap.forEach(resetSkippedUpdatesCounter);
}

#ifdef WIFI_CONFIGURED
void setupWiFi()
{
  Serial.println("Setting up WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed. OTA updates will not be available.");
  }
}

void setupOTA()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Skipping OTA setup.");
    return;
  }
  
  Serial.println("Setting up OTA...");
  
  // ElegantOTA callbacks
  ElegantOTA.onStart([]() {
    Serial.println("OTA update started!");
  });
  
  ElegantOTA.onProgress([](size_t current, size_t final) {
    Serial.printf("OTA Progress: %u%%\n", (current * 100) / final);
  });
  
  ElegantOTA.onEnd([](bool success) {
    if (success) {
      Serial.println("OTA update completed successfully!");
    } else {
      Serial.println("OTA update failed!");
    }
  });
  
  ElegantOTA.begin(&server, OTA_USERNAME, OTA_PASSWORD);
  server.begin();
  
  Serial.println("OTA ready!");
  Serial.print("Open http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.print(OTA_PORT);
  Serial.println(" in your browser to upload firmware");
}
#endif