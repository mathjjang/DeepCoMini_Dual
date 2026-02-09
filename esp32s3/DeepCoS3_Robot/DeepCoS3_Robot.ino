/*
  DeepCoS3_Robot.ino (ESP32‑S3, Arduino IDE)

  목표:
  - 기존 DeepcoMini_v1.3의 "카메라/모터/LED/커맨드 파싱"을 유지
  - Wi‑Fi/웹서버는 제거 (네트워크는 RTL8720DN이 담당)
  - RTL8720DN로부터 UART(Serial1)로 들어오는 제어 문자열을 처리

  주의:
  - Serial1 RX/TX 핀은 PCB 설계에 맞게 반드시 수정하세요.
*/

#include <Arduino.h>
#include <sstream>
#include <stdarg.h>

#include <ArduinoJson.h>

// FreeRTOS (Arduino-ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -----------------------------
// Logging policy (기본 전제: PC는 RTL만 USB로 연결)
// - S3의 USB Serial 로그는 기본적으로 "사용 안 함"
// - 디버그 로그는 Serial1로 RTL에 전달해서 RTL USB 콘솔에서 확인
// - 포맷: "@log,<text>\n"
// -----------------------------
static constexpr bool ENABLE_USB_SERIAL_DEBUG = false;

static void linkLogf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
#if 1
  // PC가 S3를 직접 USB로 연결해 디버깅할 때만 사용
  if (ENABLE_USB_SERIAL_DEBUG) {
    Serial.print("[S3] ");
    Serial.println(buf);
  }
#endif
  Serial1.print("@log,");
  Serial1.println(buf);
}

// -----------------------------
// Task split (v1.3 스타일로 분리)
// - taskSpiCamera : SPI slave 서비스(카메라 프레임 송출)
// - taskRobotCtrl : UART(Serial1 RX) 커맨드 파싱/로봇 제어
//
// NOTE:
// - PC는 RTL만 USB로 연결 → 로그는 Serial1(TX)로만 전달
// - Serial1은 RX(제어) / TX(로그) 동시 사용(풀듀플렉스)
// -----------------------------
static constexpr bool ENABLE_TASK_SPLIT = true;
static TaskHandle_t g_taskSpiCamera = nullptr;
static TaskHandle_t g_taskRobotCtrl = nullptr;
static TaskHandle_t g_taskLed = nullptr;

static void taskSpiCamera(void* arg);
static void taskRobotCtrl(void* arg);
static void taskLed(void* arg);

static volatile bool g_cameraInitOk = true;
static volatile bool g_spiInitOk = true;
static uint32_t g_bootStartMs = 0;

// -----------------------------
// LED state machine (ported from DeepcoMini_v1.3)
// Inputs are provided by RTL over Serial1:
// - "@ws,0/1"  : control WebSocket (/ws) connected
// - "@sta,N"   : number of stations associated to RTL AP (best-effort)
// -----------------------------
enum class LedStatus : uint8_t { BOOTING = 0, READY = 1, WIFI_CONNECTED = 2, CONNECTED = 3 };
static volatile bool g_wsConnected = false;   // (/ws)
static volatile uint16_t g_apStaCount = 0;    // station count (RTL AP)
static LedStatus g_ledStatus = LedStatus::BOOTING;

struct LedFlash {
  uint32_t color = 0;
  uint8_t blinksRemaining = 0;
  bool on = false;
  uint16_t onMs = 0;
  uint16_t offMs = 0;
  uint32_t untilMs = 0;
};
static LedFlash g_sideFlash;

static inline void updateLedStatusFromLinks();
static void startSideFlash(uint32_t color, uint8_t blinks, uint16_t onMs, uint16_t offMs);
static void ledUpdateOnce();

// SPI Slave (ESP-IDF)
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"

// Camera
#include "esp_camera.h"
#include "sensor.h"

// LEDs (same as v1.3)
#include <Adafruit_NeoPixel.h>

// Motors
#include <FastAccelStepper.h>

// -----------------------------
// UART link (RTL <-> S3)
// -----------------------------
static constexpr uint32_t LINK_BAUD = 115200;
// ESP32‑S3는 IO‑Matrix로 UART 핀을 자유롭게 지정 가능.
// 아래 핀은 현재 프로젝트에서 "기본값"으로 고정(보드 설계에서 그대로 배선 권장).
static constexpr int LINK_RX_PIN = 44;
static constexpr int LINK_TX_PIN = 43;

// -----------------------------
// SPI link (Camera -> RTL)
// ESP32-S3 = SPI Slave
// -----------------------------
// ESP32‑S3는 SPI Slave도 IO‑Matrix로 핀 지정 가능.
// 카메라/모터/LED에서 사용 중인 GPIO를 피해서 기본값을 잡아둠.
static constexpr int SPI_SCLK_PIN = 36;
static constexpr int SPI_MOSI_PIN = 37; // Master->Slave (RTL MOSI)
static constexpr int SPI_MISO_PIN = 35; // Slave->Master (S3 MISO)
static constexpr int SPI_CS_PIN   = 34; // Slave Select

static constexpr size_t SPI_BLOCK_BYTES = 2048;
static constexpr int SPI_QUEUE_SIZE = 2;

#pragma pack(push, 1)
struct DcmSpiHdr {
  char magic[4];        // "DCM2"
  uint8_t type;         // 0=idle, 1=jpeg
  uint8_t flags;        // bit0=start, bit1=end
  uint16_t seq;         // frame seq
  uint32_t total_len;   // JPEG total length
  uint32_t offset;      // offset of this payload
  uint16_t payload_len; // payload bytes in this block
  uint16_t reserved;
};
#pragma pack(pop)

static constexpr uint8_t SPI_TYPE_IDLE = 0;
static constexpr uint8_t SPI_TYPE_JPEG = 1;
static constexpr uint8_t SPI_FLAG_START = 0x01;
static constexpr uint8_t SPI_FLAG_END   = 0x02;

static uint8_t* g_spiTx[SPI_QUEUE_SIZE] = {nullptr, nullptr};
static uint8_t* g_spiRx[SPI_QUEUE_SIZE] = {nullptr, nullptr};
static spi_slave_transaction_t g_trans[SPI_QUEUE_SIZE];

static volatile uint32_t g_lastSpiActivityMs = 0;

// 설계 방침(사용자 요청):
// - RTL이 SPI Master로 "항상" 프레임을 pull 한다.
// - WS 연결 유무와 무관하게 S3는 SPI 트랜잭션이 계속 들어오면 프레임을 공급한다.
static bool g_camEnabled = true;
static uint16_t g_frameSeq = 1;
static camera_fb_t* g_fb = nullptr;
static size_t g_fbOff = 0;

static void fillTxBlock(uint8_t* buf) {
  if (!buf) return;
  memset(buf, 0, SPI_BLOCK_BYTES);

  DcmSpiHdr* h = (DcmSpiHdr*)buf;
  h->magic[0] = 'D'; h->magic[1] = 'C'; h->magic[2] = 'M'; h->magic[3] = '2';
  h->type = SPI_TYPE_IDLE;
  h->flags = 0;
  h->seq = 0;
  h->total_len = 0;
  h->offset = 0;
  h->payload_len = 0;
  h->reserved = 0;

  if (!g_camEnabled) {
    return;
  }

  // 프레임 없으면 새로 캡처 (SPI 클럭이 실제로 돌 때만)
  const uint32_t now = millis();
  if (!g_fb) {
    // SPI activity 없으면 캡처하지 않음(불필요한 카메라 부하 감소)
    if (now - g_lastSpiActivityMs > 500) return;
    g_fb = esp_camera_fb_get();
    if (!g_fb || !g_fb->buf || g_fb->len == 0) {
      if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
      return;
    }
    g_fbOff = 0;
    g_frameSeq++;
  }

  const size_t headerSz = sizeof(DcmSpiHdr);
  const size_t maxPayload = SPI_BLOCK_BYTES - headerSz;
  const size_t remain = (g_fb->len > g_fbOff) ? (g_fb->len - g_fbOff) : 0;
  const size_t pl = (remain > maxPayload) ? maxPayload : remain;

  h->type = SPI_TYPE_JPEG;
  h->seq = g_frameSeq;
  h->total_len = (uint32_t)g_fb->len;
  h->offset = (uint32_t)g_fbOff;
  h->payload_len = (uint16_t)pl;
  if (g_fbOff == 0) h->flags |= SPI_FLAG_START;
  if (g_fbOff + pl >= g_fb->len) h->flags |= SPI_FLAG_END;

  if (pl > 0) {
    memcpy(buf + headerSz, g_fb->buf + g_fbOff, pl);
    g_fbOff += pl;
  }

  // 마지막 청크까지 tx buffer에 복사했으면 fb는 즉시 반환 가능(이미 buf로 복사됨)
  if (g_fb && g_fbOff >= g_fb->len) {
    esp_camera_fb_return(g_fb);
    g_fb = nullptr;
    g_fbOff = 0;
  }
}

static bool spiSlaveInit() {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = SPI_MOSI_PIN;
  buscfg.miso_io_num = SPI_MISO_PIN;
  buscfg.sclk_io_num = SPI_SCLK_PIN;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = SPI_BLOCK_BYTES;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = SPI_CS_PIN;
  slvcfg.flags = 0;
  slvcfg.queue_size = SPI_QUEUE_SIZE;
  slvcfg.mode = 0;

  esp_err_t err = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    linkLogf("spi_slave_initialize failed: 0x%x", err);
    return false;
  }

  for (int i = 0; i < SPI_QUEUE_SIZE; i++) {
    g_spiTx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    g_spiRx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    if (!g_spiTx[i] || !g_spiRx[i]) {
      linkLogf("spi dma malloc failed");
      return false;
    }
    fillTxBlock(g_spiTx[i]);
    memset(&g_trans[i], 0, sizeof(g_trans[i]));
    g_trans[i].length = SPI_BLOCK_BYTES * 8;
    g_trans[i].tx_buffer = g_spiTx[i];
    g_trans[i].rx_buffer = g_spiRx[i];
    err = spi_slave_queue_trans(SPI2_HOST, &g_trans[i], portMAX_DELAY);
    if (err != ESP_OK) {
      linkLogf("spi_slave_queue_trans failed: 0x%x", err);
      return false;
    }
  }

  g_lastSpiActivityMs = millis();
  linkLogf("SPI slave initialized");
  return true;
}

static void spiSlavePoll() {
  // 고속 링크에서는 한 loop()에 트랜잭션 완료가 여러 개 쌓일 수 있어 전부 소진
  for (;;) {
    spi_slave_transaction_t* done = nullptr;
    esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, 0);
    if (err != ESP_OK || !done) break;

    g_lastSpiActivityMs = millis();

    for (int i = 0; i < SPI_QUEUE_SIZE; i++) {
      if (done == &g_trans[i]) {
        fillTxBlock(g_spiTx[i]);
        spi_slave_queue_trans(SPI2_HOST, &g_trans[i], portMAX_DELAY);
        break;
      }
    }
  }
}

// -----------------------------
// LED pins (from v1.3)
// -----------------------------
static constexpr uint8_t LED_PIN_CENTER = 2;
static constexpr uint8_t LED_PIN_RIGHT_NEOPIXEL = 47;
static constexpr uint8_t LED_PIN_LEFT_NEOPIXEL  = 48;
static constexpr uint16_t LED_PIXELS_PER_SIDE = 4;

Adafruit_NeoPixel ledRight(LED_PIXELS_PER_SIDE, LED_PIN_RIGHT_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledLeft(LED_PIXELS_PER_SIDE, LED_PIN_LEFT_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledCenter(1, LED_PIN_CENTER, NEO_RGB + NEO_KHZ800);

static inline void centerLedSet(uint8_t r, uint8_t g, uint8_t b) {
  ledCenter.setPixelColor(0, ledCenter.Color(r, g, b));
  ledCenter.show();
}
static inline void centerLedOff() {
  ledCenter.clear();
  ledCenter.show();
}
static inline void sideLedsShowSolid(uint32_t color) {
  for (uint16_t i = 0; i < LED_PIXELS_PER_SIDE; i++) {
    ledLeft.setPixelColor(i, color);
    ledRight.setPixelColor(i, color);
  }
  ledLeft.show();
  ledRight.show();
}
static inline void sideLedsShowOff() {
  sideLedsShowSolid(ledLeft.Color(0, 0, 0));
}

// v1.3: side boot progress bar
static inline void sideLedsShowBoot(uint32_t nowMs) {
  uint32_t t = nowMs - g_bootStartMs;
  uint8_t filled = (uint8_t)((t / 250) % (LED_PIXELS_PER_SIDE + 1));

  const uint32_t blue = ledLeft.Color(0, 0, 80);
  for (uint16_t i = 0; i < LED_PIXELS_PER_SIDE; i++) {
    uint32_t c = (i < filled) ? blue : 0;
    ledLeft.setPixelColor(i, c);
    ledRight.setPixelColor(i, c);
  }
  ledLeft.show();
  ledRight.show();
}

static inline void updateLedStatusFromLinks() {
  // Priority (highest -> lowest):
  // - CONNECTED: Web UI WebSocket (/ws) connected
  // - WIFI_CONNECTED: a station is connected to RTL SoftAP
  // - READY: idle
  // - BOOTING: startup animation window (kept until boot transition completes)
  if (g_ledStatus == LedStatus::BOOTING) return;
  if (g_wsConnected) {
    g_ledStatus = LedStatus::CONNECTED;
  } else if (g_apStaCount > 0) {
    g_ledStatus = LedStatus::WIFI_CONNECTED;
  } else {
    g_ledStatus = LedStatus::READY;
  }
}

static void startSideFlash(uint32_t color, uint8_t blinks, uint16_t onMs, uint16_t offMs) {
  g_sideFlash.color = color;
  g_sideFlash.blinksRemaining = blinks;
  g_sideFlash.on = true;
  g_sideFlash.onMs = onMs;
  g_sideFlash.offMs = offMs;
  g_sideFlash.untilMs = millis() + onMs;
  sideLedsShowSolid(color);
}

static void ledUpdateOnce() {
  const uint32_t nowMs = millis();

  // 1) Side flash overlay
  if (g_sideFlash.blinksRemaining > 0) {
    if (nowMs >= g_sideFlash.untilMs) {
      if (g_sideFlash.on) {
        g_sideFlash.on = false;
        g_sideFlash.untilMs = nowMs + g_sideFlash.offMs;
        sideLedsShowOff();
      } else {
        g_sideFlash.blinksRemaining--;
        if (g_sideFlash.blinksRemaining == 0) {
          // flash done -> fall through
        } else {
          g_sideFlash.on = true;
          g_sideFlash.untilMs = nowMs + g_sideFlash.onMs;
          sideLedsShowSolid(g_sideFlash.color);
        }
      }
    }
  }
  const bool sideOverlayActive = (g_sideFlash.blinksRemaining > 0);

  // 2) Steady-state rendering (ported from v1.3)
  switch (g_ledStatus) {
    case LedStatus::BOOTING: {
      if (!sideOverlayActive) sideLedsShowBoot(nowMs);
      // Center breathing in white
      uint32_t t = (nowMs - g_bootStartMs) % 2000;
      uint8_t b = (t < 1000) ? (uint8_t)(t / 4) : (uint8_t)((2000 - t) / 4);
      centerLedSet(b, b, b);
      // Transition to READY after ~3 seconds
      if (nowMs - g_bootStartMs >= 3000) {
        g_ledStatus = g_wsConnected ? LedStatus::CONNECTED : (g_apStaCount > 0 ? LedStatus::WIFI_CONNECTED : LedStatus::READY);
      }
      break;
    }
    case LedStatus::READY: {
      if (!sideOverlayActive) sideLedsShowOff();
      uint32_t t = nowMs % 2000;
      uint8_t b = (t < 1000) ? (uint8_t)(t / 4) : (uint8_t)((2000 - t) / 4);
      uint8_t r = (uint8_t)(b / 3); // dim red
      centerLedSet(r, 0, 0);
      break;
    }
    case LedStatus::WIFI_CONNECTED: {
      if (!sideOverlayActive) sideLedsShowOff();
      uint32_t t = nowMs % 2000;
      uint8_t b = (t < 1000) ? (uint8_t)(t / 4) : (uint8_t)((2000 - t) / 4);
      uint8_t g = (uint8_t)(b / 4); // dim green
      centerLedSet(0, g, 0);
      break;
    }
    case LedStatus::CONNECTED: {
      if (!sideOverlayActive) sideLedsShowSolid(ledLeft.Color(0, 0, 120)); // blue
      centerLedSet(0, 40, 0); // dim green
      break;
    }
  }
}

// -----------------------------
// Camera pins (from v1.3)
// -----------------------------
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

static void configCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    linkLogf("Camera init failed: 0x%x", err);
    g_cameraInitOk = false;
    return;
  }
  g_cameraInitOk = true;

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    const uint16_t pid = s->id.PID;
    if (pid == OV2640_PID) {
      s->set_hmirror(s, 1);
      s->set_vflip(s, 1);
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, 12);
    } else if (pid == OV5640_PID) {
      s->set_hmirror(s, 1);
      s->set_vflip(s, 0);
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, 10);
    } else {
      s->set_hmirror(s, 1);
    }
  }
}

// -----------------------------
// Motors (from v1.3)
// -----------------------------
static constexpr float WHEEL_DIAMETER_MM = 33.50f;
static constexpr float TRACK_WIDTH_MM    = 73.5f;
static constexpr float STEPS_PER_WHEEL_REV = 2000.0f;

static inline float computeStepPerAngle() {
  return (STEPS_PER_WHEEL_REV * TRACK_WIDTH_MM) / (360.0f * WHEEL_DIAMETER_MM);
}

const int m2stepPin = 40; // right STEP
const int m2dirPin = 39;  // right DIR
const int m2enPin = 38;   // right EN

const int stepPin = 21;  // left STEP
const int dirPin = 42;   // left DIR
const int enPin = 41;    // left EN

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepperLeft = nullptr;
FastAccelStepper* stepperRight = nullptr;

static constexpr float maxSpeedLimit = 2000.0f;

static void moveStep(int step) {
  if (!stepperLeft || !stepperRight) return;
  stepperLeft->setSpeedInHz(1000);
  stepperRight->setSpeedInHz(1000);
  stepperLeft->move(step);
  stepperRight->move(-step);
}

static void moveAngle(int angle) {
  if (!stepperLeft || !stepperRight) return;
  const float stepPerAngle = computeStepPerAngle() * 2;
  stepperLeft->setSpeedInHz(1000);
  stepperRight->setSpeedInHz(1000);
  stepperLeft->move(angle * stepPerAngle);
  stepperRight->move(angle * stepPerAngle);
}

static void moveForward() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(1000); stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(1000); stepperRight->runBackward(); }
}

static void turnLeft() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(700);  stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(1000); stepperRight->runBackward(); }
}

static void turnRight() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(1000); stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(700);  stepperRight->runBackward(); }
}

static void stopRobot() {
  if (stepperLeft)  stepperLeft->forceStop();
  if (stepperRight) stepperRight->forceStop();
}

static void robotMoveJson(const std::string& json) {
  JsonDocument doc;
  if (deserializeJson(doc, json.c_str()) != DeserializationError::Ok) return;
  JsonObject obj = doc.as<JsonObject>();
  float lspeed = obj["lspeed"] | 0.0f;
  float rspeed = obj["rspeed"] | 0.0f;

  if (lspeed > maxSpeedLimit) lspeed = maxSpeedLimit;
  if (lspeed < -maxSpeedLimit) lspeed = -maxSpeedLimit;
  if (rspeed > maxSpeedLimit) rspeed = maxSpeedLimit;
  if (rspeed < -maxSpeedLimit) rspeed = -maxSpeedLimit;

  if (stepperLeft) {
    float sl = fabsf(lspeed);
    stepperLeft->setSpeedInHz(sl);
    if (lspeed >= 0) stepperLeft->runForward(); else stepperLeft->runBackward();
  }
  if (stepperRight) {
    float sr = fabsf(rspeed);
    stepperRight->setSpeedInHz(sr);
    if (rspeed >= 0) stepperRight->runBackward(); else stepperRight->runForward();
  }
}

// -----------------------------
// Command parsing (from v1.3's WS handler)
// -----------------------------
static void handleCommandLine(const String& line) {
  // Link-state updates from RTL (LED state machine input)
  if (line.startsWith("@ws,")) {
    const char v = (line.length() >= 5) ? line.charAt(4) : '0';
    const bool newWs = (v == '1');
    const bool prevWs = g_wsConnected;
    g_wsConnected = newWs;
    updateLedStatusFromLinks();
    // v1.3: WS disconnect -> red 1x long blink on both sides
    if (prevWs && !newWs) {
      startSideFlash(ledLeft.Color(120, 0, 0), 1, 300, 200);
    }
    return;
  }
  if (line.startsWith("@sta,")) {
    const int n = atoi(line.c_str() + 5);
    if (n < 0) return;
    g_apStaCount = (uint16_t)n;
    // v1.3 logic: staCount==0이면 WS 연결도 존재할 수 없으므로 즉시 false로 강등
    if (g_apStaCount == 0) {
      g_wsConnected = false;
    }
    updateLedStatusFromLinks();
    return;
  }

  // (옵션) RTL이 @cam,0/@cam,1을 보내면 스트림을 끄고/켤 수는 있게 유지
  if (line.startsWith("@cam,")) {
    const char v = (line.length() >= 6) ? line.charAt(5) : '1';
    g_camEnabled = (v == '1');
    if (!g_camEnabled && g_fb) {
      esp_camera_fb_return(g_fb);
      g_fb = nullptr;
      g_fbOff = 0;
    }
    return;
  }

  // format: "key,value"
  std::string s(line.c_str());
  std::istringstream ss(s);
  std::string key, value;
  std::getline(ss, key, ',');
  std::getline(ss, value);

  if (key == "robot") {
    robotMoveJson(value);
    return;
  }

  const int valueInt = value.empty() ? 0 : atoi(value.c_str());
  if (key == "stop") stopRobot();
  else if (key == "forward") moveForward();
  else if (key == "left") turnLeft();
  else if (key == "right") turnRight();
  else if (key == "move") moveStep(valueInt);
  else if (key == "angle") moveAngle(valueInt);
}

static bool readLineFromSerial1(String& outLine) {
  static String buf;
  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      outLine = buf;
      buf = "";
      return true;
    }
    if (buf.length() < 512) buf += ch;
  }
  return false;
}

void setup() {
  // UART link
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  if (ENABLE_USB_SERIAL_DEBUG) {
    Serial.begin(115200);
    delay(200);
  }
  linkLogf("DeepCoS3_Robot boot");
  linkLogf("Serial1 baud=%u RX=%d TX=%d", (unsigned)LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);
  linkLogf("SPI slave pins: CS=%d SCLK=%d MISO=%d MOSI=%d (block=%u bytes)",
           SPI_CS_PIN, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, (unsigned)SPI_BLOCK_BYTES);

  // LEDs
  ledLeft.begin();
  ledRight.begin();
  ledCenter.begin();
  ledLeft.setBrightness(128);
  ledRight.setBrightness(128);
  ledCenter.setBrightness(255);
  ledLeft.show();
  ledRight.show();
  ledCenter.show();
  sideLedsShowOff();
  // v1.3 LED state machine starts in BOOTING (white breathing)
  g_bootStartMs = millis();
  g_ledStatus = LedStatus::BOOTING;

  // Motors
  pinMode(enPin, OUTPUT);
  pinMode(m2enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  digitalWrite(m2enPin, LOW);

  engine.init();
  stepperLeft = engine.stepperConnectToPin(stepPin);
  if (stepperLeft) {
    stepperLeft->setDirectionPin(dirPin);
    stepperLeft->setEnablePin(enPin, true);
    stepperLeft->setAutoEnable(true);
    stepperLeft->setAcceleration(10000);
    stepperLeft->setSpeedInHz(1000);
  }
  stepperRight = engine.stepperConnectToPin(m2stepPin);
  if (stepperRight) {
    stepperRight->setDirectionPin(m2dirPin);
    stepperRight->setEnablePin(m2enPin, true);
    stepperRight->setAutoEnable(true);
    stepperRight->setAcceleration(10000);
    stepperRight->setSpeedInHz(1000);
  }

  // Camera (현재는 브릿지 송출은 RTL쪽에서 담당)
  configCamera();

  // SPI slave init (camera stream)
  g_spiInitOk = spiSlaveInit();

  // 부팅 직후 안전 정지
  stopRobot();

  // ---- Start tasks ----
  if (ENABLE_TASK_SPLIT) {
    // SPI 카메라 서비스는 지연에 민감 -> 상대적으로 높은 우선순위
    xTaskCreatePinnedToCore(taskSpiCamera, "spi_cam", 4096, NULL, 3, &g_taskSpiCamera, 0);
    // 제어/UART는 안정적으로 처리 -> 그 다음 우선순위
    xTaskCreatePinnedToCore(taskRobotCtrl, "robot_ctrl", 4096, NULL, 2, &g_taskRobotCtrl, 1);
    // LED는 저우선순위로 상태 표시
    xTaskCreatePinnedToCore(taskLed, "led", 3072, NULL, 1, &g_taskLed, 1);
    linkLogf("Tasks started: spi_cam(core0) + robot_ctrl(core1) + led(core1)");
  } else {
    linkLogf("Tasks disabled: using loop() polling");
  }
}

void loop() {
  // 태스크 분리 모드에서는 loop()는 유휴(Arduino loop task)
  if (ENABLE_TASK_SPLIT) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  // 폴백: 태스크 비활성 시 기존 방식 유지
  spiSlavePoll();
  String line;
  if (readLineFromSerial1(line) && line.length() > 0) {
    handleCommandLine(line);
  }
}

// -----------------------------
// Task implementations
// -----------------------------
static void taskSpiCamera(void* arg) {
  (void)arg;
  for (;;) {
    spiSlavePoll();
    // 너무 빡빡하면 다른 태스크가 굶을 수 있어 최소한 yield
    taskYIELD();
  }
}

static void taskRobotCtrl(void* arg) {
  (void)arg;
  for (;;) {
    String line;
    if (readLineFromSerial1(line)) {
      if (line.length() > 0) {
        handleCommandLine(line);
      }
      // 바로 다음 라인도 있을 수 있어 yield만
      taskYIELD();
      continue;
    }
    // 수신 없으면 약간 쉬어줌
    vTaskDelay(1);
  }
}

static void taskLed(void* arg) {
  (void)arg;
  for (;;) {
    ledUpdateOnce();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

