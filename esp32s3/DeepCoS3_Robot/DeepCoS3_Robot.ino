/*
  DeepCoS3_Robot.ino (ESP32‑S3, Arduino IDE)
  FW Version: 0.1.10  (config.h CFG_FW_VERSION 과 일치시킬 것)

  목표:
  - 기존 DeepcoMini_v1.3의 "카메라/모터/LED/커맨드 파싱"을 유지
  - Wi‑Fi/웹서버는 제거 (네트워크는 RTL8720DN이 담당)
  - RTL8720DN로부터 UART(Serial1)로 들어오는 제어 문자열을 처리

  주의:
  - 핀/파라미터 변경은 config.h에서 수정하세요.
*/

#include "config.h"
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#include <ArduinoJson.h>

// FreeRTOS (Arduino-ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "dcm_spi_protocol.h"

// -----------------------------
// Logging policy (기본 전제: PC는 RTL만 USB로 연결)
// - S3의 USB Serial 로그는 기본적으로 "사용 안 함"
// - 디버그 로그는 Serial1로 RTL에 전달해서 RTL USB 콘솔에서 확인
// - 포맷: "@log,<text>\n"
// -----------------------------
// v0.1.10: 3단계 로그 레벨 — LOG_NONE / LOG_INFO / LOG_DEBUG
// 런타임 변경: @debug,0 / @debug,1 / @debug,2
static uint8_t g_logLevel = CFG_LOG_LEVEL;
static bool    g_logUsb   = CFG_LOG_USB;

// 내부 출력 공통 함수 (레벨 필터링 완료 후 호출)
static void _logOutput(const char* buf) {
  if (g_logUsb) {
    Serial.print("[S3] ");
    Serial.println(buf);
  }
  Serial1.print("@log,");
  Serial1.println(buf);
}

// logInfo: LOG_INFO 이상일 때 출력 (부팅, 연결, OTA, 에러 등 핵심 이벤트)
static void logInfo(const char* fmt, ...) {
  if (g_logLevel < LOG_INFO) return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  _logOutput(buf);
}

// logDebug: LOG_DEBUG 이상일 때 출력 (SPI 상세, 모터 명령, 프레임, 워치독 등)
static void logDebug(const char* fmt, ...) {
  if (g_logLevel < LOG_DEBUG) return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  _logOutput(buf);
}

// 하위 호환: 기존 linkLogf 호출을 유지하되 logInfo로 동작
#define linkLogf logInfo

// -----------------------------
// Task split (v1.3 스타일로 분리)
// - taskSpiCamera : SPI slave 서비스(카메라 프레임 송출)
// - taskRobotCtrl : UART(Serial1 RX) 커맨드 파싱/로봇 제어
//
// NOTE:
// - PC는 RTL만 USB로 연결 → 로그는 Serial1(TX)로만 전달
// - Serial1은 RX(제어) / TX(로그) 동시 사용(풀듀플렉스)
// -----------------------------
static constexpr bool ENABLE_TASK_SPLIT = CFG_ENABLE_TASK_SPLIT;
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
// v0.1.3: 연결 시간 추적
static uint32_t g_wsConnectedSinceMs = 0;
static uint32_t g_staConnectedSinceMs = 0;

struct LedFlash {
  uint32_t color = 0;
  uint8_t blinksRemaining = 0;
  bool on = false;
  uint16_t onMs = 0;
  uint16_t offMs = 0;
  uint32_t untilMs = 0;
};
static LedFlash g_sideFlash;
static portMUX_TYPE g_ledMux = portMUX_INITIALIZER_UNLOCKED;

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

// v0.1.9: SPI OTA
#include <Update.h>

// -----------------------------
// UART link (RTL <-> S3) — 설정은 config.h
// -----------------------------
static constexpr uint32_t LINK_BAUD = CFG_LINK_BAUD;
static constexpr int LINK_RX_PIN = CFG_LINK_RX_PIN;
static constexpr int LINK_TX_PIN = CFG_LINK_TX_PIN;

// -----------------------------
// SPI link (Camera -> RTL) — 설정은 config.h
// -----------------------------
static constexpr int SPI_SCLK_PIN = CFG_SPI_SCLK_PIN;
static constexpr int SPI_MOSI_PIN = CFG_SPI_MOSI_PIN;
static constexpr int SPI_MISO_PIN = CFG_SPI_MISO_PIN;
static constexpr int SPI_CS_PIN   = CFG_SPI_CS_PIN;

static constexpr size_t SPI_BLOCK_BYTES = CFG_SPI_BLOCK_BYTES;
static constexpr int SPI_QUEUE_SIZE = CFG_SPI_QUEUE_SIZE;

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
static SemaphoreHandle_t g_frameMutex = nullptr;

static void fillTxBlock(uint8_t* buf) {
  if (!buf) return;
  if (g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
  do {
    memset(buf, 0, SPI_BLOCK_BYTES);

    DcmSpiHdr* h = (DcmSpiHdr*)buf;
    h->magic[0] = 'D'; h->magic[1] = 'C'; h->magic[2] = 'M'; h->magic[3] = '2';
    h->type = SPI_TYPE_IDLE;
    h->flags = 0;
    h->seq = 0;
    h->total_len = 0;
    h->offset = 0;
    h->payload_len = 0;
    h->crc16 = 0;

    if (!g_camEnabled) {
      break;
    }

    // 프레임 없으면 새로 캡처 (SPI 클럭이 실제로 돌 때만)
    const uint32_t now = millis();
    if (!g_fb) {
      // SPI activity 없으면 캡처하지 않음(불필요한 카메라 부하 감소)
      if (now - g_lastSpiActivityMs > 500) break;
      g_fb = esp_camera_fb_get();
      if (!g_fb || !g_fb->buf || g_fb->len == 0) {
        if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
        break;
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
      h->crc16 = crc16Ccitt(buf + headerSz, pl);
      g_fbOff += pl;
    } else {
      h->crc16 = 0;
    }

    // 마지막 청크까지 tx buffer에 복사했으면 fb는 즉시 반환 가능(이미 buf로 복사됨)
    if (g_fb && g_fbOff >= g_fb->len) {
      esp_camera_fb_return(g_fb);
      g_fb = nullptr;
      g_fbOff = 0;
    }
  } while (0);
  if (g_frameMutex) xSemaphoreGive(g_frameMutex);
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
    logInfo("spi_slave_initialize failed: 0x%x", err);
    // v0.1.2: 에러 전파
    Serial1.printf("@err,SPI_INIT,SPI slave init failed: 0x%x\n", err);
    return false;
  }

  for (int i = 0; i < SPI_QUEUE_SIZE; i++) {
    g_spiTx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    g_spiRx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    if (!g_spiTx[i] || !g_spiRx[i]) {
      logInfo("spi dma malloc failed");
      return false;
    }
    fillTxBlock(g_spiTx[i]);
    memset(&g_trans[i], 0, sizeof(g_trans[i]));
    g_trans[i].length = SPI_BLOCK_BYTES * 8;
    g_trans[i].tx_buffer = g_spiTx[i];
    g_trans[i].rx_buffer = g_spiRx[i];
    err = spi_slave_queue_trans(SPI2_HOST, &g_trans[i], portMAX_DELAY);
    if (err != ESP_OK) {
      logInfo("spi_slave_queue_trans failed: 0x%x", err);
      return false;
    }
  }

  g_lastSpiActivityMs = millis();
  logInfo("SPI slave initialized");
  return true;
}

// =============================================================
// v0.1.9: SPI OTA 수신기 (RTL→S3 SPI OTA)
//
// RTL이 DcmSpiHdr(type=SPI_TYPE_OTA) + firmware payload를
// SPI로 보내면, S3는 RX 버퍼에서 꺼내 Update.h로 Flash 기록.
// 응답은 TX 버퍼에 DcmSpiHdr(type=SPI_TYPE_OTA_ACK).
// =============================================================
static struct {
  bool     active;
  uint32_t expectedSize;
  uint32_t received;
  uint16_t lastSeq;
  bool     error;
  bool     rebootPending;
  uint32_t rebootAtMs;
} g_s3Ota = {};

static void stopRobot(); // forward declaration

static void fillOtaAckBlock(uint8_t* buf) {
  memset(buf, 0, SPI_BLOCK_BYTES);
  DcmSpiHdr* h = (DcmSpiHdr*)buf;
  h->magic[0] = 'D'; h->magic[1] = 'C'; h->magic[2] = 'M'; h->magic[3] = '2';
  h->type = SPI_TYPE_OTA_ACK;
  h->flags = g_s3Ota.error ? SPI_FLAG_OTA_ERR : 0;
  h->seq = g_s3Ota.lastSeq;
  h->total_len = g_s3Ota.received;
  h->payload_len = 0;
  h->crc16 = 0;
}

static void handleSpiOtaBlock(const uint8_t* rxBuf) {
  const DcmSpiHdr* hdr = (const DcmSpiHdr*)rxBuf;

  // Magic / Type 검증
  if (hdr->magic[0] != 'D' || hdr->magic[1] != 'C' ||
      hdr->magic[2] != 'M' || hdr->magic[3] != '2') return;
  if (hdr->type != SPI_TYPE_OTA) return;

  const uint8_t* payload = rxBuf + sizeof(DcmSpiHdr);
  const uint16_t pl = hdr->payload_len;
  g_s3Ota.lastSeq = hdr->seq;

  // START 블록: OTA 초기화
  if (hdr->flags & SPI_FLAG_START) {
    // 에러 플래그가 있으면 OTA 취소 (RTL cancel)
    if (hdr->flags & SPI_FLAG_OTA_ERR) {
      if (g_s3Ota.active) {
        Update.abort();
        g_s3Ota.active = false;
        logInfo("OTA cancelled by RTL");
      }
      return;
    }

    stopRobot();
    g_s3Ota.active = true;
    g_s3Ota.expectedSize = hdr->total_len;
    g_s3Ota.received = 0;
    g_s3Ota.error = false;
    g_s3Ota.rebootPending = false;

    logInfo("OTA begin: %lu bytes", (unsigned long)hdr->total_len);

    if (!Update.begin(hdr->total_len)) {
      g_s3Ota.error = true;
      g_s3Ota.active = false;
      logInfo("OTA Update.begin() failed");
    }
    return; // START 블록에는 payload 없음
  }

  // END 블록: OTA 완료
  if (hdr->flags & SPI_FLAG_END) {
    if (!g_s3Ota.active) return;

    if (hdr->flags & SPI_FLAG_OTA_ERR) {
      Update.abort();
      g_s3Ota.active = false;
      g_s3Ota.error = true;
      logInfo("OTA aborted by RTL (end+err)");
      return;
    }

    if (g_s3Ota.received != g_s3Ota.expectedSize) {
      Update.abort();
      g_s3Ota.error = true;
      g_s3Ota.active = false;
      logInfo("OTA size mismatch: got %lu expected %lu",
               (unsigned long)g_s3Ota.received,
               (unsigned long)g_s3Ota.expectedSize);
      return;
    }

    if (!Update.end(true)) {
      g_s3Ota.error = true;
      g_s3Ota.active = false;
      logInfo("OTA Update.end() failed");
      return;
    }

    logInfo("OTA SUCCESS: %lu bytes, rebooting in 2s",
             (unsigned long)g_s3Ota.received);
    g_s3Ota.active = false;
    g_s3Ota.rebootPending = true;
    g_s3Ota.rebootAtMs = millis() + 2000;
    return;
  }

  // DATA 블록: Flash 기록
  if (!g_s3Ota.active || g_s3Ota.error) return;

  if (pl == 0) return;
  if (pl > SPI_BLOCK_BYTES - sizeof(DcmSpiHdr)) {
    g_s3Ota.error = true;
    logInfo("OTA payload too large: %u", pl);
    return;
  }

  // CRC 검증
  const uint16_t calcCrc = crc16Ccitt(payload, pl);
  if (calcCrc != hdr->crc16) {
    g_s3Ota.error = true;
    logInfo("OTA CRC error: calc=0x%04X hdr=0x%04X", calcCrc, hdr->crc16);
    return;
  }

  // Flash 기록
  const size_t written = Update.write((uint8_t*)payload, pl);
  if (written != pl) {
    g_s3Ota.error = true;
    logInfo("OTA write fail: wrote %u / %u", (unsigned)written, (unsigned)pl);
    return;
  }
  g_s3Ota.received += pl;
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
        // v0.1.9: RX 버퍼에서 OTA 데이터 확인
        const DcmSpiHdr* rxHdr = (const DcmSpiHdr*)g_spiRx[i];
        if (rxHdr->type == SPI_TYPE_OTA &&
            rxHdr->magic[0] == 'D' && rxHdr->magic[1] == 'C' &&
            rxHdr->magic[2] == 'M' && rxHdr->magic[3] == '2') {
          handleSpiOtaBlock(g_spiRx[i]);
          fillOtaAckBlock(g_spiTx[i]); // 응답을 TX에 적재
        } else {
          fillTxBlock(g_spiTx[i]); // 일반 카메라 프레임
        }
        spi_slave_queue_trans(SPI2_HOST, &g_trans[i], portMAX_DELAY);
        break;
      }
    }
  }

  // v0.1.9: OTA 후 지연 리부팅
  if (g_s3Ota.rebootPending && millis() >= g_s3Ota.rebootAtMs) {
    logInfo("OTA reboot now");
    delay(100);
    ESP.restart();
  }
}

// -----------------------------
// LED pins — 설정은 config.h
// -----------------------------
static constexpr uint8_t LED_PIN_CENTER = CFG_LED_CENTER_PIN;
static constexpr uint8_t LED_PIN_RIGHT_NEOPIXEL = CFG_LED_RIGHT_NEOPIXEL_PIN;
static constexpr uint8_t LED_PIN_LEFT_NEOPIXEL  = CFG_LED_LEFT_NEOPIXEL_PIN;
static constexpr uint16_t LED_PIXELS_PER_SIDE = CFG_LED_PIXELS_PER_SIDE;

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
  portENTER_CRITICAL(&g_ledMux);
  g_sideFlash.color = color;
  g_sideFlash.blinksRemaining = blinks;
  g_sideFlash.on = true;
  g_sideFlash.onMs = onMs;
  g_sideFlash.offMs = offMs;
  g_sideFlash.untilMs = millis() + onMs;
  portEXIT_CRITICAL(&g_ledMux);
  sideLedsShowSolid(color);
}

static void ledUpdateOnce() {
  const uint32_t nowMs = millis();
  bool doSideShowOff = false;
  bool doSideShowColor = false;
  uint32_t sideColor = 0;

  // 1) Side flash overlay
  portENTER_CRITICAL(&g_ledMux);
  if (g_sideFlash.blinksRemaining > 0 && nowMs >= g_sideFlash.untilMs) {
    if (g_sideFlash.on) {
      g_sideFlash.on = false;
      g_sideFlash.untilMs = nowMs + g_sideFlash.offMs;
      doSideShowOff = true;
    } else {
      g_sideFlash.blinksRemaining--;
      if (g_sideFlash.blinksRemaining > 0) {
        g_sideFlash.on = true;
        g_sideFlash.untilMs = nowMs + g_sideFlash.onMs;
        doSideShowColor = true;
        sideColor = g_sideFlash.color;
      }
    }
  }
  const bool sideOverlayActive = (g_sideFlash.blinksRemaining > 0);
  portEXIT_CRITICAL(&g_ledMux);

  if (doSideShowOff) sideLedsShowOff();
  if (doSideShowColor) sideLedsShowSolid(sideColor);

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
// Camera pins — 설정은 config.h
// -----------------------------
#define PWDN_GPIO_NUM  CFG_CAM_PWDN_PIN
#define RESET_GPIO_NUM CFG_CAM_RESET_PIN
#define XCLK_GPIO_NUM  CFG_CAM_XCLK_PIN
#define SIOD_GPIO_NUM  CFG_CAM_SIOD_PIN
#define SIOC_GPIO_NUM  CFG_CAM_SIOC_PIN

#define Y2_GPIO_NUM    CFG_CAM_Y2_PIN
#define Y3_GPIO_NUM    CFG_CAM_Y3_PIN
#define Y4_GPIO_NUM    CFG_CAM_Y4_PIN
#define Y5_GPIO_NUM    CFG_CAM_Y5_PIN
#define Y6_GPIO_NUM    CFG_CAM_Y6_PIN
#define Y7_GPIO_NUM    CFG_CAM_Y7_PIN
#define Y8_GPIO_NUM    CFG_CAM_Y8_PIN
#define Y9_GPIO_NUM    CFG_CAM_Y9_PIN

#define VSYNC_GPIO_NUM CFG_CAM_VSYNC_PIN
#define HREF_GPIO_NUM  CFG_CAM_HREF_PIN
#define PCLK_GPIO_NUM  CFG_CAM_PCLK_PIN

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
  config.xclk_freq_hz = CFG_CAM_XCLK_FREQ;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = CFG_CAM_JPEG_QUALITY;
  config.fb_count = CFG_CAM_FB_COUNT;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    logInfo("Camera init failed: 0x%x", err);
    g_cameraInitOk = false;
    // v0.1.2: 에러 전파 — RTL이 WS로 중계
    Serial1.printf("@err,CAM_INIT,Camera init failed: 0x%x\n", err);
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
// Motors — 설정은 config.h
// -----------------------------
static constexpr float WHEEL_DIAMETER_MM = CFG_WHEEL_DIAMETER_MM;
static constexpr float TRACK_WIDTH_MM    = CFG_TRACK_WIDTH_MM;
static constexpr float STEPS_PER_WHEEL_REV = CFG_STEPS_PER_WHEEL_REV;

static inline float computeStepPerAngle() {
  return (STEPS_PER_WHEEL_REV * TRACK_WIDTH_MM) / (360.0f * WHEEL_DIAMETER_MM);
}

const int m2stepPin = CFG_MOTOR_R_STEP_PIN;
const int m2dirPin  = CFG_MOTOR_R_DIR_PIN;
const int m2enPin   = CFG_MOTOR_R_EN_PIN;

const int stepPin = CFG_MOTOR_L_STEP_PIN;
const int dirPin  = CFG_MOTOR_L_DIR_PIN;
const int enPin   = CFG_MOTOR_L_EN_PIN;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepperLeft = nullptr;
FastAccelStepper* stepperRight = nullptr;

static constexpr float maxSpeedLimit = CFG_MAX_SPEED_LIMIT;
static constexpr int MAX_MOVE_STEP_CMD = 50000;
static constexpr int MAX_ANGLE_CMD = 3600;
static constexpr uint32_t CMD_WATCHDOG_MS = 3000;

static void moveStep(int step) {
  if (!stepperLeft || !stepperRight) return;
  stepperLeft->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  stepperRight->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  stepperLeft->move(step);
  stepperRight->move(-step);
}

static void moveAngle(int angle) {
  if (!stepperLeft || !stepperRight) return;
  const float stepPerAngle = computeStepPerAngle() * 2;
  stepperLeft->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  stepperRight->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  stepperLeft->move(angle * stepPerAngle);
  stepperRight->move(angle * stepPerAngle);
}

static void moveForward() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED); stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED); stepperRight->runBackward(); }
}

static void turnLeft() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(CFG_MOTOR_TURN_SLOW_SPEED); stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED); stepperRight->runBackward(); }
}

static void turnRight() {
  if (stepperLeft)  { stepperLeft->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED); stepperLeft->runForward(); }
  if (stepperRight) { stepperRight->setSpeedInHz(CFG_MOTOR_TURN_SLOW_SPEED); stepperRight->runBackward(); }
}

static void stopRobot() {
  if (stepperLeft)  stepperLeft->forceStop();
  if (stepperRight) stepperRight->forceStop();
}

static void robotMoveJson(const char* json) {
  if (!json) return;
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, json);
  if (err != DeserializationError::Ok) {
    logDebug("robot json parse failed: %s", err.c_str());
    return;
  }
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
static void handleCommandLine(const char* line) {
  if (!line || !line[0]) return;

  // Link-state updates from RTL (LED state machine input)
  if (strncmp(line, "@ws,", 4) == 0) {
    const char v = line[4] ? line[4] : '0';
    const bool newWs = (v == '1');
    const bool prevWs = g_wsConnected;
    g_wsConnected = newWs;
    // v0.1.3: 연결 시점 기록
    if (newWs && !prevWs) g_wsConnectedSinceMs = millis();
    updateLedStatusFromLinks();
    // v1.3: WS disconnect -> red 1x long blink on both sides
    if (prevWs && !newWs) {
      startSideFlash(ledLeft.Color(120, 0, 0), 1, 300, 200);
    }
    return;
  }
  if (strncmp(line, "@sta,", 5) == 0) {
    const int n = atoi(line + 5);
    if (n < 0) return;
    const uint16_t prev = g_apStaCount;
    g_apStaCount = (uint16_t)n;
    // v0.1.3: 연결 시점 기록
    if (n > 0 && prev == 0) g_staConnectedSinceMs = millis();
    // v1.3 logic: staCount==0이면 WS 연결도 존재할 수 없으므로 즉시 false로 강등
    if (g_apStaCount == 0) {
      g_wsConnected = false;
    }
    updateLedStatusFromLinks();
    return;
  }

  // v0.1.10: @debug,0/1/2 — 3단계 로그 레벨 변경
  if (strncmp(line, "@debug,", 7) == 0) {
    const char v = line[7] ? line[7] : '0';
    const uint8_t newLevel = (uint8_t)(v - '0');
    if (newLevel <= LOG_DEBUG) {
      g_logLevel = newLevel;
    } else {
      g_logLevel = LOG_NONE;
    }
    // 레벨 변경 알림은 항상 출력 (Serial1 경유)
    {
      static const char* levelNames[] = {"NONE", "INFO", "DEBUG"};
      const char* name = (g_logLevel <= LOG_DEBUG) ? levelNames[g_logLevel] : "?";
      char buf[64];
      snprintf(buf, sizeof(buf), "Log level -> %s (%d)", name, g_logLevel);
      // 강제 출력 (레벨 무관)
      if (g_logUsb) { Serial.print("[S3] "); Serial.println(buf); }
      Serial1.print("@log,"); Serial1.println(buf);
    }
    return;
  }

  // (옵션) RTL이 @cam,0/@cam,1을 보내면 스트림을 끄고/켤 수는 있게 유지
  if (strncmp(line, "@cam,", 5) == 0) {
    const char v = line[5] ? line[5] : '1';
    if (g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
    g_camEnabled = (v == '1');
    if (!g_camEnabled && g_fb) {
      esp_camera_fb_return(g_fb);
      g_fb = nullptr;
      g_fbOff = 0;
    }
    if (g_frameMutex) xSemaphoreGive(g_frameMutex);
    return;
  }

  // format: "key,value"
  const char* comma = strchr(line, ',');
  if (!comma) return;
  const size_t keyLen = (size_t)(comma - line);
  const char* value = comma + 1;

  if (keyLen == 5 && strncmp(line, "robot", 5) == 0) {
    robotMoveJson(value);
    return;
  }

  const int valueInt = value[0] ? atoi(value) : 0;
  if (keyLen == 4 && strncmp(line, "stop", 4) == 0) stopRobot();
  else if (keyLen == 7 && strncmp(line, "forward", 7) == 0) moveForward();
  else if (keyLen == 4 && strncmp(line, "left", 4) == 0) turnLeft();
  else if (keyLen == 5 && strncmp(line, "right", 5) == 0) turnRight();
  else if (keyLen == 4 && strncmp(line, "move", 4) == 0) {
    int clamped = valueInt;
    if (clamped > MAX_MOVE_STEP_CMD) clamped = MAX_MOVE_STEP_CMD;
    if (clamped < -MAX_MOVE_STEP_CMD) clamped = -MAX_MOVE_STEP_CMD;
    moveStep(clamped);
  } else if (keyLen == 5 && strncmp(line, "angle", 5) == 0) {
    int clamped = valueInt;
    if (clamped > MAX_ANGLE_CMD) clamped = MAX_ANGLE_CMD;
    if (clamped < -MAX_ANGLE_CMD) clamped = -MAX_ANGLE_CMD;
    moveAngle(clamped);
  }
}

static bool readLineFromSerial1(char* outLine, size_t outLen) {
  if (!outLine || outLen == 0) return false;
  static char buf[512];
  static size_t pos = 0;
  static bool overflowed = false;
  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (overflowed) {
        logDebug("UART line overflow (>512), line dropped");
        pos = 0;
        overflowed = false;
        return false;
      }
      if (pos >= outLen) pos = outLen - 1;
      memcpy(outLine, buf, pos);
      outLine[pos] = '\0';
      pos = 0;
      return true;
    }
    if (pos < sizeof(buf) - 1) {
      buf[pos++] = ch;
    } else {
      overflowed = true;
    }
  }
  return false;
}

void setup() {
  // UART link
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  // v0.1.3: USB Serial은 항상 초기화 (런타임 디버그 토글 지원)
  Serial.begin(115200);
  delay(200);
  logInfo("DeepCoS3_Robot boot (v%s, log=%d)", CFG_FW_VERSION, g_logLevel);
  logInfo("Serial1 baud=%u RX=%d TX=%d", (unsigned)LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);
  logDebug("SPI slave pins: CS=%d SCLK=%d MISO=%d MOSI=%d (block=%u bytes)",
           SPI_CS_PIN, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, (unsigned)SPI_BLOCK_BYTES);

  // LEDs
  ledLeft.begin();
  ledRight.begin();
  ledCenter.begin();
  ledLeft.setBrightness(CFG_LED_BRIGHTNESS_SIDE);
  ledRight.setBrightness(CFG_LED_BRIGHTNESS_SIDE);
  ledCenter.setBrightness(CFG_LED_BRIGHTNESS_CENTER);
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
    stepperLeft->setAcceleration(CFG_MOTOR_ACCELERATION);
    stepperLeft->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  }
  stepperRight = engine.stepperConnectToPin(m2stepPin);
  if (stepperRight) {
    stepperRight->setDirectionPin(m2dirPin);
    stepperRight->setEnablePin(m2enPin, true);
    stepperRight->setAutoEnable(true);
    stepperRight->setAcceleration(CFG_MOTOR_ACCELERATION);
    stepperRight->setSpeedInHz(CFG_MOTOR_DEFAULT_SPEED);
  }

  // Camera (현재는 브릿지 송출은 RTL쪽에서 담당)
  configCamera();

  // Camera frame shared-state mutex
  g_frameMutex = xSemaphoreCreateMutex();
  if (!g_frameMutex) {
    logInfo("WARN: frame mutex create failed");
  }

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
    logInfo("Tasks started: spi_cam(core0) + robot_ctrl(core1) + led(core1)");
  } else {
    logInfo("Tasks disabled: using loop() polling");
  }

  // v0.1.10: 부팅 진단 요약 (로그 레벨 표시)
  static const char* levelNames[] = {"NONE", "INFO", "DEBUG"};
  logInfo("Boot diag: cam=%s spi=%s log=%s(%d) usb=%s",
           g_cameraInitOk ? "OK" : "FAIL",
           g_spiInitOk ? "OK" : "FAIL",
           (g_logLevel <= LOG_DEBUG) ? levelNames[g_logLevel] : "?",
           g_logLevel,
           g_logUsb ? "ON" : "OFF");
}

void loop() {
  // 태스크 분리 모드에서는 loop()는 유휴(Arduino loop task)
  if (ENABLE_TASK_SPLIT) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  // 폴백: 태스크 비활성 시 기존 방식 유지
  if (g_spiInitOk) {
    spiSlavePoll();
  } else {
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  char line[513];
  if (readLineFromSerial1(line, sizeof(line)) && line[0] != '\0') {
    handleCommandLine(line);
  }
}

// -----------------------------
// Task implementations
// -----------------------------
static void taskSpiCamera(void* arg) {
  (void)arg;
  for (;;) {
    if (g_spiInitOk) {
      spiSlavePoll();
    } else {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    // 너무 빡빡하면 다른 태스크가 굶을 수 있어 최소한 yield
    taskYIELD();
  }
}

static void taskRobotCtrl(void* arg) {
  (void)arg;
  uint32_t lastCmdMs = millis();
  bool watchdogStopped = false;
  for (;;) {
    char line[513];
    if (readLineFromSerial1(line, sizeof(line))) {
      if (line[0] != '\0') {
        if (line[0] != '@') {
          // 모터 제어 명령 계열만 워치독 타이머 리셋
          lastCmdMs = millis();
          watchdogStopped = false;
        }
        handleCommandLine(line);
      }
      // 바로 다음 라인도 있을 수 있어 yield만
      taskYIELD();
      continue;
    }
    if ((millis() - lastCmdMs) > CMD_WATCHDOG_MS) {
      stopRobot();
      if (!watchdogStopped) {
        logDebug("command watchdog stop (>%lums no UART command)", (unsigned long)CMD_WATCHDOG_MS);
        watchdogStopped = true;
      }
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

