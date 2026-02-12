#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FastAccelStepper.h>
#include <string.h>
#include <stdarg.h>
#include "esp_camera.h"
#include "sensor.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dcm_spi_protocol.h"

// ==============================================
// Step motor (ported from DeepcoMini_v1_0_copy)
// ==============================================
const int stepPin = CFG_MOTOR_L_STEP_PIN;   // left STEP
const int dirPin  = CFG_MOTOR_L_DIR_PIN;    // left DIR
const int enPin   = CFG_MOTOR_L_EN_PIN;     // left EN

const int m2stepPin = CFG_MOTOR_R_STEP_PIN; // right STEP
const int m2dirPin  = CFG_MOTOR_R_DIR_PIN;  // right DIR
const int m2enPin   = CFG_MOTOR_R_EN_PIN;   // right EN

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepperLeft = nullptr;
FastAccelStepper* stepperRight = nullptr;

bool motor_sync = true;
bool stepLeft_flag = false;
bool stepRight_flag = false;
bool quick_stop_flag = false;

const float maxSpeedLimit = CFG_MAX_SPEED_LIMIT; // SPS

// ==============================================
// Camera standalone test (Stage 1)
// - Ported from DeepcoMini_v1.3 config/tuning
// - Goal: verify camera init + frame capture stability before SPI/WS
// ==============================================
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

static bool g_cameraOk = false;
static uint32_t g_camFrames = 0;
static uint32_t g_camFails = 0;
static TaskHandle_t g_spiTaskHandle = nullptr;

// SPI slave (S3 -> RTL camera stream)
static constexpr int SPI_SCLK_PIN = CFG_SPI_SCLK_PIN;
static constexpr int SPI_MOSI_PIN = CFG_SPI_MOSI_PIN;
static constexpr int SPI_MISO_PIN = CFG_SPI_MISO_PIN;
static constexpr int SPI_CS_PIN   = CFG_SPI_CS_PIN;
static constexpr size_t SPI_BLOCK_BYTES = CFG_SPI_BLOCK_BYTES;
static constexpr int SPI_QUEUE_SIZE = CFG_SPI_QUEUE_SIZE;
static uint8_t* g_spiTx[SPI_QUEUE_SIZE] = {nullptr, nullptr};
static uint8_t* g_spiRx[SPI_QUEUE_SIZE] = {nullptr, nullptr};
static spi_slave_transaction_t g_trans[SPI_QUEUE_SIZE];
static bool g_spiOk = false;

// Snap-based camera: frame is captured in command handler (loop task),
// then served by SPI service task. fillTxBlock never blocks.
static camera_fb_t* g_fb = nullptr;
static size_t g_fbOff = 0;
static uint16_t g_frameSeq = 0;
static volatile bool g_snapReady = false;  // true = frame captured, SPI can serve it
static volatile bool g_streaming = false;  // true = continuous capture mode (no UART per frame)

static void s3Logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
  // Mirror to RTL via UART1 so logs are visible on RTL USB console.
  Serial1.print("@log,");
  Serial1.println(buf);
}

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
    g_cameraOk = false;
    s3Logf("[S3][CAM] init fail: 0x%x", err);
    return;
  }
  g_cameraOk = true;
  s3Logf("[S3][CAM] init OK");

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    const uint16_t pid = s->id.PID;
    if (pid == OV2640_PID) {
      s->set_hmirror(s, 1);
      s->set_vflip(s, 1);
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, 12);
      s3Logf("[S3][CAM] sensor=OV2640");
    } else if (pid == OV5640_PID) {
      s->set_hmirror(s, 1);
      s->set_vflip(s, 0);
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, 10);
      s3Logf("[S3][CAM] sensor=OV5640");
    } else {
      s->set_hmirror(s, 1);
      s3Logf("[S3][CAM] sensor=0x%04X", pid);
    }
  }
}

// ---- Snap command handler (called from loop task — blocking OK) ----
static void handleSnap() {
  if (!g_cameraOk) {
    Serial1.println("@snap,fail,no_camera");
    return;
  }

  // 이전 프레임이 남아있으면 강제 해제
  if (g_snapReady || g_fb) {
    if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
    g_fbOff = 0;
    g_snapReady = false;
  }

  // 카메라 프레임 캡처 (blocking OK — loop task)
  g_fb = esp_camera_fb_get();
  if (!g_fb || !g_fb->buf || g_fb->len == 0) {
    g_camFails++;
    if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
    Serial1.println("@snap,fail,capture");
    return;
  }

  g_fbOff = 0;
  g_frameSeq++;
  if (g_frameSeq == 0) g_frameSeq = 1;
  g_camFrames++;
  g_snapReady = true;

  char resp[32];
  snprintf(resp, sizeof(resp), "@snap,ok,%lu", (unsigned long)g_fb->len);
  Serial1.println(resp);

  // 스트리밍 중 로그 범람 방지: 50프레임마다 또는 첫 프레임만 로그
  if (g_frameSeq <= 1 || (g_frameSeq % 50) == 0) {
    s3Logf("[S3][SNAP] seq=%u size=%lu", (unsigned)g_frameSeq, (unsigned long)g_fb->len);
  }
}

// ---- Stream start/stop (called from loop task via UART command) ----
static void handleStreamStart() {
  if (!g_cameraOk) {
    Serial1.println("@stream,fail,no_camera");
    return;
  }
  // 기존 프레임 정리
  if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
  g_fbOff = 0;
  g_snapReady = false;
  g_streaming = true;
  Serial1.println("@stream,ok");
  s3Logf("[S3][STREAM] started");
}

static void handleStreamStop() {
  g_streaming = false;
  if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
  g_fbOff = 0;
  g_snapReady = false;
  Serial1.println("@stream,ok");
  s3Logf("[S3][STREAM] stopped");
}

// ---- SPI TX block fill (called from SPI service task — NEVER blocks) ----
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
  h->crc16 = 0;

  // 카메라 프레임이 준비되지 않았으면 IDLE 반환
  if (!g_snapReady || !g_fb) return;

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
  }

  // 프레임 전송 완료 시 정리
  if (g_fbOff >= g_fb->len) {
    esp_camera_fb_return(g_fb);
    g_fb = nullptr;
    g_fbOff = 0;
    g_snapReady = false;
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
    s3Logf("[S3][SPI] init fail: 0x%x", err);
    return false;
  }

  for (int i = 0; i < SPI_QUEUE_SIZE; i++) {
    g_spiTx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    g_spiRx[i] = (uint8_t*)heap_caps_malloc(SPI_BLOCK_BYTES, MALLOC_CAP_DMA);
    if (!g_spiTx[i] || !g_spiRx[i]) {
      s3Logf("[S3][SPI] dma alloc fail");
      return false;
    }
    fillTxBlock(g_spiTx[i]);
    memset(&g_trans[i], 0, sizeof(g_trans[i]));
    g_trans[i].length = SPI_BLOCK_BYTES * 8;
    g_trans[i].tx_buffer = g_spiTx[i];
    g_trans[i].rx_buffer = g_spiRx[i];
    err = spi_slave_queue_trans(SPI2_HOST, &g_trans[i], 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      s3Logf("[S3][SPI] queue fail: 0x%x", err);
      return false;
    }
  }

  s3Logf("[S3][SPI] init OK (SCLK=%d MOSI=%d MISO=%d CS=%d)",
         SPI_SCLK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN);
  return true;
}

static void spiServiceTask(void* parameter) {
  (void)parameter;
  for (;;) {
    // Block up to 10ms waiting for a completed transaction (better than busy-poll)
    spi_slave_transaction_t* done = nullptr;
    esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, pdMS_TO_TICKS(10));
    if (err != ESP_OK || !done) continue;

    // Find which slot completed and refill it
    for (int i = 0; i < SPI_QUEUE_SIZE; i++) {
      if (done == &g_trans[i]) {
        fillTxBlock(g_spiTx[i]);
        err = spi_slave_queue_trans(SPI2_HOST, &g_trans[i], 0);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
          s3Logf("[S3][SPI] requeue fail: 0x%x", err);
        }
        break;
      }
    }
  }
}

void moveStep(int step) {
  if (stepperLeft && stepperRight) {
    stepperLeft->setSpeedInHz(1000);
    stepperRight->setSpeedInHz(1000);
    stepperLeft->move(step);
    stepperRight->move(-step);
    motor_sync = true;
  }
}

void moveAngle(int angle) {
  const float stepPerAngle = 11.21111f * 2.0f;
  if (stepperLeft && stepperRight) {
    stepperLeft->setSpeedInHz(1000);
    stepperRight->setSpeedInHz(1000);
    stepperLeft->move(angle * stepPerAngle);
    stepperRight->move(angle * stepPerAngle);
  }

  quick_stop_flag = false;
  motor_sync = true;
  stepLeft_flag = true;
  stepRight_flag = true;
}

void moveForward() {
  motor_sync = false;
  if (stepperLeft) {
    stepperLeft->setSpeedInHz(1000);
    stepperLeft->runForward();
  }
  if (stepperRight) {
    stepperRight->setSpeedInHz(1000);
    stepperRight->runBackward();
  }
}

void turnLeft() {
  motor_sync = false;
  if (stepperLeft) {
    stepperLeft->setSpeedInHz(700);
    stepperLeft->runForward();
  }
  if (stepperRight) {
    stepperRight->setSpeedInHz(1000);
    stepperRight->runBackward();
  }
}

void turnRight() {
  motor_sync = false;
  if (stepperLeft) {
    stepperLeft->setSpeedInHz(1000);
    stepperLeft->runForward();
  }
  if (stepperRight) {
    stepperRight->setSpeedInHz(700);
    stepperRight->runBackward();
  }
}

void stop() {
  motor_sync = true;
  if (stepperLeft) stepperLeft->forceStop();
  if (stepperRight) stepperRight->forceStop();
}

void robotMove(const char* command) {
  if (!command) return;
  JsonDocument doc;
  deserializeJson(doc, command);
  JsonObject object1 = doc.as<JsonObject>();
  float lspeed = object1["lspeed"];
  float rspeed = object1["rspeed"];

  if (lspeed > maxSpeedLimit) lspeed = maxSpeedLimit;
  if (lspeed < -maxSpeedLimit) lspeed = -maxSpeedLimit;
  if (rspeed > maxSpeedLimit) rspeed = maxSpeedLimit;
  if (rspeed < -maxSpeedLimit) rspeed = -maxSpeedLimit;

  motor_sync = false;

  if (stepperLeft){
    float sl = fabsf(lspeed);
    stepperLeft->setSpeedInHz(sl);
    if (lspeed >= 0) stepperLeft->runForward(); else stepperLeft->runBackward();
  }
  if (stepperRight){
    float sr = fabsf(rspeed);
    stepperRight->setSpeedInHz(sr);
    if (rspeed >= 0) stepperRight->runBackward(); else stepperRight->runForward();
  }
}

static void handleCommandLine(const char* line) {
  if (!line || !line[0]) return;
  Serial.printf("[S3][CMD-IN] %s\n", line);

  if (strcmp(line, "@snap") == 0) {
    handleSnap();
    return;
  }
  if (strcmp(line, "@stream,start") == 0) {
    handleStreamStart();
    return;
  }
  if (strcmp(line, "@stream,stop") == 0) {
    handleStreamStop();
    return;
  }

  const char* comma = strchr(line, ',');
  if (!comma) return;

  const size_t keyLen = (size_t)(comma - line);
  const char* value = comma + 1;

  if (keyLen == 5 && strncmp(line, "robot", 5) == 0) {
    robotMove(value);
    return;
  }

  if (!value[0]) return;
  const int valueInt = atoi(value);

  if (keyLen == 4 && strncmp(line, "stop", 4) == 0) {
    stop();
  } else if (keyLen == 7 && strncmp(line, "forward", 7) == 0) {
    moveForward();
  } else if (keyLen == 4 && strncmp(line, "left", 4) == 0) {
    turnLeft();
  } else if (keyLen == 5 && strncmp(line, "right", 5) == 0) {
    turnRight();
  } else if (keyLen == 4 && strncmp(line, "move", 4) == 0) {
    moveStep(valueInt);
  } else if (keyLen == 5 && strncmp(line, "angle", 5) == 0) {
    moveAngle(valueInt);
  }
}

static bool readLineFromSerial(char* outLine, size_t outLen) {
  if (!outLine || outLen == 0) return false;
  static char buf[512];
  static size_t pos = 0;
  static bool overflowed = false;
  static uint32_t lastRxMs = 0;
  static constexpr uint32_t kIdleFlushMs = 40;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    lastRxMs = millis();
    if (ch == '\0' || ch == '\r') continue;
    if (ch == '\n') {
      if (overflowed) {
        Serial.println("[S3][SERIAL] line overflow (>512), dropped");
        pos = 0;
        overflowed = false;
        return false;
      }
      if (pos >= outLen) pos = outLen - 1;
      memcpy(outLine, buf, pos);
      outLine[pos] = '\0';
      pos = 0;
      Serial.printf("[S3][SERIAL-RX] %s\n", outLine);
      return true;
    }
    if (pos < sizeof(buf) - 1) {
      buf[pos++] = ch;
    } else {
      overflowed = true;
    }
  }

  // Support "No line ending" in serial monitor
  if (pos > 0 && (millis() - lastRxMs) > kIdleFlushMs) {
    if (overflowed) {
      Serial.println("[S3][SERIAL] line overflow (>512), dropped");
      pos = 0;
      overflowed = false;
      return false;
    }
    if (pos >= outLen) pos = outLen - 1;
    memcpy(outLine, buf, pos);
    outLine[pos] = '\0';
    pos = 0;
    Serial.printf("[S3][SERIAL-RX] %s\n", outLine);
    return true;
  }
  return false;
}

static bool readLineFromSerial1(char* outLine, size_t outLen) {
  if (!outLine || outLen == 0) return false;
  static char buf[512];
  static size_t pos = 0;
  static bool overflowed = false;
  static uint32_t lastRxMs = 0;
  static constexpr uint32_t kIdleFlushMs = 40;

  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    lastRxMs = millis();
    if (ch == '\0' || ch == '\r') continue;
    if (ch == '\n') {
      if (overflowed) {
        Serial.println("[S3][UART1] line overflow (>512), dropped");
        pos = 0;
        overflowed = false;
        return false;
      }
      if (pos >= outLen) pos = outLen - 1;
      memcpy(outLine, buf, pos);
      outLine[pos] = '\0';
      pos = 0;
      Serial.printf("[S3][UART1-RX] %s\n", outLine);
      return true;
    }
    if (pos < sizeof(buf) - 1) {
      buf[pos++] = ch;
    } else {
      overflowed = true;
    }
  }

  if (pos > 0 && (millis() - lastRxMs) > kIdleFlushMs) {
    if (overflowed) {
      Serial.println("[S3][UART1] line overflow (>512), dropped");
      pos = 0;
      overflowed = false;
      return false;
    }
    if (pos >= outLen) pos = outLen - 1;
    memcpy(outLine, buf, pos);
    outLine[pos] = '\0';
    pos = 0;
    Serial.printf("[S3][UART1-RX] %s\n", outLine);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(CFG_LINK_BAUD);
  Serial1.begin(CFG_LINK_BAUD, SERIAL_8N1, CFG_LINK_RX_PIN, CFG_LINK_TX_PIN);
  delay(200);
  s3Logf("[S3][BOOT] serial motor test");
  s3Logf("[S3][UART1] baud=%u RX=%d TX=%d", (unsigned)CFG_LINK_BAUD, CFG_LINK_RX_PIN, CFG_LINK_TX_PIN);

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

  s3Logf("[S3][MOTOR] INIT L=%d R=%d", stepperLeft ? 1 : 0, stepperRight ? 1 : 0);
  s3Logf("[S3][READY] commands: stop, forward, left, right, move, angle, robot");
  configCamera();
  g_spiOk = spiSlaveInit();
  xTaskCreate(spiServiceTask, "spi_service", 6144, NULL, 2, &g_spiTaskHandle);
  s3Logf("[S3][CAM] spi_service started (spi=%d cam=%d)", g_spiOk ? 1 : 0, g_cameraOk ? 1 : 0);
}

void loop() {
  char line[513];
  if (readLineFromSerial(line, sizeof(line)) && line[0] != '\0') {
    handleCommandLine(line);
  }
  if (readLineFromSerial1(line, sizeof(line)) && line[0] != '\0') {
    handleCommandLine(line);
  }

  // Stream mode: 이전 프레임이 SPI로 전송 완료되면 자동으로 다음 프레임 캡처
  // UART 응답 없음 — SPI 헤더만으로 RTL이 프레임을 조립
  if (g_streaming && !g_snapReady && g_cameraOk) {
    if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
    g_fb = esp_camera_fb_get();  // blocking OK (loop task)
    if (g_fb && g_fb->buf && g_fb->len > 0) {
      g_fbOff = 0;
      g_frameSeq++;
      if (g_frameSeq == 0) g_frameSeq = 1;
      g_camFrames++;
      g_snapReady = true;
    } else {
      g_camFails++;
      if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }
    }
  } else {
    delay(1);  // 스트리밍 중 아닐 때만 yield (esp_camera_fb_get이 자체 yield)
  }
}

