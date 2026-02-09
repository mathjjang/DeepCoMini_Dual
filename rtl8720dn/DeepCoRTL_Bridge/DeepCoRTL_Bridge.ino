/*
  DeepCoRTL_Bridge.ino (RTL8720DN / BW16 등, Arduino IDE)

  목표:
  - RTL8720DN이 Wi‑Fi AP + WebSocket 서버를 제공 (로봇 IP = 192.168.4.1)
  - DeepCoConnector가 기대하는 엔드포인트를 유지
    - control: ws://192.168.4.1/ws      (port 80)
    - camera : ws://192.168.4.1:81/     (port 81)
  - 수신한 제어 문자열을 UART로 ESP32‑S3에게 전달

  필요 라이브러리(Arduino Library Manager):
  - WebSockets2_Generic (khoih-prog)  -> #include <ArduinoWebsockets.h>

  보드 패키지:
  - Realtek AmebaD (RTL8720DN) 보드 매니저 설치 필요
*/

#include <WiFi.h>
#include <SPI.h>

// WebSockets2_Generic uses ArduinoWebsockets API (gilmaimon/TinyWebsockets 기반)
#include <ArduinoWebsockets.h>
using namespace websockets;

// Realtek/Ameba low-level Wi-Fi API (AP client list)
extern "C" {
  #include "wifi_conf.h"
}

// -----------------------------
// Optional: split work into RTOS tasks (if available)
// - Task A: WS accept/poll + UART (robot control + log pump)
// - Task B: SPI pull (camera frames) -> keep latest frame cached
// -----------------------------
#if defined(__has_include)
  #if __has_include("FreeRTOS.h")
    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"
    #define RTL_HAS_FREERTOS 1
  #elif __has_include(<FreeRTOS.h>)
    #include <FreeRTOS.h>
    #include <task.h>
    #include <semphr.h>
    #define RTL_HAS_FREERTOS 1
  #elif __has_include("freertos/FreeRTOS.h")
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
    #define RTL_HAS_FREERTOS 1
  #else
    #define RTL_HAS_FREERTOS 0
  #endif
#else
  #define RTL_HAS_FREERTOS 0
#endif

// 사용자가 원하는 구조(태스크 분리)를 기본값으로 켬.
// 단, RTL Arduino 코어에서 FreeRTOS 헤더가 없으면 자동으로 loop() 방식으로 폴백됩니다.
static constexpr bool RTL_USE_TASKS = (RTL_HAS_FREERTOS != 0);
static bool g_tasksEnabledRuntime = RTL_USE_TASKS; // task 생성 실패 시 loop 모드로 런타임 폴백
#if RTL_HAS_FREERTOS
static bool g_tasksStartedRuntime = false;
#endif

// -----------------------------
// User config
// -----------------------------
static const char* AP_PASSWORD = "12345678";

// 5GHz를 쓰고 싶으면 보통 36/40/44/48 또는 149/153/157/161 등으로 시도
// (보드/펌웨어/지역설정에 따라 지원 채널이 제한될 수 있음)
static const int   AP_CHANNEL  = 36;

// 로봇 고정 IP (DeepCoConnector 호환)
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_SN(255, 255, 255, 0);
static const IPAddress AP_DNS(8, 8, 8, 8);

// UART link to ESP32‑S3 (Robot control)
// NOTE:
// - AmebaD의 Serial1은 보통 "핀 재매핑"이 아니라 보드 variant에 고정된 핀을 사용합니다.
// - BW16(variant: rtl8720dn_bw16) 기준 핀은 아래 상수에 "문서용"으로 명시합니다.
static const uint32_t LINK_BAUD = 115200;

// -----------------------------
// Pin map (BW16 기준, 코드에서 바로 보이게)
// -----------------------------
// USB 디버그 콘솔(보드 USB-C로 연결되는 경우가 많음)
static constexpr int RTL_LOG_TX_PIN = D0; // PA7
static constexpr int RTL_LOG_RX_PIN = D1; // PA8

// UART 링크(Serial1): RTL <-> S3
static constexpr int RTL_LINK_TX_PIN = D4; // PB1, RTL -> S3 (S3 RX로 연결)
static constexpr int RTL_LINK_RX_PIN = D5; // PB2, S3 -> RTL (S3 TX에서 들어옴)

// SPI Master(카메라 링크): RTL(마스터) <-> S3(슬레이브)
static constexpr int RTL_SPI_SS_PIN_DOC   = D9;  // PA15
static constexpr int RTL_SPI_SCLK_PIN_DOC = D10; // PA14
static constexpr int RTL_SPI_MISO_PIN_DOC = D11; // PA13
static constexpr int RTL_SPI_MOSI_PIN_DOC = D12; // PA12

// -----------------------------
// SPI link (Camera stream)
// RTL8720DN = SPI Master, ESP32‑S3 = SPI Slave
// -----------------------------
// NOTE(AmebaD SPI):
// - SPI SS는 보드(variant)의 SPI_SS를 쓰는 게 가장 안전합니다.
// - 아래 SPI_SS_PIN은 "CS로 토글할 Arduino pin number" 입니다.
//   (BW16 기준: SPI_SS == D9 == PA15)
// 혼동 방지를 위해, BW16에서는 명시적으로 D9를 사용합니다.
static constexpr int SPI_SS_PIN = RTL_SPI_SS_PIN_DOC; // D9 (PA15)
static constexpr uint32_t SPI_HZ = 30000000; // QVGA 30fps 목표: 30MHz부터 시도(배선/보드에 따라 조정)
static constexpr size_t SPI_BLOCK_BYTES = 2048; // DMA/메모리 안전을 위해 2KB 블록

// Camera bridge: ON (완성본)
#define ENABLE_CAMERA_BRIDGE 1

// 설계 방침(사용자 요청):
// - RTL은 SPI로 "항상" 카메라 프레임을 당겨서 최신 프레임을 유지한다.
// - WS 카메라 클라이언트가 연결되어 있을 때만 네트워크로 송출한다.

// -----------------------------
// Helpers
// -----------------------------
static void buildSsidFromMac(char* out, size_t outLen) {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  // DCM-XXXXXXXX 스타일 유지 (마지막 4바이트)
  snprintf(out, outLen, "DCM-%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
}

static void sendSafetyStopToS3() {
  // DeepcoMini v1.3 커맨드 규약 그대로
  Serial1.println("stop,100");
}

// -----------------------------
// WebSocket servers
// -----------------------------
WebsocketsServer wsControl;
WebsocketsClient controlClient;   // single client (DeepCoConnector 1개 전제)
static bool hasControlClient = false;

#if ENABLE_CAMERA_BRIDGE
WebsocketsServer wsCamera;
WebsocketsClient cameraClient;
static bool hasCameraClient = false;
#endif

// -----------------------------
// LED link-state reporting to S3 (so S3 can reproduce v1.3 LED logic)
// - "@ws,0/1"  : control WS (/ws) connected
// - "@sta,N"   : number of stations associated to RTL SoftAP (real count via wifi_get_associated_client_list)
// -----------------------------
static uint8_t g_ledWs = 0;
static uint16_t g_ledStaCount = 0;
static uint32_t g_lastStaPollMs = 0;

static uint16_t getApStaCount() {
  // wifi_conf.h / wifi_structures.h:
  // - wifi_get_associated_client_list(void* buf, unsigned short len)
  // - rtw_maclist_t { unsigned int count; rtw_mac_t mac_list[1]; }
  // AP_STA_NUM is the max client count supported by driver/SDK.
  uint8_t buf[sizeof(rtw_maclist_t) + (AP_STA_NUM - 1) * sizeof(rtw_mac_t)] = {0};
  const int ret = wifi_get_associated_client_list(buf, (unsigned short)sizeof(buf));
  if (ret != RTW_SUCCESS) return 0;
  const rtw_maclist_t* ml = (const rtw_maclist_t*)buf;
  if (!ml) return 0;
  if (ml->count > AP_STA_NUM) return (uint16_t)AP_STA_NUM;
  return (uint16_t)ml->count;
}

static void sendLedLinkStateToS3(bool force = false) {
  // station count polling (1Hz)
  const uint32_t nowMs = millis();
  if (force || (nowMs - g_lastStaPollMs >= 1000)) {
    g_lastStaPollMs = nowMs;
    g_ledStaCount = getApStaCount();
  }

  // staCount==0이면 WS도 존재할 수 없으므로 즉시 0으로 강등(v1.3과 동일한 안전 로직)
  uint8_t wsNow = (hasControlClient && controlClient.available()) ? 1 : 0;
  if (g_ledStaCount == 0) wsNow = 0;

  if (force || wsNow != g_ledWs) {
    g_ledWs = wsNow;
    Serial1.print("@ws,");
    Serial1.println((int)g_ledWs);
  }
  static uint16_t lastSentSta = 0xFFFF;
  if (force || g_ledStaCount != lastSentSta) {
    lastSentSta = g_ledStaCount;
    Serial1.print("@sta,");
    Serial1.println((int)lastSentSta);
  }
}

// -----------------------------
// SPI framing (S3 -> RTL)
// -----------------------------
#pragma pack(push, 1)
struct DcmSpiHdr {
  char magic[4];        // "DCM2"
  uint8_t type;         // 0=idle, 1=jpeg
  uint8_t flags;        // bit0=start, bit1=end
  uint16_t seq;         // frame seq
  uint32_t total_len;   // JPEG total length
  uint32_t offset;      // offset of this payload in JPEG
  uint16_t payload_len; // bytes following header in this block
  uint16_t reserved;
};
#pragma pack(pop)

static constexpr uint8_t SPI_TYPE_IDLE = 0;
static constexpr uint8_t SPI_TYPE_JPEG = 1;
static constexpr uint8_t SPI_FLAG_START = 0x01;
static constexpr uint8_t SPI_FLAG_END   = 0x02;

static uint8_t g_spiRx[SPI_BLOCK_BYTES];

// 최신 프레임 캐시(WS 송출용). SPI task가 업데이트하고, WS task가 읽음.
static uint8_t g_frameBuf[64 * 1024]; // 64KB cap (QVGA JPEG 권장)
static volatile size_t  g_frameLen = 0;
static volatile uint16_t g_frameSeq = 0;

#if RTL_HAS_FREERTOS
static SemaphoreHandle_t g_frameMutex = nullptr;
#endif

static volatile bool g_wantStream = false;
static uint32_t g_statBlocks = 0;
static uint32_t g_statFrames = 0;
static uint32_t g_statBytes = 0;

static inline bool hdrLooksOk(const DcmSpiHdr& h) {
  return (h.magic[0] == 'D' && h.magic[1] == 'C' && h.magic[2] == 'M' && h.magic[3] == '2');
}

static bool spiReadBlock(uint8_t* dst, size_t len) {
  if (!dst || len == 0) return false;

  // AmebaD SPI API는 보드별로 구현이 조금 다를 수 있어,
  // "in-place transfer" 방식으로 최대 호환을 노린다.
  memset(dst, 0, len);

  // AmebaD SPI는 transfer(pin, void*, count, mode)를 제공하며 내부에서 write+read 스트림을 돌립니다.
  // byte 단위 루프보다 훨씬 빠릅니다.
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  SPI.transfer((byte)SPI_SS_PIN, dst, len, SPI_LAST);
  SPI.endTransaction();
  return true;
}

static bool pullOneJpegFrameOverSpi(uint32_t timeoutMs) {
  const uint32_t start = millis();

  bool haveHeader = false;
  uint16_t curSeq = 0;
  size_t expectedTotal = 0;
  size_t received = 0;

  while (millis() - start < timeoutMs) {
    if (!spiReadBlock(g_spiRx, SPI_BLOCK_BYTES)) {
      delay(1);
      continue;
    }
    g_statBlocks++;

    if (SPI_BLOCK_BYTES < sizeof(DcmSpiHdr)) continue;
    const DcmSpiHdr* hdr = (const DcmSpiHdr*)g_spiRx;
    if (!hdrLooksOk(*hdr)) {
      // 동기 깨짐 -> 다음 블록
      continue;
    }
    if (hdr->type == SPI_TYPE_IDLE) {
      // 아직 프레임 없음
      delay(2);
      continue;
    }
    if (hdr->type != SPI_TYPE_JPEG) {
      continue;
    }

    if (!haveHeader) {
      if ((hdr->flags & SPI_FLAG_START) == 0 || hdr->offset != 0) {
        continue;
      }
      curSeq = hdr->seq;
      expectedTotal = hdr->total_len;
      received = 0;

      if (expectedTotal == 0 || expectedTotal > sizeof(g_frameBuf)) {
        // 너무 큼 -> 드랍
        haveHeader = false;
        expectedTotal = 0;
        received = 0;
        continue;
      }
      haveHeader = true;
    }

    // 같은 프레임인지 확인
    if (!haveHeader || hdr->seq != curSeq) {
      haveHeader = false;
      continue;
    }
    if (hdr->offset != received) {
      // 순서 깨짐 -> 드랍
      haveHeader = false;
      continue;
    }

    const size_t pl = hdr->payload_len;
    const size_t headerSz = sizeof(DcmSpiHdr);
    if (pl > SPI_BLOCK_BYTES - headerSz) { haveHeader = false; continue; }
    if (hdr->offset + pl > expectedTotal) { haveHeader = false; continue; }

    memcpy(g_frameBuf + hdr->offset, g_spiRx + headerSz, pl);
    received += pl;
    g_statBytes += (uint32_t)pl;

    const bool isEnd = (hdr->flags & SPI_FLAG_END) != 0;
    if (isEnd && received == expectedTotal) {
      // 최신 프레임 캐시 업데이트
#if RTL_HAS_FREERTOS
      if (RTL_USE_TASKS && g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
#endif
      g_frameLen = expectedTotal;
      g_frameSeq = curSeq;
#if RTL_HAS_FREERTOS
      if (RTL_USE_TASKS && g_frameMutex) xSemaphoreGive(g_frameMutex);
#endif
      g_statFrames++;
      return true;
    }
  }

  return false;
}

static void attachControlCallbacks(WebsocketsClient& c) {
  c.onMessage([&](WebsocketsClient& client, WebsocketsMessage msg) {
    if (!msg.isText()) return;
    const String s = msg.data();
    // 그대로 S3로 전달
    Serial1.println(s);
  });

  c.onEvent([&](WebsocketsClient& client, WebsocketsEvent ev, WSInterfaceString data) {
    (void)data;
    if (ev == WebsocketsEvent::ConnectionClosed) {
      hasControlClient = false;
      sendSafetyStopToS3();
    }
  });
}

#if ENABLE_CAMERA_BRIDGE
static void attachCameraCallbacks(WebsocketsClient& c) {
  c.onEvent([&](WebsocketsClient& client, WebsocketsEvent ev, WSInterfaceString data) {
    (void)data;
    if (ev == WebsocketsEvent::ConnectionClosed) {
      hasCameraClient = false;
    }
  });
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[RTL] DeepCoRTL_Bridge boot");
  Serial.printf("[RTL] Build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("[RTL] Compile detect: RTL_HAS_FREERTOS=%d, defaultTasks=%d\n",
                (int)RTL_HAS_FREERTOS, (int)RTL_USE_TASKS);
  Serial.printf("[RTL] Serial1 pins (fixed by variant): TX=%d(D4) RX=%d(D5)\n",
                RTL_LINK_TX_PIN, RTL_LINK_RX_PIN);
  Serial.printf("[RTL] SPI pins (fixed by variant): SS=%d(D9) SCLK=%d(D10) MISO=%d(D11) MOSI=%d(D12)\n",
                RTL_SPI_SS_PIN_DOC, RTL_SPI_SCLK_PIN_DOC, RTL_SPI_MISO_PIN_DOC, RTL_SPI_MOSI_PIN_DOC);

  // UART to S3
  Serial1.begin(LINK_BAUD);

  // SPI master init (camera)
  SPI.begin();
  pinMode(SPI_SS_PIN, OUTPUT);
  digitalWrite(SPI_SS_PIN, HIGH);

#if RTL_HAS_FREERTOS
  if (RTL_USE_TASKS) {
    g_frameMutex = xSemaphoreCreateMutex();
    if (!g_frameMutex) {
      Serial.println("[RTL] WARN: frame mutex create failed, falling back to loop() mode");
      // 강제 폴백 (컴파일 타임 const라서 런타임 변경은 불가) -> mutex 없는 상태로도 동작은 하지만
      // 데이터 레이스 가능성이 있어, 아래에서는 mutex 존재 여부를 체크하고 사용합니다.
    }
  }
#endif

  // Wi-Fi AP start
  WiFi.disablePowerSave();
  WiFi.config(AP_IP, AP_DNS, AP_GW, AP_SN);  // AP 모드에도 적용됨(문서)

  char ssid[4 + 8 + 1] = {0};
  buildSsidFromMac(ssid, sizeof(ssid));
  Serial.print("[RTL] AP SSID = ");
  Serial.println(ssid);

  int status = WiFi.apbegin(ssid, AP_PASSWORD, AP_CHANNEL);
  if (status != WL_CONNECTED) {
    // AmebaD는 apbegin() 리턴이 status 형태(문서상 "status of AP")
    Serial.print("[RTL] apbegin() status = ");
    Serial.println(status);
  }

  Serial.print("[RTL] AP IP = ");
  Serial.println(WiFi.localIP());
  // Initial LED link state for S3
  sendLedLinkStateToS3(true);

  // WebSocket servers
  wsControl.listen(80);
  Serial.println("[RTL] WS control listening on :80 (/ws accepted)");

#if ENABLE_CAMERA_BRIDGE
  wsCamera.listen(81);
  Serial.println("[RTL] WS camera listening on :81 (/ accepted)");
#else
  Serial.println("[RTL] Camera bridge disabled (ENABLE_CAMERA_BRIDGE=0)");
#endif

#if RTL_HAS_FREERTOS
  if (RTL_USE_TASKS) {
    Serial.println("[RTL] Mode: RTOS tasks (will start on first loop)");
  } else {
    Serial.println("[RTL] Mode: loop() fallback (RTOS headers not found)");
  }
#else
  Serial.println("[RTL] Mode: loop() fallback (no RTOS headers)");
#endif
}

static void acceptControlIfAny() {
  // poll()이 true면 accept 가능성이 있음
  if (!wsControl.poll()) return;
  if (!wsControl.available()) return;

  WebsocketsClient c = wsControl.accept();
  if (!c.available()) return;

  // single-client 정책: 기존 연결 종료
  if (hasControlClient && controlClient.available()) {
    controlClient.close();
  }

  controlClient = c;
  hasControlClient = true;
  attachControlCallbacks(controlClient);
  Serial.println("[RTL] control WS client connected");
  sendLedLinkStateToS3(true);

  // 안전: 연결되자마자 stop 한번
  sendSafetyStopToS3();
}

static void pollControlClient() {
  if (!hasControlClient) return;
  if (!controlClient.available()) {
    hasControlClient = false;
    sendSafetyStopToS3();
    sendLedLinkStateToS3(true);
    return;
  }
  controlClient.poll(); // ping/pong 포함 처리 + 메시지 콜백 호출
  sendLedLinkStateToS3(false);
}

#if ENABLE_CAMERA_BRIDGE
static void acceptCameraIfAny() {
  if (!wsCamera.poll()) return;
  if (!wsCamera.available()) return;

  WebsocketsClient c = wsCamera.accept();
  if (!c.available()) return;

  if (hasCameraClient && cameraClient.available()) {
    cameraClient.close();
  }

  cameraClient = c;
  hasCameraClient = true;
  attachCameraCallbacks(cameraClient);
  Serial.println("[RTL] camera WS client connected");
  sendLedLinkStateToS3(true);
}

static void pollCameraClient() {
  if (!hasCameraClient) return;
  if (!cameraClient.available()) {
    hasCameraClient = false;
    sendLedLinkStateToS3(true);
    return;
  }
  cameraClient.poll();
  sendLedLinkStateToS3(false);
}
#endif

// S3 -> RTL 텍스트(선택): 디버그/상태를 WS로 되돌리고 싶으면 사용
static void pumpS3TextToWs() {
  // 현재는 "줄 단위" 텍스트만 처리
  static String line;
  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (line.length() > 0) {
        // 로그 전용 채널: S3가 "@log,"로 보내면 RTL USB 콘솔에 출력
        if (line.startsWith("@log,")) {
          Serial.print("[S3] ");
          Serial.println(line.substring(5));
        } else {
          // 그 외 텍스트는 (선택) WS control로 echo
          if (hasControlClient && controlClient.available()) {
            controlClient.send(line.c_str());
          }
        }
        line = "";
      }
      continue;
    }
    // binary 혼입 방지용 제한
    if ((uint8_t)ch >= 0x20 || ch == '\t') {
      if (line.length() < 256) line += ch;
    }
  }
}

void loop() {
#if RTL_HAS_FREERTOS
  // 첫 loop()에서 task 생성(Arduino core 호환성 확보)
  ensureTasksStartedOnce();
#endif

  // RTOS task 모드: 실제 작업은 task에서 수행, loop()는 유휴
#if RTL_HAS_FREERTOS
  if (g_tasksEnabledRuntime) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }
#endif

  // loop() 폴백 모드(기존 동작 유지)
  acceptControlIfAny();
  pollControlClient();

#if ENABLE_CAMERA_BRIDGE
  acceptCameraIfAny();
  pollCameraClient();
  g_wantStream = (hasCameraClient && cameraClient.available());

  // 카메라: SPI로 프레임을 "항상" 받아서 최신 프레임 유지
  static uint32_t lastPullMs = 0;
  const uint32_t nowMs = millis();
  const uint32_t pullIntervalMs = g_wantStream ? 0 : 200; // no-client: 5fps 정도로만 유지

  if (g_wantStream || (nowMs - lastPullMs >= pullIntervalMs)) {
    lastPullMs = nowMs;
    const uint16_t beforeSeq = g_frameSeq;
    if (pullOneJpegFrameOverSpi(g_wantStream ? 120 : 20)) {
      if (g_wantStream && beforeSeq != g_frameSeq) {
        // DeepCoConnector는 WS Binary message 1개 = JPEG 1장 전제를 갖고 있음
        (void)cameraClient.sendBinary((const char*)g_frameBuf, (size_t)g_frameLen);
      }
      // wantStream=false면: 최신 프레임만 캐시해두고 송출은 안 함
    }
  }

  // 성능 로그(2초마다): QVGA 30fps 튜닝용
  static uint32_t lastStatMs = 0;
  if (nowMs - lastStatMs >= 2000) {
    lastStatMs = nowMs;
    const float secs = 2.0f;
    const float fps = (float)g_statFrames / secs;
    const float kbps = ((float)g_statBytes / 1024.0f) / secs;
    Serial.printf("[RTL][SPI] fps=%.1f blocks=%lu KB/s=%.1f (SPI_HZ=%lu)\n",
                  fps, (unsigned long)g_statBlocks, kbps, (unsigned long)SPI_HZ);
    g_statBlocks = 0;
    g_statFrames = 0;
    g_statBytes = 0;
  }
#endif

  // S3 로그/상태 수신
  pumpS3TextToWs();
}

#if RTL_HAS_FREERTOS
// -----------------------------
// RTOS tasks (preferred)
// -----------------------------
static void taskWsUart(void* arg) {
  (void)arg;
  uint16_t lastSentSeq = 0;
  uint32_t lastStatMs = millis();
  uint32_t lastBeatMs = millis();

  for (;;) {
    acceptControlIfAny();
    pollControlClient();

#if ENABLE_CAMERA_BRIDGE
    acceptCameraIfAny();
    pollCameraClient();
    g_wantStream = (hasCameraClient && cameraClient.available());

    // 최신 프레임이 갱신되면 송출
    if (g_wantStream) {
      uint16_t curSeq = 0;
      size_t curLen = 0;
      if (g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
      curSeq = g_frameSeq;
      curLen = (size_t)g_frameLen;

      if (curLen > 0 && curSeq != 0 && curSeq != lastSentSeq && cameraClient.available()) {
        // mutex를 잡은 상태에서 송출(안전성 우선)
        (void)cameraClient.sendBinary((const char*)g_frameBuf, curLen);
        lastSentSeq = curSeq;
      }
      if (g_frameMutex) xSemaphoreGive(g_frameMutex);
    }

    // 성능 로그(2초마다): SPI task가 stat을 올리고, 출력만 여기서
    const uint32_t nowMs = millis();
    if (nowMs - lastStatMs >= 2000) {
      lastStatMs = nowMs;
      const float secs = 2.0f;
      const float fps = (float)g_statFrames / secs;
      const float kbps = ((float)g_statBytes / 1024.0f) / secs;
      Serial.printf("[RTL][SPI] fps=%.1f blocks=%lu KB/s=%.1f (SPI_HZ=%lu)\n",
                    fps, (unsigned long)g_statBlocks, kbps, (unsigned long)SPI_HZ);
      g_statBlocks = 0;
      g_statFrames = 0;
      g_statBytes = 0;
    }
#endif

    pumpS3TextToWs();

    // 태스크 동작 확인용 하트비트(5초마다)
    const uint32_t nowBeat = millis();
    if (nowBeat - lastBeatMs >= 5000) {
      lastBeatMs = nowBeat;
      Serial.printf("[RTL][TASK] ws_uart alive (wantStream=%d)\n", (int)g_wantStream);
    }

    // WS/UART은 응답성이 중요: 짧게 양보
    vTaskDelay(1);
  }
}

static void taskSpiPull(void* arg) {
  (void)arg;
  uint32_t lastPullMs = 0;
  uint32_t lastBeatMs = millis();

  for (;;) {
#if ENABLE_CAMERA_BRIDGE
    const uint32_t nowMs = millis();
    const uint32_t pullIntervalMs = g_wantStream ? 0 : 200; // no-client: 5fps 정도

    if (g_wantStream || (nowMs - lastPullMs >= pullIntervalMs)) {
      lastPullMs = nowMs;
      (void)pullOneJpegFrameOverSpi(g_wantStream ? 120 : 20);
    } else {
      vTaskDelay(5);
    }

    // 태스크 동작 확인용 하트비트(5초마다)
    const uint32_t nowBeat = millis();
    if (nowBeat - lastBeatMs >= 5000) {
      lastBeatMs = nowBeat;
      Serial.printf("[RTL][TASK] spi_pull alive (seq=%u len=%u)\n",
                    (unsigned)g_frameSeq, (unsigned)g_frameLen);
    }
#else
    vTaskDelay(50);
#endif
  }
}

// Arduino core에 따라 setup() 이후 자동으로 task를 만들 수 있게,
// 여기서는 "첫 loop()"에서 한번만 생성하는 방식으로 안전하게 동작시킴.
static void ensureTasksStartedOnce() {
  static bool started = false;
  if (started) return;
  started = true;

  if (!RTL_USE_TASKS) return;

  // WS/UART task는 상대적으로 높은 우선순위
  BaseType_t okA = xTaskCreate(taskWsUart, "ws_uart", 4096, NULL, 2, NULL);
  // SPI pull task는 낮은 우선순위 (제어/통신이 우선)
  BaseType_t okB = xTaskCreate(taskSpiPull, "spi_pull", 4096, NULL, 1, NULL);

  if (okA == pdPASS && okB == pdPASS) {
    g_tasksStartedRuntime = true;
    Serial.println("[RTL] Runtime: tasks started OK (ws_uart + spi_pull)");
  } else {
    g_tasksEnabledRuntime = false;
    Serial.printf("[RTL] Runtime: task create FAILED (ws_uart=%d spi_pull=%d) -> falling back to loop()\n",
                  (int)okA, (int)okB);
  }
}
#endif

