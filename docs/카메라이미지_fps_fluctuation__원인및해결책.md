# 카메라 이미지 FPS Fluctuation (22~0 진동) — 원인 및 해결책

> 작성일: 2026-02-12  
> 대상: DeepCo_Dual v0.2.0 (RTL8720DN Master + ESP32-S3 Slave)  
> 증상: 카메라 스트리밍 FPS가 22~0 사이를 오가며 불안정. 0 FPS 구간이 반복적으로 발생.

---

## 1. 전체 파이프라인 구조

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ESP32-S3 (Slave)                              │
│                                                                         │
│  loop() 태스크 (prio 1)          spiServiceTask (prio 2)               │
│  ┌──────────────────┐           ┌──────────────────────┐               │
│  │ esp_camera_fb_get │──g_fb──→│ fillTxBlock()         │               │
│  │ g_snapReady=true  │ g_fbOff │ DcmSpiHdr + payload   │               │
│  │ (blocking capture)│←────────│ g_snapReady=false      │               │
│  └──────────────────┘  ★뮤텍스 └──────────┬───────────┘               │
│                          없음!              │ SPI Slave TX              │
└─────────────────────────────────────────────┼───────────────────────────┘
                                              │ SPI Bus (10MHz)
┌─────────────────────────────────────────────┼───────────────────────────┐
│                          RTL8720DN (Master)  │                          │
│                                              ▼                          │
│  taskSpiPull (prio 1)            taskWsUart (prio 2)                   │
│  ┌────────────────────┐         ┌──────────────────────┐               │
│  │ pullStreamFrame()  │g_frame  │ wsCamera.sendBIN()   │               │
│  │ spiReadBlock() x N │─Ready──→│ g_frameReady=false   │               │
│  │ 프레임 조립 (CRC)  │  =true  │ pollCameraClient()   │               │
│  │ g_frameReady=true  │←────────│ pumpS3TextToWs()     │               │
│  └────────────────────┘ =false  └──────────┬───────────┘               │
│                                             │ WebSocket                 │
└─────────────────────────────────────────────┼───────────────────────────┘
                                              ▼
                                      웹 브라우저 (클라이언트)
```

### 정상 흐름 (1프레임 사이클)

1. **S3 `loop()`**: `esp_camera_fb_get()` → JPEG 캡처 → `g_snapReady = true`
2. **S3 `spiServiceTask`**: RTL이 SPI 클럭하면 `fillTxBlock()`으로 JPEG 청크 전송
3. **RTL `taskSpiPull`**: `spiReadBlock()` 반복 → DcmSpiHdr 기반 프레임 조립 → `g_frameReady = true`
4. **RTL `taskWsUart`**: `wsCamera.sendBIN()` → 웹소켓으로 JPEG 전송 → `g_frameReady = false`
5. RTL SPI pull 재개 → 다음 프레임

---

## 2. 원인 분석 (심각도 순)

---

### 원인 1 (Critical): S3 Race Condition — 뮤텍스 없는 공유 변수

**ESP32-S3는 듀얼코어**. `loop()` 태스크와 `spiServiceTask`가 다른 코어에서 동시 실행 가능.

#### 공유 변수 (잠금 없이 양쪽에서 읽기/쓰기)

| 변수 | loop() (캡처) | spiServiceTask → fillTxBlock() (SPI TX) |
|------|---------------|----------------------------------------|
| `g_snapReady` | **write** (true) | **read** + **write** (false) |
| `g_fb` | **write** (새 포인터) | **read** (데이터 복사) + **write** (nullptr) |
| `g_fbOff` | **write** (0으로 리셋) | **read** + **write** (증가) |
| `g_frameSeq` | **write** (증가) | **read** |

#### fillTxBlock() — 프레임 전송 완료 시 (spiServiceTask)

```cpp
if (g_fbOff >= g_fb->len) {
    esp_camera_fb_return(g_fb);   // ① fb 반환
    g_fb = nullptr;                // ② 포인터 클리어
    g_fbOff = 0;                   // ③ 오프셋 리셋
    g_snapReady = false;           // ④ 캡처 허용 신호  ← 이것이 먼저 보일 수 있음
}
```

#### loop() — 다음 프레임 캡처 (기본 태스크)

```cpp
if (g_streaming && !g_snapReady && g_cameraOk) {
    if (g_fb) { esp_camera_fb_return(g_fb); g_fb = nullptr; }  // ★ double-free 위험!
    g_fb = esp_camera_fb_get();
    ...
    g_snapReady = true;
}
```

#### 경합 시나리오 (듀얼코어)

```
시간   Core 0 (loop)                Core 1 (spiServiceTask)
─────  ──────────────────────       ─────────────────────────
 T1                                  g_snapReady = false ← ④ 먼저 실행됨
 T2    !g_snapReady == true ✓
       g_fb != nullptr ✓            (아직 g_fb = nullptr 실행 안 됨!)
 T3    esp_camera_fb_return(g_fb)   ← 이미 ①에서 반환된 fb를 또 반환!
       ═══ DOUBLE-FREE ═══          ← 크래시 또는 힙 손상
```

**결과**:
- 힙 손상 → ESP32-S3 불안정 → SPI 응답 불능
- `fillTxBlock`이 이미 해제된 `g_fb`를 읽음 → 가비지 데이터 → CRC 실패
- RTL `pullStreamFrame`에서 연쇄 CRC/시퀀스 실패 → 500ms 타임아웃 → **FPS = 0**

---

### 원인 2 (High): RTL pullStreamFrame의 IDLE 블록마다 1ms 대기

S3가 카메라 캡처 중(`esp_camera_fb_get()` 블로킹, ~30-50ms)일 때, SPI는 **IDLE 블록만 반환**.

```cpp
// pullStreamFrame() 내부
if (hdr->type == SPI_TYPE_IDLE) {
    vTaskDelay(1);    // ← IDLE마다 1ms 대기
    continue;
}
if (!hdrLooksOk(*hdr)) {
    vTaskDelay(1);    // ← bad magic도 1ms 대기
    continue;
}
```

#### 오버헤드 계산

| 상황 | 발생 빈도 | 대기 시간 |
|------|----------|----------|
| S3 카메라 캡처 중 (30-50ms) | 매 프레임 사이 | IDLE x 30-50회 = **30-50ms** |
| S3 SPI 리필 못함 (bad magic) | RTL이 S3보다 빠를 때 | bad magic x N회 = **N ms** |
| 정상 프레임 수신 (3블록) | 프레임당 1회 | ~0.3ms (SPI 전송만) |

→ 프레임당 **실제 SPI 전송 0.3ms + 대기 30-50ms** = 효율 ~1%

정상 상태에서도 최대 ~20 FPS 정도만 가능하며, S3 race condition과 겹치면 IDLE/bad magic이 500ms 타임아웃 전체를 채움.

---

### 원인 3 (High): Producer-Consumer 단일 프레임 버퍼 + sendBIN 블로킹

RTL에서 SPI pull과 WebSocket 전송이 **`g_frameReady` 플래그 하나**로 직렬화됨:

```
taskSpiPull:  g_frameReady==true → vTaskDelay(1), SPI 정지 ❌
taskWsUart:   g_frameReady==true → sendBIN() → g_frameReady=false ✓
```

#### sendBIN 소요 시간 분석

| JPEG 크기 | WiFi 처리량 (AP 모드) | sendBIN 예상 소요 |
|-----------|---------------------|-------------------|
| 3-5 KB | ~1-2 MB/s | 3-5ms |
| 5-8 KB | ~1-2 MB/s | 5-8ms |
| TCP 혼잡 시 | 버퍼 풀 | **50-200ms+** |

TCP 전송 버퍼가 가득 차면 `sendBIN`이 블로킹됨. 이 동안:

1. `taskSpiPull` 완전 정지 (SPI 읽기 불가)
2. S3는 이미 다음 프레임 캡처 완료했지만 전달 불가
3. `sendBIN` 해소 → SPI 재개 → S3 SPI 버퍼는 이미 stale

**결과**: TCP 혼잡 발생 시 → FPS 순간 0 → 혼잡 해소 시 FPS 급등 → 반복

---

### 원인 4 (Medium): S3 SPI 서비스 태스크의 10ms 폴링 한계

```cpp
// S3 spiServiceTask
esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, pdMS_TO_TICKS(10));
if (err != ESP_OK || !done) continue;  // 10ms 타임아웃
```

S3 SPI 큐 깊이 = **2 슬롯**:
- RTL이 FIFO-batch로 빠르게 SPI 클럭 → 슬롯 0, 1 모두 소진
- S3가 아직 리필 안 됨 (10ms 대기 중) → RTL이 읽으면 **garbage (bad magic)**
- RTL `vTaskDelay(1)` → 재시도 → 또 garbage

---

### 원인 5 (Medium): 500ms 타임아웃의 누적 효과

```cpp
// RTL pullStreamFrame()
static bool pullStreamFrame(uint32_t timeoutMs) {   // timeoutMs = 500
  while (millis() - start < timeoutMs) {
    spiReadBlock(g_spiRx, SPI_BLOCK_BYTES);
    ...
  }
  return false;  // 500ms 소진 → 실패
}
```

프레임 조립 실패 시 **최대 500ms를 소모**한 뒤에야 재시도. 이것이 **0 FPS 구간**의 직접적 원인:

```
[성공] 3ms 만에 프레임 완성 → FPS ~22
[실패] 500ms 타임아웃 소모 → FPS = 0  ← 이 구간이 체감됨
[성공] 3ms 만에 프레임 완성 → FPS ~22
```

---

## 3. 22→0→22 진동 타임라인

```
시간      상태                    FPS
──────    ────────────────────    ────
  0ms     정상 스트리밍            ~22
 50ms     프레임 성공               ↑
100ms     프레임 성공               ↑
          ... (10-20회 연속 성공) ...

250ms     ★ S3 race condition 발생
          fillTxBlock이 깨진 데이터 전송
251ms     RTL: CRC 실패 → 프레임 리셋
260ms     RTL: 시퀀스 불연속 → 리셋
270ms     RTL: IDLE 블록 → vTaskDelay(1)
          ... 반복 ...
750ms     pullStreamFrame 500ms 타임아웃      → 0
          g_snapFailCount++
751ms     pullStreamFrame 재호출
755ms     S3 정상 복구 → 프레임 수신 성공     → 22
          ... 10-20회 연속 성공 후 다시 반복 ...
```

---

## 4. 해결 방안 (우선순위순)

---

### 수정 1 (Critical): S3 뮤텍스 추가

`g_snapReady`, `g_fb`, `g_fbOff` 접근을 뮤텍스로 보호.

```cpp
// 선언부
static SemaphoreHandle_t g_frameMutex = nullptr;

// setup()에서
g_frameMutex = xSemaphoreCreateMutex();

// loop() — 카메라 캡처
if (g_streaming && g_cameraOk) {
    bool needCapture = false;
    if (xSemaphoreTake(g_frameMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        needCapture = !g_snapReady;
        xSemaphoreGive(g_frameMutex);
    }
    if (needCapture) {
        camera_fb_t* fb = esp_camera_fb_get();  // 뮤텍스 밖에서 블로킹 (OK)
        if (fb && fb->buf && fb->len > 0) {
            xSemaphoreTake(g_frameMutex, portMAX_DELAY);
            if (g_fb) esp_camera_fb_return(g_fb);
            g_fb = fb;
            g_fbOff = 0;
            g_frameSeq++;
            if (g_frameSeq == 0) g_frameSeq = 1;
            g_snapReady = true;
            xSemaphoreGive(g_frameMutex);
        } else {
            if (fb) esp_camera_fb_return(fb);
        }
    }
}

// fillTxBlock() — SPI TX
if (xSemaphoreTake(g_frameMutex, 0) != pdTRUE) {
    // 잠금 실패 → IDLE 반환 (블로킹하지 않음)
    h->type = SPI_TYPE_IDLE;
    return;
}
// ... 기존 로직 (g_fb, g_fbOff 접근) ...
xSemaphoreGive(g_frameMutex);
```

**효과**: double-free, 가비지 포인터 접근 제거 → CRC 실패/시퀀스 불연속 대폭 감소

---

### 수정 2 (High): RTL IDLE 대기 시간 제거

IDLE 블록과 bad magic에서 `vTaskDelay(1)` 대신 `taskYIELD()` (또는 `vTaskDelay(0)`) 사용:

```cpp
// pullStreamFrame() 내부
if (!hdrLooksOk(*hdr)) {
    taskYIELD();    // 0ms yield — 다른 태스크에 기회만 줌
    continue;
}
if (hdr->type == SPI_TYPE_IDLE) {
    taskYIELD();    // 0ms yield
    continue;
}
```

**효과**: 프레임 사이 대기 시간 30-50ms → 수ms로 감소. 단, CPU 사용률 증가 (RTOS 양보는 유지).

---

### 수정 3 (High): RTL 더블 버퍼링

`g_frameBuf`를 2개로 두어 sendBIN 중에도 SPI pull 가능:

```cpp
static uint8_t g_frameBuf[2][CFG_FRAME_BUF_SIZE];
static volatile int g_writeBuf = 0;   // SPI가 쓰는 버퍼
static volatile int g_readBuf  = -1;  // WS가 읽는 버퍼 (-1=없음)
static volatile size_t g_frameLen[2] = {0, 0};

// taskSpiPull: g_frameBuf[g_writeBuf]에 조립
// 완성 시: g_readBuf = g_writeBuf; g_writeBuf ^= 1;

// taskWsUart: g_readBuf >= 0이면 sendBIN(g_frameBuf[g_readBuf])
// 완료 후: g_readBuf = -1;
```

**효과**: sendBIN 블로킹 중에도 SPI pull 계속 → FPS 안정화. TCP 혼잡 시에도 최신 프레임 유지.

---

### 수정 4 (Medium): S3 SPI 폴링 타임아웃 축소 + 큐 깊이 증가

```cpp
// AS-IS
esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, pdMS_TO_TICKS(10));

// TO-BE
esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, pdMS_TO_TICKS(1));
```

```cpp
// config.h
#define CFG_SPI_QUEUE_SIZE  3   // 2 → 3 (더 많은 파이프라인 여유)
```

**효과**: RTL의 고속 SPI 클럭에 S3 리필이 더 빨리 대응 → garbage 블록 감소.

---

### 수정 5 (Medium): pullStreamFrame 타임아웃 축소

```cpp
// AS-IS
if (pullStreamFrame(500)) {

// TO-BE
if (pullStreamFrame(200)) {
```

**효과**: 실패 시 0 FPS 구간이 500ms → 200ms로 축소. 평균 FPS 안정화.

---

## 5. 예상 효과 요약

| 수정 | 적용 전 | 적용 후 | 비고 |
|------|--------|--------|------|
| S3 뮤텍스 | CRC/시퀀스 실패 빈발 | 실패 거의 제거 | **가장 중요** |
| IDLE 대기 제거 | 프레임 사이 30-50ms 낭비 | 수ms로 감소 | FPS 상한 향상 |
| RTL 더블 버퍼링 | sendBIN 중 SPI 정지 | 병렬 동작 | FPS 하한 보장 |
| S3 SPI 폴링 축소 | garbage 블록 빈발 | 감소 | 안정성 향상 |
| 타임아웃 축소 | 0 FPS 구간 500ms | 200ms | 체감 개선 |

### 적용 우선순위

```
1순위: S3 뮤텍스 (원인 1)     ─── 근본 원인 제거
2순위: IDLE 대기 제거 (원인 2) ─── 즉각 FPS 개선
3순위: RTL 더블 버퍼링 (원인 3) ── 안정성 확보
4순위: S3 SPI 폴링 축소 (원인 4) ─ 보조 개선
5순위: 타임아웃 축소 (원인 5)   ── 체감 개선
```

---

## 6. 참조 코드 위치

| 항목 | 파일 | 라인 |
|------|------|------|
| S3 `loop()` 스트리밍 캡처 | `DeepCoS3_Robot.ino` | 477-498 |
| S3 `fillTxBlock()` | `DeepCoS3_Robot.ino` | 220-264 |
| S3 `spiServiceTask` | `DeepCoS3_Robot.ino` | 311-333 |
| S3 `handleStreamStart/Stop` | `DeepCoS3_Robot.ino` | 196-218 |
| RTL `taskSpiPull` | `DeepCoRTL_Bridge.ino` | 1810-1854 |
| RTL `taskWsUart` | `DeepCoRTL_Bridge.ino` | 1785-1810 |
| RTL `pullStreamFrame()` | `DeepCoRTL_Bridge.ino` | 1076-1158 |
| RTL `pollCameraClient()` | `DeepCoRTL_Bridge.ino` | 831-848 |
| RTL `onCameraWsEvent()` | `DeepCoRTL_Bridge.ino` | 1308-1355 |
| RTL `g_frameReady/g_streaming` | `DeepCoRTL_Bridge.ino` | 201-203 |
| RTL `g_frameBuf` 등 | `DeepCoRTL_Bridge.ino` | 904-918 |
