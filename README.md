# DeepCo_Dual

ESP32-S3 + RTL8720DN 듀얼 칩 기반 자율주행 AI 학습용 로봇 플랫폼 펌웨어.

## 개요

DeepCo_Dual은 **ESP32-S3**(로봇 제어 + 카메라)와 **RTL8720DN**(5GHz Wi-Fi 네트워크 브릿지)을 조합한 듀얼 칩 아키텍처의 교육용 로봇입니다. RTL8720DN이 5GHz SoftAP + WebSocket 서버를 담당하고, ESP32-S3가 모터 구동, 카메라 캡처, LED 상태 표시를 담당합니다. 두 칩은 UART(제어 명령)와 SPI(카메라 데이터/OTA)로 통신합니다.

5GHz Wi-Fi를 사용하므로 2.4GHz 대비 간섭이 적어, 교실/전시장 등 다수 로봇 동시 운용 환경에 적합합니다.

## 하드웨어

### 칩 구성

| 칩 | 역할 | 주요 기능 |
|----|------|----------|
| **ESP32-S3** | 로봇 제어 | 모터, 카메라, LED, SPI Slave |
| **RTL8720DN** (BW16) | 네트워크 브릿지 | 5GHz SoftAP, WebSocket 서버, SPI Master |

### ESP32-S3 핀 맵

| 기능 | GPIO |
|------|------|
| Left STEP / DIR / EN | 21 / 42 / 41 |
| Right STEP / DIR / EN | 40 / 39 / 38 |
| UART TX / RX (→ RTL) | 43 / 44 |
| SPI CS / MISO / SCLK / MOSI | 34 / 35 / 36 / 37 |
| LED Center | 2 |
| LED Left / Right NeoPixel | 48 / 47 |
| Camera (ESP32S3-EYE) | XCLK:15, SDA:4, SCL:5, VSYNC:6, HREF:7, PCLK:13 |

### RTL8720DN (BW16) 핀 맵

| 기능 | Pin |
|------|-----|
| USB 디버그 콘솔 (Serial) | D0(PA7) / D1(PA8) |
| UART TX / RX (→ S3) | D4(PB1) / D5(PB2) |
| SPI SS / SCLK / MISO / MOSI | D9(PA15) / D10(PA14) / D11(PA13) / D12(PA12) |

### 배선 요약

```
         RTL8720DN (BW16)                ESP32-S3
        ┌──────────────┐              ┌──────────────┐
        │              │              │              │
  GND ──┤ GND      GND ├──────────── ┤ GND          │
        │              │              │              │
  UART  │ D4  (TX) ────┼──────────── ┤ GPIO44 (RX)  │
        │ D5  (RX) ────┼──────────── ┤ GPIO43 (TX)  │
        │              │              │              │
  SPI   │ D9  (SS) ────┼──────────── ┤ GPIO34 (CS)  │
        │ D10 (SCLK)───┼──────────── ┤ GPIO36 (SCLK)│
        │ D12 (MOSI)───┼──────────── ┤ GPIO37 (MOSI)│
        │ D11 (MISO)───┼──────────── ┤ GPIO35 (MISO)│
        │              │              │              │
        └──────────────┘              └──────────────┘
           5GHz WiFi AP                Camera + Motor
```

## 시스템 아키텍처

```
┌─────────── RTL8720DN ───────────┐      ┌──────── ESP32-S3 ────────┐
│                                 │      │                          │
│  Port 80: WS Robot Control (/ws)│ UART │  Motor Control           │
│  Port 81: WS Camera Stream     │◄────►│  (FastAccelStepper)      │
│                                 │      │                          │
│  SoftAP: DCM-XXXXXXXX (5GHz)   │ SPI  │  Camera (OV2640/OV5640)  │
│  OTA: RTL自体 + S3 SPI OTA     │◄────►│  JPEG → SPI Slave 송출   │
│                                 │      │                          │
│  LED 상태 동기 (UART)           │      │  NeoPixel LED 시스템     │
└─────────────────────────────────┘      └──────────────────────────┘
          │         │
      ┌───┘         └────┐
      ▼                  ▼
  DeepCoConnector    브라우저
  (앱/PC 제어)    (카메라 뷰어)
```

### 통신 프로토콜

| 인터페이스 | 속도 | 용도 |
|-----------|------|------|
| UART | 115200 bps | 로봇 제어 명령 전달 (RTL → S3), 상태 리포트 (S3 → RTL) |
| SPI | 10MHz+ | 카메라 JPEG 프레임 전송 (S3 → RTL), OTA 펌웨어 전달 (RTL → S3) |

### SPI 프로토콜 (DCM2)

| 필드 | 크기 | 설명 |
|------|------|------|
| Magic | 4B | `0x44 0x43 0x4D 0x32` ("DCM2") |
| Type | 1B | 0x01=Camera, 0x02=OTA, 0x03=OTA_ACK |
| Flags | 1B | START, END, IDLE, OTA_ERR |
| Seq | 2B | 프레임 시퀀스 번호 |
| Total Len | 4B | 전체 JPEG 크기 |
| Offset | 4B | 현재 블록 오프셋 |
| Payload Len | 2B | 현재 블록 페이로드 크기 |
| CRC16 | 2B | CRC16-CCITT (페이로드 무결성) |
| **Payload** | ~2KB | JPEG 데이터 |

블록 크기: 2048 바이트. CRC16-CCITT로 모든 블록의 무결성을 검증합니다.

## 카메라 모드

### Snap 모드 (수동/디버그)
RTL이 UART로 `@snap` 전송 → S3가 1회 캡처 → SPI로 pull. 디버그용.

### Stream 모드 (실시간 스트리밍)
WS 카메라 클라이언트 접속 시 자동 시작. S3가 `loop()`에서 자율적으로 연속 캡처 → SPI TX 큐잉. RTL이 SPI 헤더만으로 프레임 조립 (UART 통신 없음). 평균 **19 FPS**.

## 제어 프로토콜

### 로봇 명령 (Port 80 WebSocket `/ws`, 텍스트)

| 명령 | 설명 |
|------|------|
| `forward,100` | 전진 |
| `left,100` | 좌회전 |
| `right,100` | 우회전 |
| `stop,100` | 정지 |
| `move,N` | N 스텝 직진 |
| `angle,N` | N도 회전 |
| `robot,{lspeed:N,rspeed:N}` | 좌/우 모터 직접 속도 제어 (SPS) |

### Joy 프로토콜 (Port 80 WebSocket, 텍스트)

| 명령 | 설명 |
|------|------|
| `speed,N` | 속도 저장 (-100 ~ 100, 음수=후진) |
| `joy,{x:N}` | 조이스틱 방향 (-100=좌 ~ 100=우) |

### 시리얼 명령 (RTL USB 콘솔)

| 명령 | 설명 |
|------|------|
| `@info` | 현재 설정 출력 |
| `@diag` | 시스템 진단 (JSON) |
| `@set,channel,N` | Wi-Fi 채널 변경 |
| `@set,password,xxx` | AP 비밀번호 변경 |
| `@debug,0/1/2` | RTL 로그 레벨 (NONE/INFO/DEBUG) |
| `@s3debug,0/1/2` | S3 로그 레벨 변경 |
| `@passthru` | S3 패스스루 플래시 모드 진입 |
| `@snap` | 카메라 1회 캡처 |
| `@reboot` | RTL 소프트웨어 리셋 |

## OTA 펌웨어 업데이트

WebSocket 기반 OTA로 RTL과 S3 모두 무선 업데이트 가능합니다.

### RTL OTA
- AmebaD Flash API 직접 사용 (듀얼 OTA 파티션)
- WS binary: `[offset(4BE) + length(4BE) + payload]` 청크 포맷

### S3 SPI OTA
- RTL이 WS로 수신한 바이너리를 SPI로 S3에 전달
- S3가 `Update.h`로 자체 기록
- SPI(30MHz) 전송: UART(115200bps) 대비 ~16배 속도

### OTA 명령

| 명령 | 설명 |
|------|------|
| `@ota,start,rtl,<size>` | RTL 자체 OTA 시작 |
| `@ota,start,s3,<size>` | S3 SPI OTA 시작 |
| `@ota,end,<size>` | OTA 완료 |
| `@ota,cancel` | OTA 취소 |

테스트 UI: `upload_test/mini_dual_firmware_upload.html`

## LED 상태

| 상태 | 센터 LED | 좌/우 LED |
|------|---------|-----------|
| BOOTING (3초) | 흰색 브리딩 | 파란색 프로그레스 바 |
| READY | 빨간색 브리딩 (dim) | OFF |
| WIFI_CONNECTED | 초록색 브리딩 (dim) | OFF |
| CONNECTED | 초록색 (steady) | 파란색 (steady) |

## FreeRTOS 태스크 구조

### RTL8720DN

| 태스크 | 역할 |
|--------|------|
| `taskWsUart` | WS loop, UART 수신, 프레임 WS 송출 (consumer) |
| `taskSpiPull` | SPI Master 프레임 수집 (producer) |

### ESP32-S3

| 태스크 | 코어 | 역할 |
|--------|------|------|
| `loop()` | Core 1 | 카메라 캡처, UART 명령 처리 |
| `spiServiceTask` | Core 0 | SPI Slave 트랜잭션 서빙 |
| `ledTask` | Any | NeoPixel LED 애니메이션 |

## 안전 기능

- **모터 워치독 (3초)**: UART 단절 시 자동 정지
- **WS 연결 해제 시 안전 정지**: RTL이 S3에 즉시 정지 명령 전달
- **move/angle 클램핑**: 비정상 큰 값 방지 (50000 steps / 3600도)
- **SPI CRC16 무결성 검증**: 모든 블록에 CRC16-CCITT 적용
- **패스스루 안전 정지**: S3 리부트 전 모터 정지 보장

## 빌드 환경

### ESP32-S3

| 항목 | 버전 |
|------|------|
| Arduino Core | ESP32 by Espressif (Boards Manager) |
| 보드 | ESP32S3 Dev Module |
| Partition | Custom (`esp32s3/DeepCoS3_Robot/partitions.csv`) |

#### 라이브러리

| 라이브러리 | 용도 |
|-----------|------|
| FastAccelStepper | 스테퍼 모터 제어 |
| ArduinoJson | JSON 파싱 |
| Adafruit_NeoPixel | LED 제어 |
| Update (내장) | SPI OTA 수신 |
| esp_camera (내장) | 카메라 캡처 |

### RTL8720DN (BW16)

| 항목 | 버전 |
|------|------|
| Arduino Core | Realtek AmebaD (Boards Manager URL 추가 필요) |
| 보드 | BW16 / AMBxx |

#### 라이브러리

| 라이브러리 | 용도 |
|-----------|------|
| arduinoWebSockets (Links2004) | WebSocket 서버 (프로젝트 로컬: `src/dc_ws/`) |
| flash_api / sys_api (내장) | RTL OTA Flash 기록 |

## WiFi 설정

- **모드**: SoftAP (5GHz)
- **SSID**: `DCM-XXXXXXXX` (MAC 주소 기반 자동 생성)
- **비밀번호**: `12345678` (런타임 변경 가능)
- **채널**: MAC 기반 자동 분산 (8채널 풀: 36, 40, 44, 48, 149, 153, 157, 161)
- **IP**: `192.168.4.1`

## 프로젝트 구조

```
DeepCo_Dual/
├── esp32s3/
│   ├── DeepCoS3_Robot/
│   │   ├── DeepCoS3_Robot.ino        # 로봇 제어 + 카메라 + SPI Slave
│   │   ├── config.h                  # S3 핀/파라미터 설정
│   │   ├── dcm_spi_protocol.h        # SPI 프로토콜 공유 헤더
│   │   └── partitions.csv            # OTA 파티션 테이블
│   └── partitions/                   # 4MB/8MB/16MB 파티션 프리셋
├── rtl8720dn/
│   └── DeepCoRTL_Bridge/
│       ├── DeepCoRTL_Bridge.ino      # Wi-Fi AP + WS 서버 + SPI Master + OTA
│       ├── config.h                  # RTL 핀/WiFi/SPI 설정
│       ├── dcm_spi_protocol.h        # SPI 프로토콜 공유 헤더
│       └── src/dc_ws/                # arduinoWebSockets 로컬 복사본
├── upload_test/
│   ├── mini_dual_firmware_upload.html # OTA 테스트 UI
│   └── OTA_TASKS.md
├── docs/                             # 설계 문서, 분석 자료
├── CHANGELOG.md
└── README.md
```

## 사용법

1. ESP32-S3에 `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` 업로드
2. RTL8720DN에 `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` 업로드
3. UART + SPI 배선 연결 (위 배선 요약 참조)
4. RTL 시리얼 모니터에서 SSID 확인 (예: `DCM-E31D06E8`)
5. PC/모바일에서 해당 5GHz SSID에 연결 (비밀번호: `12345678`)
6. DeepCoConnector 앱 또는 브라우저에서 `ws://192.168.4.1/ws` 접속하여 제어
7. 카메라: `ws://192.168.4.1:81/` 접속 시 자동 스트리밍 시작

## 로그 레벨

| 레벨 | 값 | 출력 내용 |
|------|---|----------|
| LOG_NONE | 0 | 출력 차단 (배포/생산) |
| LOG_INFO | 1 | 핵심 이벤트 (부팅, 연결, OTA, 에러) — **기본값** |
| LOG_DEBUG | 2 | 전부 출력 (SPI, 모터, 프레임, 워치독 등) |

## 버전 히스토리

현재 펌웨어: **v0.2.1**

상세 변경 이력은 [CHANGELOG.md](CHANGELOG.md)를 참조하세요.

| 버전 | 주요 변경 |
|------|----------|
| v0.2.1 | 카메라 FPS 안정화 (평균 19 FPS), 뮤텍스, 더블 버퍼링 |
| v0.2.0 | Snap/Stream 카메라 프로토콜, Producer-Consumer 패턴 |
| v0.1.10 | 3단계 로그 레벨 시스템 (NONE/INFO/DEBUG) |
| v0.1.9 | WebSocket OTA (RTL 자체 + S3 SPI OTA) |
| v0.1.8 | SPI CRC16 무결성, 모터 워치독, 안전성 강화 |
| v0.1.7 | WebSocket 라이브러리 교체 (arduinoWebSockets) |
| v0.1.6 | 동시성 안전, 메모리 안정성, 프로토콜 강화 |
| v0.1.5 | S3 패스스루 플래시 모드, BW16 타겟 수정, MiniWS |
| v0.1.4 | MAC 기반 Wi-Fi 채널 자동 분산, 설계 문서 |
| v0.1.3 | `@diag` 진단 명령, `@debug` 런타임 토글 |
| v0.1.2 | SPI 동기화 복구, 에러 전파 시스템 |
| v0.1.1 | config.h 분리, Wi-Fi 런타임 설정 명령 |
| v0.1.0 | 초기 듀얼 칩 구현 |
