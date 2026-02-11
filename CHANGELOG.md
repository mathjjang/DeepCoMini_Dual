# DeepCoMini_Dual — 변경 이력 (CHANGELOG)

> 듀얼 칩 로봇 제어 시스템 (ESP32-S3 + RTL8720DN)
> 모든 버전은 누적 구조이며, 이전 버전의 기능을 포함합니다.

---

## [v0.1.5] — 2026-02-11 — 옵션 E (RTL 하이브리드) 패스스루 플래시 구현

### 기능
- **S3 패스스루 플래시 모드** (`config.h`, `DeepCoRTL_Bridge.ino`)
  - 옵션 E (RTL 하이브리드) 구현: USB-C → RTL, RTL이 S3 플래시 대행
  - `@passthru` 시리얼 명령으로 패스스루 모드 진입
  - RTL이 GPIO 2개로 S3의 BOOT/EN 핀 제어 → 부트모드 진입
  - USB↔UART 투명 브릿지 루프: PC의 esptool이 RTL을 경유하여 S3 직접 플래시
  - 무통신 타임아웃(10초)으로 자동 정상 모드 복귀
  - 완료 후 S3 자동 정상 부트 리셋
  - `CFG_PASSTHRU_ENABLE = 1` (기본 활성화)
  - `@info` 명령에 패스스루 GPIO 핀 정보 표시 추가

### 문서
- **듀얼칩설계전략및펌웨어업그레이드방법.md** 업데이트
  - 옵션 E (RTL 하이브리드) 섹션 추가 (1.6)
  - 종합 비교표에 E 열 추가 + "단일 실패점" 행 추가 (1.7)
  - 옵션 E 펌웨어 업그레이드 방법 섹션 추가 (2.5)
  - 추천 설계를 D/E 상황별 비교로 확장 (3.1)
  - 보드 설계 요구사항을 공통/옵션별로 분리 (3.4)

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 패스스루 GPIO 핀/타임아웃/보드레이트 설정 추가 (+18줄)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — 패스스루 모드 전체 구현 (~130줄)
  - `enterS3BootMode()` / `resetS3Normal()` / `runPassthruBridge()` / `startPassthruMode()`
  - `@passthru` 명령 핸들러 추가
  - setup()에 GPIO 초기화 추가
- `듀얼칩설계전략및펌웨어업그레이드방법.md` — 옵션 E 전체 추가 (+110줄)

---

## [v0.1.4] — 2026-02-09 — 교실 다수 로봇 채널 분산 + 설계 문서

### 기능
- **MAC 기반 Wi-Fi 채널 자동 분산** (`config.h`, `DeepCoRTL_Bridge.ino`)
  - 교실에서 다수 로봇 사용 시 같은 채널 간섭 방지
  - MAC 마지막 바이트 % 8 → 5GHz 비중첩 8채널(36,40,44,48,149,153,157,161)에 자동 분산
  - `CFG_AP_CHANNEL_AUTO = 1` (기본 활성화)
  - `@set,channel,N`으로 수동 지정 시 자동 분산 무시
  - `@info` 명령에 `auto/MAC` / `fixed` 표시 추가

### 문서
- **듀얼칩설계전략및펌웨어업그레이드방법.md** 신규 작성
  - USB-C 연결 전략 4가지 비교 (USB→RTL / USB→S3 / USB→Hub / USB→S3 Hybrid)
  - 펌웨어 업그레이드 누적 단계 구조 정리 (단계1: USB패스스루 → 단계2: Wi-Fi OTA → 단계3: SPI 고속 OTA)
  - 추천 설계: Option D (USB-C → S3 Hybrid, S3가 GPIO로 RTL 부트/리셋 제어)
  - OTA 파티션 비교: RTL(SDK 기본 내장) vs S3(partitions.csv 수정 필요)
  - RTL/S3 플래시 레이아웃 다이어그램 추가

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 자동 채널 분산 설정 추가 (+13줄)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — `pickChannelFromMac()` 함수 추가, setup() 채널 선택 로직 변경 (+32줄)
- `듀얼칩설계전략및펌웨어업그레이드방법.md` — 신규 (443줄)
- `ROADMAP.md` — v0.1.1~v0.1.3 완료 상태 갱신

---

## [v0.1.3] — 2026-02-09 — 디버깅 및 진단 개선

### 기능
- **USB Serial 디버그 런타임 토글** (`DeepCoS3_Robot.ino`)
  - S3: `@debug,0` / `@debug,1` 명령으로 디버그 출력 ON/OFF
  - RTL: `@s3debug,0` / `@s3debug,1` → UART로 S3에 전달
  - USB Serial은 항상 초기화 상태 유지 (토글 즉시 반영)

- **종합 진단 명령 `@diag`** (`DeepCoRTL_Bridge.ino`)
  - 업타임, AP 채널, STA 수, WS 연결 상태 등 시스템 전체 상태
  - 프레임 시퀀스, 크기, SPI 속도, 버퍼 크기 포함
  - JSON 형식으로 USB 콘솔 + WS 제어 클라이언트에 동시 전달

- **연결 시간 추적** (`DeepCoS3_Robot.ino`)
  - WS 연결 시점, STA 연결 시점 기록
  - `@diag` 응답에 연결 경과 시간 포함

- **부팅 시 진단 요약 출력** (`DeepCoS3_Robot.ino`)
  - 카메라/SPI/디버그 상태를 부팅 직후 콘솔에 출력

### 변경 파일
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — 디버그 토글 + 부팅 진단 (+37줄)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — `@diag`, `@s3debug` 명령 추가 (+43줄)

---

## [v0.1.2] — 2026-02-09 — 통신 안정성 강화

### 기능
- **SPI 동기화 손실 복구 강화** (`DeepCoRTL_Bridge.ino`)
  - 연속 매직 불일치 5회 시 CS 토글로 SPI 라인 리셋
  - 시퀀스 깨짐 시 즉시 새 프레임 탐색 (3회 리트라이)
  - sync_err / seq_err / oversize 에러 카운터 추가

- **프레임 버퍼 확대** (`config.h`)
  - 64KB → 128KB (`CFG_FRAME_BUF_SIZE`)
  - VGA 해상도 JPEG도 안정적으로 수용

- **에러 상태 전파 시스템** (`DeepCoS3_Robot.ino`, `DeepCoRTL_Bridge.ino`)
  - S3: 카메라/SPI 초기화 실패 시 `@err,CAM_INIT,...` / `@err,SPI_INIT,...` 전송
  - RTL: `@err,` 프리픽스 감지 → USB 콘솔 + WS 제어 클라이언트로 중계
  - 성능 로그에 에러 통계 포함

### 변경 파일
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — 에러 전파 메시지 추가 (+4줄)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — SPI 재동기화 + 에러 카운터 + 에러 중계 (+65줄)
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — `CFG_FRAME_BUF_SIZE` 128KB로 변경

---

## [v0.1.1] — 2026-02-09 — 설정 분리 및 유연성 개선

### 기능
- **설정 헤더 파일 분리** (`config.h` 신규)
  - ESP32-S3 `config.h`: 카메라 핀, LED 핀, 모터 핀, UART, SPI, 로봇 물리 파라미터
  - RTL8720DN `config.h`: Wi-Fi AP 설정, SPI, WS 포트, 성능 로그 간격
  - 기존 `.ino` 파일 내 하드코딩 값을 모두 `config.h` 매크로로 대체

- **Wi-Fi 런타임 설정 명령** (`DeepCoRTL_Bridge.ino`)
  - `@set,channel,149` — Wi-Fi 채널 변경 (재부팅 후 적용)
  - `@set,password,xxxx` — AP 비밀번호 변경 (재부팅 후 적용)
  - `@reboot` — RTL 소프트웨어 리셋
  - `@info` — 현재 설정 출력 (채널, 비밀번호, IP, SPI 속도 등)

### 변경 파일
- `esp32s3/DeepCoS3_Robot/config.h` — 신규 (93줄)
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 신규 (73줄)
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — config.h 매크로 사용으로 리팩토링 (±144줄)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — config.h + 시리얼 명령 추가 (±165줄)

---

## [v0.1.0] — 2026-02-09 — 초기 듀얼 칩 구현

### 기능
- **ESP32-S3 (로봇 제어)**
  - 스테퍼 모터 제어 (FastAccelStepper, 차동 구동)
  - OV2640 카메라 캡처 + SPI Slave로 RTL에 전달
  - NeoPixel LED 상태 표시 (BOOTING → READY → WIFI_CONNECTED → CONNECTED)
  - UART로 RTL 제어 명령 수신 및 처리

- **RTL8720DN (네트워크 브릿지)**
  - 5GHz Wi-Fi SoftAP (MAC 기반 SSID: `DCM-XXXXXXXX`)
  - WebSocket 제어 서버 (`ws://192.168.4.1/ws`, port 80)
  - WebSocket 카메라 서버 (`ws://192.168.4.1:81/`, port 81)
  - SPI Master로 S3에서 JPEG 프레임 수신 → WS 카메라 클라이언트 송출
  - WS 연결 해제 시 S3에 자동 정지 명령 전달 (안전 메커니즘)

- **SPI 프로토콜 (`DCM2`)**
  - 매직 바이트 `0x44 0x43 0x4D 0x32` ("DCM2")
  - 타입(0x01=카메라), 시퀀스 번호, 페이로드 길이, START/END 플래그
  - 블록 크기: 2048 바이트

- **DeepCoConnector 호환**
  - 기존 v1.3 로봇 제어 프로토콜 유지
  - 고정 IP: 192.168.4.1

- **FreeRTOS 멀티태스크 아키텍처**
  - SPI 카메라 수신 태스크 (우선순위 3)
  - WS 제어/카메라 태스크 (우선순위 2)
  - LED 상태 업데이트 태스크 (우선순위 1)

### 문서
- `README_Arduino_빌드방법.md` — 빌드 가이드
- `dual_chip개발 전략.md` — 개발 전략 문서
- `.gitignore` — Arduino IDE 빌드 아티팩트 제외

### 파일 구조
```
DeepCo_Dual/
├── esp32s3/DeepCoS3_Robot/
│   └── DeepCoS3_Robot.ino          (로봇 제어 + 카메라 + SPI Slave)
├── rtl8720dn/DeepCoRTL_Bridge/
│   └── DeepCoRTL_Bridge.ino        (Wi-Fi AP + WS 서버 + SPI Master)
├── README_Arduino_빌드방법.md
├── dual_chip개발 전략.md
└── .gitignore
```
