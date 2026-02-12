# DeepCoMini_Dual — 변경 이력 (CHANGELOG)

> 듀얼 칩 로봇 제어 시스템 (ESP32-S3 + RTL8720DN)
> 모든 버전은 누적 구조이며, 이전 버전의 기능을 포함합니다.

---

## [v0.2.1] — 2026-02-12 — 카메라 FPS 안정화 (19 FPS 평균, 0 FPS 제거)

### 배경
- v0.2.0에서 카메라 스트리밍 FPS가 22~0 사이를 진동하며 불안정
- 5개 원인 분석 후 전수 수정: S3 race condition, RTL 대기 낭비, 단일 버퍼, SPI 폴링 한계, 타임아웃 과다

### S3: 뮤텍스 추가 (Fix #1 — Critical)
- `g_frameMutex` 도입: `g_snapReady`, `g_fb`, `g_fbOff`, `g_frameSeq` 공유 변수 보호
- 듀얼코어 race condition (double-free, 가비지 포인터) 근본 제거
- `fillTxBlock()`: 논블로킹 뮤텍스 (`xSemaphoreTake(mutex, 0)`) — 실패 시 IDLE 반환
- `loop()` 캡처: 블로킹 `esp_camera_fb_get()`은 뮤텍스 밖에서 호출

### RTL: IDLE 대기 제거 (Fix #2 — High)
- `pullStreamFrame()`, `pullSnapFrame()`: IDLE/bad magic 시 `vTaskDelay(1)` → `taskYIELD()`
- 프레임 사이 대기 30-50ms → 수ms로 감소

### RTL: 더블 버퍼링 (Fix #3 — High)
- `g_frameBufA[6KB]` + `g_frameBufB[6KB]` 교대 사용 (포인터 스왑)
- `taskWsUart`가 `sendBIN` 전에 버퍼 교체 → SPI pull이 sendBIN 중에도 계속 동작
- 스왑을 `g_frameReady=false` 보다 먼저 수행하여 race-free 보장
- `CFG_FRAME_BUF_SIZE` 8KB→6KB: BD_RAM_NS 여유 확보 (2×6KB=12KB)

### RTL: pullStreamFrame 개선 (Fix #5 — Medium)
- 타임아웃 500ms → 100ms: 실패 시 0 FPS 구간 대폭 축소
- seq 불일치 시 현재 블록이 새 프레임 START이면 즉시 재시작 (1프레임 낭비 제거)

### S3: SPI 폴링 타임아웃 축소 + 큐 깊이 증가 (Fix #4 — Medium)
- `spi_slave_get_trans_result` 타임아웃: 10ms → 1ms
- `CFG_SPI_QUEUE_SIZE`: 2 → 3 (RTL 고속 SPI 클럭 대응)

### RTL: FIFO-Batch SPI Read
- `spiReadBlock()` 재구현: AmebaD 64-entry TX/RX FIFO 직접 레지스터 접근
- 바이트 단위 2048회 → ~32회 배치 전송 (파이프라인 방식)
- `spi_api.h` + `extern spi_t spi_obj0` 활용

### 성능 결과
| 지표 | v0.2.0 | v0.2.1 |
|------|--------|--------|
| FPS 범위 | 22~0 (진동) | 13~22 (안정) |
| 평균 FPS | ~10 | **~19** |
| stream_fail | 수십~수백 | **1** (시작 시 1회) |
| 0 FPS 구간 | 반복 발생 | **없음** |
| 프레임 드롭 | 빈번 | **0** |

---

## [v0.2.0] — 2026-02-12 — Snap/Stream 카메라 프로토콜 (UART-free SPI 스트리밍)

### 배경
- v0.1.x의 연속 SPI 폴링 방식은 S3의 `esp_camera_fb_get()` 블로킹, SPI 큐 언더런, 프레임 동기화 문제로 불안정
- 카메라 SPI 통신을 전면 재설계하여 두 가지 모드(snap/stream)로 분리
- AmebaD(RTL8720DN)의 `SPI.transfer()` 버퍼 일괄 전송이 수신 데이터를 올바르게 저장하지 못하는 버그 발견 및 우회
- WebSocket 라이브러리의 멀티스레드 비안전 문제를 producer-consumer 패턴으로 해결

### Snap 모드 (수동/디버그용)
- RTL이 UART로 `@snap` 전송 → S3가 프레임 캡처 → `@snap,ok,<len>` UART 응답 → RTL이 SPI pull
- 1회성 요청-응답 구조, USB 시리얼에서 수동 테스트 가능
- 실패 사유 전파: S3가 `@snap,fail,<reason>` 응답 (no_camera, capture, busy)
- RTL `doSnapCycle()`: UART 타임아웃 + SPI pull 타임아웃 관리

### Stream 모드 (웹페이지 카메라 스트리밍)
- RTL이 카메라 WS 클라이언트 접속 시 `@stream,start` 전송 → S3가 연속 캡처 모드 진입
- **프레임당 UART 통신 없음** — S3가 `loop()`에서 자율적으로 `esp_camera_fb_get()` → SPI TX 큐잉
- RTL `pullStreamFrame()`: SPI 헤더의 `total_len`/`START`/`END`/`seq` 필드만으로 프레임 조립
- WS 클라이언트 해제 시 자동 `@stream,stop` → S3 연속 캡처 중단
- UART 충돌 문제 완전 제거: `taskSpiPull`은 UART를 전혀 사용하지 않음

### AmebaD SPI 바이트 단위 전송
- `SPI.transfer(pin, buf, len, mode)` 버퍼 일괄 전송이 RX 데이터를 올바르게 저장하지 못하는 문제 발견
- `spiReadBlock()`: 수동 `digitalWrite(CS)` + 바이트별 `SPI.transfer(0x00)` 루프로 우회
- 10MHz에서 안정적 CRC 통과 확인 (7000바이트 패턴 검증)

### Producer-Consumer WebSocket 패턴
- 기존: `taskSpiPull`에서 `wsCamera.sendBIN()` 직접 호출 → 멀티스레드 충돌로 프리징
- 변경: `taskSpiPull`(producer)이 `g_frameReady = true` 설정 → `taskWsUart`(consumer)가 `sendBIN()` 수행
- 모든 WebSocket 호출(`sendBIN`, `loop`)이 단일 태스크(`taskWsUart`)에서만 실행

### S3 펌웨어 간소화
- v0.1.10의 3단계 로그 시스템, LED 제어, 워치독 등 복잡한 로직 제거
- 카메라 캡처 + SPI 서빙 핵심 기능만 유지
- `fillTxBlock()`: 절대 블로킹 없음 (카메라 캡처는 `loop()` 또는 `handleSnap()`에서만)
- `spiServiceTask`: SPI 트랜잭션 완료 대기 → TX 블록 리필 → 재큐잉

### SPI 프로토콜 (dcm_spi_protocol.h)
- `SPI_FLAG_IDLE_CAM_NOT_READY`, `SPI_FLAG_IDLE_FB_FAIL` 플래그 제거 (snap/stream에서 불필요)
- CRC16-CCITT 검증 유지
- `DcmSpiHdr`: magic("DCM2"), type, flags, seq, total_len, offset, payload_len, crc16 (20바이트)

### UART 명령 체계

| 명령 | 방향 | 용도 |
|------|------|------|
| `@snap` | RTL→S3 | 1회 프레임 캡처 요청 |
| `@snap,ok,<len>` | S3→RTL | 캡처 성공 + 프레임 크기 |
| `@snap,fail,<reason>` | S3→RTL | 캡처 실패 + 사유 |
| `@stream,start` | RTL→S3 | 연속 캡처 모드 시작 |
| `@stream,stop` | RTL→S3 | 연속 캡처 모드 중지 |
| `@stream,ok` | S3→RTL | 스트림 시작/중지 확인 |
| `@stream,fail,<reason>` | S3→RTL | 스트림 시작 실패 |

### 변경 파일
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — 전면 재작성: snap/stream 핸들러, 자율 캡처 loop, 간소화
- `esp32s3/DeepCoS3_Robot/config.h` — JPEG 퀄리티 조정, 버전 0.2.0
- `esp32s3/DeepCoS3_Robot/dcm_spi_protocol.h` — 불필요 플래그 제거
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — pullSnapFrame/pullStreamFrame, doSnapCycle, producer-consumer, 바이트 단위 SPI
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 버전 0.2.0
- `rtl8720dn/DeepCoRTL_Bridge/dcm_spi_protocol.h` — 불필요 플래그 제거
- `upload_test/mini_dual_firmware_upload.html` — 버전 푸터 갱신

---

## [v0.1.10] — 2026-02-12 — 3단계 로그 레벨 시스템 (LOG_NONE / LOG_INFO / LOG_DEBUG)

### 배경
- 기존 로그는 ON/OFF 토글만 가능 → 개발/시연/배포 상황에 맞는 세분화된 출력 제어 필요
- 듀얼 칩(S3 + RTL) 양쪽에 통일된 로그 레벨 아키텍처 적용
- SPI 통계, heartbeat 등 반복 로그가 시리얼 모니터를 압도하는 문제 해결

### 로그 레벨 정의 (공통)
- **LOG_NONE (0)** — 출력 완전 차단 (배포/생산)
- **LOG_INFO (1)** — 핵심 이벤트만 (부팅, 연결, OTA, 에러) [기본값]
- **LOG_DEBUG (2)** — 전부 출력 (SPI 상세, 모터, 프레임, 워치독, heartbeat 등)

### S3 (DeepCoS3_Robot) 변경
- **`config.h`**: `CFG_ENABLE_USB_SERIAL_DEBUG` 제거 → `CFG_LOG_LEVEL` + `CFG_LOG_USB` 도입
- **`logInfo()` / `logDebug()`** 함수 신규 — 레벨 필터링 후 USB Serial + Serial1(UART) 동시 출력
- **`linkLogf` → `#define linkLogf logInfo`** 하위 호환 매크로 유지
- **`@debug,0/1/2`** 런타임 명령으로 3단계 변경 (기존 0/1 토글 확장)
- 부팅 메시지에 버전 + 로그 레벨 표시: `DeepCoS3_Robot boot (v0.1.10, log=1)`
- 부팅 진단에 로그 레벨/USB 상태 표시: `Boot diag: cam=OK spi=OK log=INFO(1) usb=ON`
- **INFO 분류**: 부팅, OTA 전체, Camera/SPI init 실패, mutex 실패, 태스크 시작
- **DEBUG 분류**: SPI 핀 상세, JSON 파싱 에러, UART overflow, 워치독 timeout

### RTL (DeepCoRTL_Bridge) 변경
- **`config.h`**: `LOG_NONE/INFO/DEBUG` 상수 + `CFG_LOG_LEVEL` 추가
- **`RTL_INFO()` / `RTL_DEBUG()`** 매크로 신규 — `g_logLevel` 기반 필터링
- **`@debug,0/1/2`** USB 시리얼 명령 추가 (RTL 자체 로그 레벨 변경)
- **`@s3debug,0/1/2`** 3단계 지원 (기존 0/1에서 확장)
- 부팅 메시지에 버전 + 로그 레벨 표시: `DeepCoRTL_Bridge boot (v0.1.10, log=INFO)`
- 도움말에 `@debug,0|1|2` 추가
- **INFO 분류**: 부팅, AP/WS 연결, OTA 전체, Passthru 모드, 에러
- **DEBUG 분류**: SPI fps/통계, Task heartbeat, malloc 경고, 핀 매핑, 자동채널 상세

### 런타임 명령 요약

| 명령 | 대상 | 효과 |
|------|------|------|
| `@debug,0` | RTL | RTL 로그 OFF |
| `@debug,1` | RTL | RTL 핵심 로그만 |
| `@debug,2` | RTL | RTL 전부 출력 |
| `@s3debug,0` | RTL→S3 | S3 로그 OFF |
| `@s3debug,1` | RTL→S3 | S3 핵심 로그만 |
| `@s3debug,2` | RTL→S3 | S3 전부 출력 |

### 변경 파일
- `esp32s3/DeepCoS3_Robot/config.h` — 로그 레벨 상수 + CFG_LOG_LEVEL/CFG_LOG_USB 추가
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — logInfo/logDebug 함수, @debug 3단계, 로그 분류
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 로그 레벨 상수 + CFG_LOG_LEVEL 추가
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — RTL_INFO/RTL_DEBUG 매크로, @debug 명령, 로그 분류

---

## [v0.1.9] — 2026-02-11 — WebSocket OTA (RTL 자체 + S3 SPI OTA)

### 배경
- Wi-Fi OTA로 펌웨어 업데이트를 가능하게 하여, USB 직접 연결 없이 무선 업그레이드 지원
- RTL8720DN: AmebaD Flash API를 사용한 듀얼 OTA 파티션 직접 기록 방식
- ESP32-S3: RTL이 WebSocket으로 수신한 바이너리를 SPI로 S3에 전달, S3가 `Update.h`로 자체 기록
- 초기에는 S3 OTA를 UART로 계획했으나, SPI(30MHz)가 UART(115200bps) 대비 ~16배 빠른 속도 이점으로 SPI 방식 채택

### RTL 자체 OTA (DeepCoRTL_Bridge.ino)
- **Flash API 직접 사용** (`flash_api.h`, `sys_api.h`)
  - `flash_read_word`, `flash_erase_sector`, `flash_stream_write`, `flash_write_word`
  - 듀얼 OTA 주소: `OTA_ADDR_1 = 0x006000`, `OTA_ADDR_2 = 0x106000` (최대 1MB)
  - 부트로더 서명 검증: `0x35393138` / `0x31313738`
- **OTA 상태 관리** (`g_ota` 구조체)
  - `otaChooseAddress()`: 현재 부팅 이미지 감지 → 반대쪽에 기록
  - `otaEraseFlash()`: 이미지 크기만큼 섹터 삭제
  - `otaWriteChunk()`: WS binary [offset(4BE)+len(4BE)+payload] 파싱 → Flash 기록 + 체크섬 누적
  - `otaEnd()`: 크기/체크섬 검증 → 서명 기록 → 기존 이미지 무효화 → 2초 후 `sys_reset()`
  - `otaCancel()`: 새 이미지 무효화 + 기존 복원

### S3 SPI OTA (RTL→S3)
- **RTL 측** (`DeepCoRTL_Bridge.ino`)
  - `g_otaS3` 상태 구조체 + `spiOtaTransfer()`, `spiCheckS3Ack()`, `spiSendOtaBlock()`
  - `otaS3Begin()`: 안전 정지 → 카메라 OFF → SPI START 블록 전송
  - `otaS3WriteChunk()`: WS binary → SPI 블록 크기로 분할 → SPI_TYPE_OTA 전송
  - `otaS3End()`: END 블록 전송 → S3 최종 ACK 확인
  - `otaS3Cancel()`: ERR 플래그 END 전송
- **S3 측** (`DeepCoS3_Robot.ino`)
  - `#include <Update.h>` 추가
  - `g_s3Ota` 상태 구조체
  - `handleSpiOtaBlock()`: SPI RX에서 OTA 데이터 감지 → `Update.begin/write/end` 호출
  - `fillOtaAckBlock()`: SPI TX에 ACK 응답 적재 (에러 플래그 포함)
  - `spiSlavePoll()`: `SPI_TYPE_OTA` 감지 시 OTA 핸들러 호출
  - OTA 성공 후 2초 지연 `ESP.restart()`

### SPI 프로토콜 확장 (dcm_spi_protocol.h)
- `SPI_TYPE_OTA = 2` — OTA 데이터 블록
- `SPI_TYPE_OTA_ACK = 3` — S3 응답 블록
- `SPI_FLAG_OTA_ERR = 0x04` — OTA 에러 플래그

### WebSocket OTA 프로토콜
- `@ota,start,rtl,<size>` → RTL 자체 OTA 시작
- `@ota,start,s3,<size>` → S3 SPI OTA 시작
- `@ota,end,<size>` → OTA 완료 (대상 자동 판별)
- `@ota,cancel` → OTA 취소
- WS binary: `[offset(4BE) + length(4BE) + payload]` 청크 포맷

### OTA 테스트 UI
- **`upload_test/mini_dual_firmware_upload.html`** 신규
  - RTL/S3 대상 선택, 파일 업로드, 진행률 표시
  - WebSocket 전용 (HTTP 제거)
- **`upload_test/OTA_TASKS.md`** 신규 — OTA 구현 단계 체크리스트

### 설계 문서 업데이트
- **`듀얼칩설계전략및펌웨어업그레이드방법.md`** 수정
  - Section 2.5: S3 펌웨어 업그레이드 → SPI OTA 방식으로 갱신
  - Section 3.2: Phase 2 + Phase 3 통합 (UART OTA 생략, SPI OTA 직행)
  - ASCII 다이어그램 및 구현 노트 갱신

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — RTL OTA + S3 SPI OTA 전체 구현 (+500줄)
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 버전 0.1.9
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — SPI OTA 수신기 + Update.h 연동 (+150줄)
- `esp32s3/DeepCoS3_Robot/config.h` — 버전 0.1.9
- `shared/dcm_spi_protocol.h` — OTA 타입/플래그 상수 추가
- **신규**: `upload_test/mini_dual_firmware_upload.html` — OTA 테스트 UI
- **신규**: `upload_test/OTA_TASKS.md` — OTA 구현 체크리스트
- `듀얼칩설계전략및펌웨어업그레이드방법.md` — SPI OTA 반영

---

## [v0.1.8] — 2026-02-11 — SPI CRC16 무결성 + S3 안전성 강화 + 코드 품질 개선

### 배경
- v0.1.7 코드 리뷰에서 데이터 무결성, 메모리 안정성, 안전성 관련 개선점 도출
- RTL/S3 양쪽에 걸쳐 9건의 개선 사항 적용
- SPI 프로토콜 공유 헤더 분리로 양 칩 간 구조체 동기화 보장

### SPI 프로토콜 무결성 (RTL + S3)
- **CRC16-CCITT 검증 추가** — SPI 데이터 무결성 보장
  - `DcmSpiHdr.crc16` 필드 추가 (기존 `reserved` 필드 대체)
  - S3 `fillTxBlock()`: 매 블록마다 `crc16Ccitt(payload)` 계산 후 헤더에 삽입
  - RTL `pullOneJpegFrameOverSpi()`: 수신 시 CRC 검증, 불일치 시 `g_statCrcErrors++`
  - `@diag` JSON에 `crc_err` 필드 추가
  - SPI 성능 로그에 `crc_err` 카운트 추가
- **`dcm_spi_protocol.h` 공유 헤더 신규** — 프로토콜 동기화 보장
  - `DcmSpiHdr` 구조체, `SPI_TYPE_*`, `SPI_FLAG_*` 상수, `crc16Ccitt()` 함수
  - RTL/S3 양쪽 `.ino`에서 인라인 정의 제거 → `#include "dcm_spi_protocol.h"`
  - 양 칩의 구조체 크기·필드 순서 불일치 위험 근본 제거
- **`static_assert(SPI_BLOCK_BYTES >= sizeof(DcmSpiHdr))`** — 컴파일 타임 검증 (RTL)

### S3 안전성 강화 (DeepCoS3_Robot.ino)
- **모터 명령 워치독 (3초)** — UART 단절 시 안전 정지
  - `CMD_WATCHDOG_MS = 3000`: 3초간 모터 명령 없으면 `stopRobot()` 자동 호출
  - `@ws,*`, `@sta,*` 등 상태 메시지는 워치독 리셋하지 않음 (모터 명령만 리셋)
  - 한 번 트리거 시 `linkLogf()` 경고 1회 출력 (로그 홍수 방지)
- **`move`/`angle` 명령 클램핑** — 비정상 큰 값 방지
  - `MAX_MOVE_STEP_CMD = 50000`, `MAX_ANGLE_CMD = 3600`
- **프레임 버퍼 mutex 보호** — `fillTxBlock()` + `@cam,` 명령 동시 접근 안전
  - `g_frameMutex` (xSemaphoreCreateMutex) 추가
  - `fillTxBlock()`: do/while(0) + mutex 래핑
  - `@cam,0` 명령: mutex 하에 `g_camEnabled`/`g_fb` 조작
- **LED 플래시 spinlock** — 멀티태스크 LED 상태 안전
  - `portMUX_TYPE g_ledMux` 추가
  - `startSideFlash()` / `ledUpdateOnce()`: `portENTER/EXIT_CRITICAL` 보호
- **SPI 초기화 실패 시 안전 폴백**
  - `taskSpiCamera` / `loop()`: `g_spiInitOk` 체크 후 `vTaskDelay(5)` 폴백

### S3 메모리·코드 품질 (DeepCoS3_Robot.ino)
- **UART 수신: String → char[] 전환** — 힙 파편화 근본 제거
  - `readLineFromSerial1(char*, size_t)`: static char buf[512] + 오버플로우 감지
  - `handleCommandLine(const char*)`: `strncmp`/`strchr` 토큰 파싱
  - `#include <sstream>` 삭제 — `std::istringstream`/`std::string` 완전 제거
- **`robotMoveJson(const char*)` 시그니처 변경** — `std::string` 힙 할당 제거

### RTL 안정성·운영 개선 (DeepCoRTL_Bridge.ino)
- **WS 텍스트 전달 길이 제한** — UART 블로킹 방지
  - `kMaxFwd = 256`: 15KB WS 메시지가 UART를 1.3초 블록하는 문제 해결
- **패스스루 진입 전 안전 정지** — S3 리부트 전 모터 정지 보장
  - `startPassthruMode()`: `sendSafetyStopToS3()` + `delay(50)` 추가
- **Wi-Fi 채널 화이트리스트 검증** — 잘못된 채널 설정 방지
  - `isValidChannel()`: 2.4GHz(1~13) + 5GHz(36/40/44/48/149/153/157/161/165)
  - 기존 `ch > 0 && ch < 200` → 유효 채널만 허용
- **설정 변경 리부트 알림** — `g_needReboot` 활용
  - `warnRebootIfNeeded()`: 10초 주기로 "[RTL] WARN: settings changed" 콘솔 출력
  - `loop()` + `taskWsUart` 양쪽에서 호출
- **malloc 실패 로깅** — `taskWsUart`
  - `mallocFailCount` 카운터 + 5초 주기 경고 로그
- **frameSeq 0 래핑 수정** — `curSeq != 0` 조건 제거 → 매 36분 1프레임 누락 해결
- **빈 섹션 주석 제거** — "WebSocket servers" 잔존 주석 정리

### 변경 파일
- **신규**: `rtl8720dn/DeepCoRTL_Bridge/dcm_spi_protocol.h` (37줄) — SPI 프로토콜 공유 헤더
- **신규**: `esp32s3/DeepCoS3_Robot/dcm_spi_protocol.h` (37줄) — 동일 복사본
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — CRC16, 채널검증, 패스스루안전, 리부트알림, WS제한, malloc로그 (+99줄 변경)
- `esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` — 워치독, char[]전환, mutex, spinlock, CRC16, 클램핑 (+246/-156줄 변경)

---

## [v0.1.7] — 2026-02-11 — WebSocket 라이브러리 교체 (arduinoWebSockets)

### 배경
- 커스텀 MiniWS.h는 최소 기능만 구현한 프로토타입이었으나, 검증된 오픈소스로 교체하여 안정성 확보
- Links2004/arduinoWebSockets가 `NETWORK_AMEBAD` (RTL8720DN)를 네이티브 지원
- 프로젝트 로컬 복사본(`src/dc_ws/`)으로 글로벌 라이브러리 패치 불필요

### WebSocket 아키텍처 변경
- **MiniWS.h (커스텀) → arduinoWebSockets (Links2004)** 교체
  - `MiniWsServer` / `MiniWsClient` 제거 → `WebSocketsServer` 이벤트 기반 전환
  - `src/dc_ws/` 프로젝트 로컬 복사본: WebSockets.h/cpp, WebSocketsServer.h/cpp, libb64, libsha1
  - `NETWORK_AMEBAD` 자동 감지 (`ARDUINO_AMEBA` / `ARDUINO_ARCH_AMEBAD` / `CONFIG_PLATFORM_8721D`)
- **이벤트 콜백 패턴**
  - `onControlWsEvent(num, type, payload, length)` — WStype_CONNECTED / DISCONNECTED / TEXT 처리
  - `onCameraWsEvent(num, type, payload, length)` — WStype_CONNECTED / DISCONNECTED 처리
  - 경로 검증: control은 `/ws`만 허용, camera는 `/` 또는 빈 경로만 허용 (불일치 시 즉시 disconnect)
- **클라이언트 번호 추적**
  - `g_controlClientNum` / `g_cameraClientNum` (uint8_t, 0xFF = 미연결)
  - `controlClientConnected()` / `cameraClientConnected()` 인라인 헬퍼 함수
  - `clientIsConnected(num)` 기반 3중 체크 (flag + num + 라이브러리 확인)
- **단일 클라이언트 정책**: 새 연결 시 기존 클라이언트 자동 disconnect 후 교체
- **연결 시 안전 정지**: control 연결 직후 `sendSafetyStopToS3()` 자동 호출

### RTOS 프레임 전송 개선
- **Mutex 점유 시간 대폭 단축** (`taskWsUart`)
  - Before: mutex 잡은 상태로 `sendBinary()` (네트워크 전송 대기 포함)
  - After: mutex 구간에서 `malloc + memcpy`만 수행 → mutex 해제 → `sendBIN()` → `free()`
  - SPI 태스크가 mutex 대기하는 시간 최소화 → 프레임 수집 연속성 향상

### AP STA 카운트 실제 API 사용
- **`wifi_get_associated_client_list()`** (Realtek wifi_conf.h)
  - `extern "C" { #include "wifi_conf.h" }` 추가
  - `rtw_maclist_t` 기반 실제 연결 클라이언트 수 조회
  - 기존 추정치 대신 AP 드라이버 직접 조회 → LED 상태 정확도 향상

### 정리
- **MiniWS.h 삭제**: 더 이상 사용하지 않음
- **빈 함수 제거**: `acceptControlIfAny()` / `acceptCameraIfAny()` 불필요 (arduinoWebSockets가 `loop()` 내에서 자동 처리)
- **forward declaration 추가**: `sendLedLinkStateToS3(bool force)` — 함수 순서 의존성 해소

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/MiniWS.h` — **삭제**
- `rtl8720dn/DeepCoRTL_Bridge/src/dc_ws/` — **신규**: arduinoWebSockets 프로젝트 로컬 복사
  - `WebSockets.h/cpp`, `WebSocketsServer.h/cpp`, `WebSocketsVersion.h`
  - `libb64/cencode.c`, `libb64/cencode_inc.h`
  - `libsha1/libsha1.h`
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — WebSocket 전면 교체
  - MiniWS include → `src/dc_ws/WebSocketsServer.h`
  - 이벤트 콜백 2개 신규 (`onControlWsEvent`, `onCameraWsEvent`)
  - 클라이언트 번호 추적 + 헬퍼 함수
  - RTOS taskWsUart: malloc+copy 패턴으로 mutex 최적화
  - `extern "C" wifi_conf.h` + `getApStaCount()` 실제 API

---

## [v0.1.6] — 2026-02-11 — 동시성 안전 + 메모리 안정성 + 프로토콜 강화

### 배경
- RTL8720DN 펌웨어 코드 리뷰에서 High 4건 / Medium 4건 / Low 1건의 개선점 도출
- RTOS 멀티태스크 환경에서의 레이스 컨디션, String 힙 파편화, WS 프로토콜 검증 부족 등을 체계적으로 수정

### High (안정성)
- **RTL_PRINTF 전역 버퍼 동시 접근 제거** (`DeepCoRTL_Bridge.ino`)
  - `static char _rtl_pf[256]` 전역 버퍼 → 매크로 내 `char _pf[192]` 스택 로컬 버퍼
  - taskWsUart / taskSpiPull 동시 호출 시 로그 문자열 깨짐 방지

- **프레임 버퍼 쓰기/읽기 동기화 범위 확장** (`DeepCoRTL_Bridge.ino`)
  - `memcpy(g_frameBuf + offset, ...)` 구간까지 g_frameMutex 보호 확장
  - 기존: g_frameLen/g_frameSeq만 mutex 보호 → 개선: memcpy + 메타데이터 갱신 원자적 보호
  - WS sendBinary 중 SPI 쓰기가 프레임을 덮어쓰는 레이스 제거

- **MiniWS 헤더 라인 무제한 누적 방지** (`MiniWS.h`)
  - `_readLine()`: String → char buf[256] 고정 버퍼, 상한 초과 시 handshake 거부
  - 악성/오류 클라이언트의 무한 헤더로 인한 힙 압박 방지

- **String 중심 처리 → char[] 전환** (`MiniWS.h`, `DeepCoRTL_Bridge.ino`)
  - WS handshake(`_doHandshake`): String → char[] 기반 헤더 파싱
  - WS text frame(`_readFrame`): char[] 버퍼로 수신 후 콜백 시에만 최소 String 생성
  - USB 시리얼 수신(`pollUsbSerialCommands`): `static String usbLine` → `static char usbBuf[128]`
  - S3→RTL 텍스트 수신(`pumpS3TextToWs`): `static String line` → `static char s3Buf[256]`
  - RTL8720DN 제한 메모리에서 장시간 운영 시 힙 파편화 위험 대폭 감소

### Medium (안정성·보안)
- **WS handshake 프로토콜 검증 강화** (`MiniWS.h`)
  - `Upgrade: websocket` + `Connection: Upgrade` 필수 헤더 검증 추가 (case-insensitive)
  - 기존: Sec-WebSocket-Key만 확인 → 개선: 3가지 필수 헤더 모두 검증
  - 비정상 HTTP 요청 수용 방지

- **통계 변수 동시 접근 보호** (`DeepCoRTL_Bridge.ino`)
  - `SNAPSHOT_AND_RESET_STATS` 매크로: taskENTER_CRITICAL 임계구역에서 원자적 복사+리셋
  - 기존: 읽기와 리셋 사이에 다른 태스크가 값 변경 가능 → 개선: 6개 변수 일괄 스냅샷
  - stat 변수에 volatile 추가

- **상태 플래그 volatile 일관성 확보** (`DeepCoRTL_Bridge.ino`)
  - `hasControlClient`, `hasCameraClient`, `g_needReboot` → volatile 추가
  - 태스크 간 가시성 보장 + 접근 규칙 주석 문서화

- **패스스루 모드 강제 탈출 경로 추가** (`DeepCoRTL_Bridge.ino`)
  - Ctrl+C(0x03) x 3회를 500ms 이내 입력 시 즉시 탈출
  - 기존: 무통신 타임아웃(10초)만 → 개선: 사용자가 즉시 빠져나올 수 있음

### Low (유지보수성)
- **명령 파싱 하드코딩 인덱스 제거** (`DeepCoRTL_Bridge.ino`)
  - `handleSerialCommand(const String&)` → `handleSerialCommand(const char*)`
  - `cmd.c_str() + 13`, `cmd.substring(14)` 등 매직 인덱스 → `strchr()` 구분자 탐색 + `strncmp()` 토큰 매칭
  - `@set,<key>,<value>` 명령의 key/value 분리를 구분자 기반으로 변경
  - 새로운 @set 서브명령 추가 시 유지보수 용이

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/MiniWS.h` — v2 전면 개선 (String→char[], 헤더검증, 라인상한)
  - `_readLine()`: char buf + 길이 상한
  - `_doHandshake()`: char[] 기반 + Upgrade/Connection 검증
  - `_readFrame()` text: char[] 수신 후 최소 String 콜백
  - case-insensitive 헬퍼 함수 추가 (`_ws_lower`, `_ws_prefixEq`, `_ws_containsEq`, `_ws_trimEnd`)
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — 동시성/메모리/파싱 전면 개선
  - RTL_PRINTF 매크로: 스택 로컬 버퍼
  - 프레임 memcpy mutex 확장
  - SNAPSHOT_AND_RESET_STATS 매크로 (Arduino IDE 전처리기 호환)
  - volatile 플래그 일관성
  - 패스스루 Ctrl+C x3 탈출
  - handleSerialCommand/pollUsbSerialCommands/pumpS3TextToWs char[] 전환

---

## [v0.1.5] — 2026-02-11 — 옵션 E 패스스루 + BW16 타겟 컴파일 수정

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

### BW16 (RTL8720DN) 타겟 컴파일 수정
- **핀 매핑 수정** (`config.h`, `DeepCoRTL_Bridge.ino`)
  - BW16 variant.h에 맞게 `D0`~`D12` → `AMB_D0`~`AMB_D12` 전면 변경
  - config.h 핀 정의와 주석 일괄 수정 (PA26→AMB_D8, PA25→AMB_D7 등)
- **MiniWS.h 신규 작성** — 커스텀 WebSocket 서버/클라이언트
  - 외부 라이브러리(WebSockets2_Generic) 비호환 → Ameba 내장 WiFiServer/WiFiClient만 사용
  - SHA-1, Base64 자체 구현 (mbedtls 링크 불필요)
  - Text/Binary 메시지, Ping/Pong, Close 프레임 처리
- **RTL_PRINTF 매크로 도입** (`DeepCoRTL_Bridge.ino`)
  - `LOGUARTClass`/`UARTClassTwo`에 `printf()` 없음 → snprintf+print 에뮬레이션
- **IPAddress::toString() 미지원 우회** — 수동 IP 포맷팅
- **프레임 버퍼 128KB → 8KB** (`config.h`)
  - RTL8720DN RAM 한계로 인한 링커 오버플로우(BD_RAM_NS) 해결
- **`#undef min` / `#undef max`** — Ameba SDK 매크로 충돌 해결
- **`hdrLooksOk` → 매크로 전환** — Arduino IDE 전처리기 자동 프로토타입 이슈 우회
- **partitions.csv 한글 주석 → 영문** — gen_esp32part.py UnicodeDecodeError 해결

### 문서
- **듀얼칩설계전략및펌웨어업그레이드방법.md** 업데이트
  - 옵션 E (RTL 하이브리드) 섹션 추가 (1.6)
  - 종합 비교표에 E 열 추가 + "단일 실패점" 행 추가 (1.7)
  - 옵션 E 펌웨어 업그레이드 방법 섹션 추가 (2.5)
  - 추천 설계를 D/E 상황별 비교로 확장 (3.1)
  - 보드 설계 요구사항을 공통/옵션별로 분리 (3.4)

### 변경 파일
- `rtl8720dn/DeepCoRTL_Bridge/MiniWS.h` — **신규**: 커스텀 WebSocket 구현 (445줄)
- `rtl8720dn/DeepCoRTL_Bridge/config.h` — 핀 매핑 수정 + 패스스루 설정 + 프레임 버퍼 축소
- `rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino` — MiniWS 통합 + 핀 매핑 + RTL_PRINTF + 패스스루
- `esp32s3/DeepCoS3_Robot/partitions.csv` — 한글 주석 → 영문
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
