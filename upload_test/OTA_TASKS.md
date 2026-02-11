# OTA 펌웨어 업로드 테스트 — 태스크 목록

> DeepCo_Dual (RTL8720DN + ESP32-S3) Wi-Fi OTA 펌웨어 업데이트 구현

---

## 전체 구조

```
[PC 브라우저 / DeepCoConnector]
  │
  │  WebSocket ws://192.168.4.1/ws
  │
  ├─ RTL 펌웨어:  WS @ota,start,rtl → binary chunks → @ota,end → RTL Flash → 재부팅
  │
  └─ S3 펌웨어:   WS @ota,start,s3  → binary chunks → @ota,end → RTL → SPI → S3 Update.h → 재부팅
```

- 로봇 Wi-Fi AP: `DCM-XXXXXXXX` (192.168.4.1)
- **WebSocket 단일 채널 사용** (이미 port 80 WS 서버 구동 중, HTTP 서버 별도 구현 불필요)
- RTL RAM 제약상 HTTP multipart 파싱보다 WS binary 청크가 효율적
- S3는 RTL 경유로만 OTA 가능 (S3에 직접 Wi-Fi 없음)
- **v0.1.9: S3 OTA는 SPI 경유** — 기존 DcmSpiHdr 프로토콜 활용, UART 대비 ~15배 고속

---

## Phase 1: 프론트엔드 (HTML 테스트 페이지)

| # | 태스크 | 상태 | 설명 |
|---|--------|------|------|
| 1.1 | `mini_dual_firmware_upload.html` 작성 | DONE | 단일 HTML 파일 (외부 의존성 없음, 오프라인 사용 가능) |
| 1.2 | 로봇 Wi-Fi 연결 상태 표시 | DONE | WebSocket 연결 상태 LED + 텍스트 |
| 1.3 | 대상 선택 UI (RTL / S3) | DONE | 탭 버튼 방식 |
| 1.4 | .bin 파일 선택 + 유효성 검사 | DONE | 파일 크기 제한 (RTL: 1MB, S3: 1.9MB), ESP32 매직 바이트 확인 |
| 1.5 | WS binary 청크 업로드 + 진행률 | DONE | 4KB 단위 binary 전송, 속도/ETA 표시 |
| 1.6 | 업로드 결과 표시 + 재부팅 카운트다운 | DONE | 성공/실패 메시지, 10초 재부팅 대기 + 자동 재연결 |
| 1.7 | 실시간 로그 영역 | DONE | WebSocket 수신 메시지 + OTA 상태 + @diag 파싱 |

---

## Phase 2: RTL 펌웨어 — WebSocket OTA 핸들러

| # | 태스크 | 상태 | 설명 |
|---|--------|------|------|
| 2.1 | `@ota,start,rtl,<size>` 명령 처리 | DONE | WS text 수신 → OTA 모드 진입, Flash 준비, `@ota,ready` 응답 |
| 2.2 | WS binary 청크 수신 → Flash 기록 | DONE | 8-byte 헤더 [offset(4B)+length(4B)] + payload, `flash_stream_write` |
| 2.3 | `@ota,end,<total_size>` 검증 및 확정 | DONE | 크기 + 체크섬 검증 → Flash 서명 기록 → `@ota,done` 응답 |
| 2.4 | RTL OTA Flash API 호출 | DONE | AmebaD `flash_api.h`: `flash_erase_sector`, `flash_stream_write/read`, `flash_write_word` |
| 2.5 | OTA 완료 후 자동 재부팅 | DONE | `sys_reset()` (2초 딜레이 후) |
| 2.6 | 에러 처리 + `@ota,error,<reason>` | DONE | size_mismatch, checksum_mismatch, flash_write_fail, signature_fail |
| 2.7 | `@ota,cancel` 처리 | DONE | Flash 롤백 (새 이미지 무효화 + 기존 복원) + 카메라 복구 |

---

## Phase 3: S3 펌웨어 — SPI OTA (RTL 경유)

> **변경**: UART OTA → **SPI OTA** (v0.1.9)
>
> 근거:
> - SPI 4MHz full-duplex 이미 구축 (DcmSpiHdr 프로토콜, CRC16-CCITT)
> - UART 115200bps ≈ 12KB/s → 1.87MB ≈ **2분 40초**
> - SPI 4MHz ≈ 500KB/s → 1.87MB ≈ **4초** (이론), 오버헤드 포함 **~10초**
> - CRC16 무결성 검증이 SPI 프로토콜에 내장
> - UART 기반 base64 인코딩 → 33% 오버헤드 제거

| # | 태스크 | 상태 | 설명 |
|---|--------|------|------|
| 3.1 | `dcm_spi_protocol.h` OTA 타입 추가 | DONE | `SPI_TYPE_OTA=2`, `SPI_TYPE_OTA_ACK=3`, `SPI_FLAG_OTA_ERR=0x04` |
| 3.2 | RTL `@ota,start,s3,<size>` 분기 처리 | DONE | WS text → `otaS3Begin()` → SPI START 블록 → `@ota,ready` |
| 3.3 | RTL WS binary → SPI 중계 | DONE | `otaS3WriteChunk()` — 4KB WS 청크를 2KB SPI 블록으로 분할 전송 |
| 3.4 | RTL `@ota,end` / `@ota,cancel` S3 분기 | DONE | `otaS3End()` — SPI END 블록 + S3 ACK 확인, `otaS3Cancel()` |
| 3.5 | S3 SPI RX에서 OTA 블록 감지 | DONE | `spiSlavePoll()`에서 `SPI_TYPE_OTA` 감지 → `handleSpiOtaBlock()` |
| 3.6 | S3 `Update.h` 통합 | DONE | `Update.begin()` → `Update.write()` → `Update.end()` |
| 3.7 | S3 OTA ACK 응답 | DONE | TX 버퍼에 `SPI_TYPE_OTA_ACK` + received/error 정보 |
| 3.8 | S3 OTA 완료 후 자동 재부팅 | DONE | 2초 딜레이 후 `ESP.restart()` |
| 3.9 | CRC16 무결성 검증 | DONE | 각 SPI 블록의 payload CRC16 검증, 실패 시 즉시 에러 |
| 3.10 | S3 OTA 파티션 검증 | DONE | `partitions.csv`에 ota_0/ota_1 이미 구성됨 |

---

## Phase 4: 안전성 및 검증

| # | 태스크 | 상태 | 설명 |
|---|--------|------|------|
| 4.1 | OTA 중 모터 안전 정지 | DONE | RTL: `sendSafetyStopToS3()`, S3: `stopRobot()` |
| 4.2 | OTA 중 카메라 비활성화 | DONE | OTA 시작 시 `@cam,0`, 취소 시 `@cam,1` 복구 |
| 4.3 | RTL OTA 실패 시 롤백 | DONE | Flash 서명 복원 (새 이미지 무효화 + 기존 복원) |
| 4.4 | S3 OTA 실패 시 롤백 | DONE | `Update.abort()` + ESP32 자동 rollback (ota_0/ota_1) |
| 4.5 | 펌웨어 버전 확인 | DONE | `@diag` JSON에 `fw_version`, `ota_active`, `ota_s3_active` 포함 |
| 4.6 | 중복 업로드 방지 | DONE | RTL/S3 OTA 동시 진행 불가, `already_active` 에러 |
| 4.7 | SPI OTA 에러 피드백 | DONE | S3 에러 시 ACK에 `SPI_FLAG_OTA_ERR` → RTL → WS `@ota,error` |

---

## SPI OTA 프로토콜 상세

```
RTL → S3 (SPI Master TX, type=SPI_TYPE_OTA)
┌──────────────────────────────────────────────┐
│  DcmSpiHdr (20B)                             │
│  ├─ magic: "DCM2"                            │
│  ├─ type: 2 (OTA)                            │
│  ├─ flags: START=0x01, END=0x02, ERR=0x04    │
│  ├─ seq: 블록 시퀀스                          │
│  ├─ total_len: 펌웨어 전체 크기               │
│  ├─ offset: 현재 payload의 오프셋             │
│  ├─ payload_len: 페이로드 바이트 수           │
│  └─ crc16: payload CRC16-CCITT               │
├──────────────────────────────────────────────┤
│  Payload (max 2028B)                         │
│  = SPI_BLOCK_BYTES(2048) - sizeof(DcmSpiHdr) │
└──────────────────────────────────────────────┘

S3 → RTL (SPI Slave TX, type=SPI_TYPE_OTA_ACK)
┌──────────────────────────────────────────────┐
│  DcmSpiHdr (20B)                             │
│  ├─ magic: "DCM2"                            │
│  ├─ type: 3 (OTA_ACK)                        │
│  ├─ flags: 0=OK, 0x04=ERROR                  │
│  ├─ seq: 마지막 수신 블록의 seq               │
│  ├─ total_len: 누적 수신 바이트               │
│  └─ payload_len: 0                            │
└──────────────────────────────────────────────┘
```

### 전송 흐름

1. `@ota,start,s3,<size>` (WS text) → RTL `otaS3Begin()`
2. RTL → SPI START 블록 (payload=0) → S3 `Update.begin()`
3. WS binary chunks → RTL `otaS3WriteChunk()` → SPI DATA 블록들 (2KB 단위)
4. `@ota,end,<size>` (WS text) → RTL `otaS3End()` → SPI END 블록 → S3 `Update.end()`
5. S3 → RTL ACK 확인 → WS `@ota,done` → S3 `ESP.restart()`

---

## 기술 제약 사항

| 항목 | RTL8720DN | ESP32-S3 |
|------|-----------|----------|
| Flash 크기 | 2MB | 4MB |
| OTA 파티션 | SDK 기본 내장 (1MB 영역) | partitions.csv 설정 (1.875MB x 2) |
| OTA API | `flash_api.h` (AmebaD SDK) | `Update.h` (Arduino-ESP32) |
| 최대 .bin 크기 | ~1MB | ~1.875MB |
| RAM 여유 | 매우 제한 (수 KB) | 상대적 여유 (수십 KB) |
| 수신 경로 | WebSocket binary (port 80) | **SPI** (RTL 경유, DcmSpiHdr 프로토콜) |

---

## 우선순위

1. ~~**Phase 1** (프론트엔드)~~ — DONE
2. ~~**Phase 2** (RTL OTA)~~ — DONE
3. ~~**Phase 3** (S3 SPI OTA)~~ — DONE (v0.1.9)
4. **Phase 4** (안전성) — 실기기 테스트 후 최종 검증 필요
