# DeepCo_Dual 로드맵

> 듀얼 칩 로봇 제어 시스템 (ESP32-S3 + RTL8720DN)
> 이 문서는 프로젝트의 버전별 작업 현황과 개선 계획을 관리합니다.

---

## v0.1.0 — 초기 듀얼 칩 구현

### 완료 항목
- [x] ESP32-S3 / RTL8720DN 역할 분리 설계 및 구현
- [x] UART 제어 명령 브릿지 (RTL → S3)
- [x] SPI 카메라 JPEG 스트리밍 (S3 → RTL)
- [x] RTL8720DN Wi-Fi SoftAP (5GHz, ch36)
- [x] WebSocket 제어 서버 (`ws://192.168.4.1/ws`, port 80)
- [x] WebSocket 카메라 서버 (`ws://192.168.4.1:81/`, port 81)
- [x] DeepCoConnector 호환 유지 (기존 v1.3 프로토콜)
- [x] FreeRTOS 멀티태스크 아키텍처 (SPI/제어/LED 분리)
- [x] SPI 프로토콜 설계 (`DCM2` 매직, 시퀀스, START/END 플래그)
- [x] LED 상태 머신 (BOOTING → READY → WIFI_CONNECTED → CONNECTED)
- [x] RTL → S3 연결 상태 전달 (`@ws`, `@sta` 명령)
- [x] WS 연결 해제 시 자동 정지 (안전 메커니즘)
- [x] 빌드 가이드 문서 (`README_Arduino_빌드방법.md`)
- [x] 개발 전략 문서 (`dual_chip개발 전략.md`)

---

## v0.1.1 — 설정 분리 및 유연성 개선

- [x] 핀 번호/보드레이트/버퍼 크기를 설정 헤더(`config.h`)로 분리
  - ESP32-S3: 카메라핀, LED핀, 모터핀, UART, SPI, 로봇 물리 파라미터
  - RTL8720DN: Wi-Fi, SPI, WS포트, 성능 로그 간격
- [x] Wi-Fi 채널/비밀번호를 USB 시리얼 명령으로 변경 가능
  - `@set,channel,149` → 채널 변경 (재부팅 후 적용)
  - `@set,password,xxxx` → 비밀번호 변경 (재부팅 후 적용)
  - `@reboot` → RTL 소프트웨어 리셋
  - `@info` → 현재 설정 출력

---

## v0.1.2 — 통신 안정성 강화

- [x] SPI 동기화 손실 시 복구 로직 강화
  - 연속 매직 불일치(5회) 시 CS 토글로 SPI 라인 리셋
  - 시퀀스 깨짐 시 즉시 새 프레임 탐색 (3회 리트라이)
- [x] 프레임 버퍼 크기 64KB → 128KB 확대 (VGA JPEG 수용)
- [x] 에러 상태 전파 시스템 (S3 → RTL → WS → PC)
  - S3: `@err,CAM_INIT,...` / `@err,SPI_INIT,...` 전송
  - RTL: `@err,` 프리픽스 감지 시 USB콘솔 + WS 제어 클라이언트로 중계
  - 성능 로그에 sync_err / seq_err / oversize 카운트 추가

---

## v0.1.3 — 디버깅 및 진단 개선

- [x] USB Serial 디버그 런타임 토글
  - S3: `@debug,0` / `@debug,1` 명령으로 ON/OFF
  - RTL: `@s3debug,0` / `@s3debug,1` → S3에 전달
  - USB Serial은 항상 초기화 (토글 즉시 반영)
- [x] 종합 진단 명령 `@diag` 추가
  - 업타임, 채널, STA수, WS 연결 상태, 프레임 시퀀스/크기, SPI 속도, 버퍼 크기
  - JSON 형식으로 USB콘솔 + WS 클라이언트에 전달
- [x] 연결 시간 추적 (WS 연결 시점, STA 연결 시점 기록)
- [x] 부팅 시 진단 요약 출력 (cam/spi/debug 상태)

---

## v0.1.4 — 교실 다수 로봇 채널 분산

- [x] MAC 기반 Wi-Fi 채널 자동 분산 (5GHz 8채널)
- [x] 듀얼칩설계전략및펌웨어업그레이드방법.md 작성

---

## v0.1.5 — 옵션 E 패스스루 + BW16 타겟 컴파일 수정

- [x] S3 패스스루 플래시 모드 (`@passthru` 명령)
  - RTL이 GPIO로 S3 BOOT/EN 제어 → 부트모드 진입
  - USB↔UART 투명 브릿지 (esptool이 RTL 경유 S3 플래시)
  - 무통신 타임아웃 자동 종료 + S3 정상 리셋
- [x] config.h에 패스스루 GPIO/타임아웃 설정 추가
- [x] 옵션 E 문서 (설계전략, 비교표, 펌웨어 업그레이드) 업데이트
- [x] BW16 핀 매핑 수정 (D0~D12 → AMB_D0~AMB_D12)
- [x] MiniWS.h 커스텀 WebSocket 구현 (외부 라이브러리 비호환 대체)
- [x] RTL_PRINTF 매크로 (printf 미지원 우회)
- [x] 프레임 버퍼 128KB → 8KB (RAM 오버플로우 해결)
- [x] partitions.csv 한글→영문 (UnicodeDecodeError 해결)

---

## v0.1.6 — 동시성 안전 + 메모리 안정성 + 프로토콜 강화

- [x] RTL_PRINTF 전역 버퍼 → 스택 로컬 버퍼 (태스크 동시 접근 제거)
- [x] 프레임 버퍼 memcpy 구간까지 mutex 보호 확장 (JPEG 깨짐 방지)
- [x] MiniWS _readLine() 라인 길이 상한 256B (무한 헤더 방지)
- [x] String → char[] 전환 (WS handshake, readFrame, 시리얼 파서, S3 텍스트 펌프)
- [x] WS handshake: Upgrade + Connection 필수 헤더 검증 추가
- [x] 통계 변수: SNAPSHOT_AND_RESET_STATS 매크로 (임계구역 원자적 복사+리셋)
- [x] 상태 플래그 volatile 일관성 (hasControlClient, hasCameraClient, g_needReboot)
- [x] 패스스루 Ctrl+C x3 강제 탈출 경로 추가
- [x] 명령 파싱: 매직 인덱스 → strchr/strncmp 토큰 기반 변경

---

## Backlog (미배정 — 필요 시 버전에 할당)

- [ ] OTA(무선) 펌웨어 업데이트 지원 (누적 단계 2)
  - RTL이 Wi-Fi AP를 가지고 있으므로 OTA 구현 가능
- [ ] PlatformIO 또는 Arduino CLI 빌드 자동화
  - 현재: Arduino IDE 수동 빌드만 지원
  - 목표: 커맨드 라인 빌드로 CI/CD 연동 가능
- [ ] SPI 단일 링크로 제어+카메라 멀티플렉싱 통합
  - 현재: UART(제어) + SPI(카메라) 2링크
  - 목표: SPI 하나로 통합하여 배선 단순화
- [ ] 프레임 CRC32 체크 활성화
  - `dual_chip개발 전략.md`에 설계는 있으나 구현 보류 중
- [ ] Wi-Fi 설정 EEPROM/Flash 영구 저장
  - 현재: 재부팅 시 config.h 기본값으로 복원

---

## 변경 이력

| 버전 | 날짜 | 요약 |
|------|------|------|
| v0.1.0 | 2026-02-09 | 초기 듀얼 칩 구현 완료, GitHub 등록 |
| v0.1.1 | 2026-02-09 | config.h 분리, Wi-Fi 런타임 설정 명령 |
| v0.1.2 | 2026-02-09 | SPI 재동기화, 128KB 버퍼, 에러 전파 |
| v0.1.3 | 2026-02-09 | 디버그 토글, @diag 명령, 연결 시간 추적 |
| v0.1.4 | 2026-02-09 | MAC 기반 채널 자동 분산, 듀얼칩 설계전략 문서 |
| v0.1.5 | 2026-02-11 | 옵션 E: S3 패스스루 플래시, @passthru 명령 |
| v0.1.6 | 2026-02-11 | 동시성 안전, 메모리 안정성, WS 프로토콜 강화 |
