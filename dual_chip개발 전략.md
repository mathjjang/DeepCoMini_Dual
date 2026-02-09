# DeepCo Dual Chip 개발 전략 (RTL8720DN + ESP32‑S3)

## 0) 목표(한 줄 요약)
현 `DeepcoMini_v1.3`(ESP32‑S3 단독)를 **ESP32‑S3(카메라/모터/로컬제어)** + **RTL8720DN(무선/네트워크 서버)** 로 분리하되, **DeepCoConnector/웹 쪽 인터페이스는 최대한 그대로 유지**한다.

---

## 1) 배경/현상
`DeepcoMini_v1.3`는 ESP32‑S3가 2.4GHz SoftAP를 직접 열고,
- 제어 WebSocket: `ws://192.168.4.1/ws` (port 80의 `/ws`)
- 카메라 WebSocket: `ws://192.168.4.1:81/` (port 81)
를 제공한다.

전시장 2.4GHz 혼잡 환경에서 SoftAP가 자주 끊기는 문제가 있어, **무선 부분을 5GHz 지원 가능한 RTL8720DN으로 분리**한다.

---

## 2) “외부(DeepCoConnector) 호환성”을 최우선으로 고정
DeepCoConnector는 로봇을 다음 타겟으로 붙는다(하드코딩/전제 존재):
- control downstream: `ws://192.168.4.1/ws`
- camera downstream: `ws://192.168.4.1:81/`

따라서 Dual‑Chip 1차 목표는 다음을 **그대로 제공**하는 것:
- **로봇 IP**: `192.168.4.1` (고정)
- **제어 WS 엔드포인트**: `ws://192.168.4.1/ws`
- **카메라 WS 엔드포인트**: `ws://192.168.4.1:81/`

> 결론: “네트워크 서버”는 RTL8720DN이 담당하되, DeepCoConnector 입장에서는 **로봇이 그대로 있는 것처럼** 보이게 만든다.

---

## 3) 역할 분리(권장)

### 3.1 ESP32‑S3(현재 Mini 1.3에서 남길 것)
- **카메라**
  - OV2640/OV5640 초기화 및 캡처(`esp_camera`)
  - JPEG 프레임 생성(현재처럼)
- **모터/주행**
  - `FastAccelStepper` 기반 구동 로직
  - 기존 문자열 커맨드 파서(`forward,100`, `angle,93`, `robot,{...}` 등) 유지
- **LED/상태표시**
  - 현재 LED 상태 머신은 유지하되, “Wi‑Fi 연결/클라이언트 연결” 신호는 RTL로부터 전달받는다(아래 6장).
- **RTL로부터 받은 명령 실행 + 상태 응답**
  - 제어 명령 실행 결과/상태를 RTL로 송신

### 3.2 RTL8720DN(현재 Mini 1.3에서 옮길 것)
- **Wi‑Fi**
  - (권장) 5GHz AP(SoftAP) 또는 STA 모드 운영
  - DHCP/고정 IP 운용: **자신을 `192.168.4.1`로 설정**하고 클라이언트에게 `192.168.4.x` 할당
- **네트워크 서버**
  - Control WS 서버: `ws://<ip>/ws`
  - Camera WS 서버: `ws://<ip>:81/`
  - (선택) HTTP `/`(헬스/진단 페이지) — 필수 아님(DeepCoConnector는 WS 중심)
- **브릿지(중요)**
  - Control WS에서 받은 텍스트를 S3로 전달
  - S3가 보내는 JPEG 프레임을 받아 Camera WS로 바이너리 프레임으로 송출
- **상태 관리/안전**
  - 클라이언트 연결 끊김/재연결 처리
  - 제어 채널이 끊기면 “안전 정지(stop)”를 S3에 강제 전송(폴백)

---

## 4) 데이터 플로우(그림)

### 4.1 Control
PC(DeepCoConnector) → `ws://192.168.4.1/ws` → (RTL) → [칩간 링크] → (S3) → 모터/LED

### 4.2 Camera
(S3) → JPEG 프레임 → [칩간 링크] → (RTL) → `ws://192.168.4.1:81/` → PC(DeepCoConnector)

---

## 5) 칩간 링크(가장 중요한 설계 결정)

### 5.1 결론(권장 조합)
- **Control**: UART도 가능(저대역, 단순)
- **Camera 스트리밍**: **SPI 권장** (UART로는 QVGA/VGA MJPEG에 한계가 빨리 온다)

즉,
- **UART(제어/상태)** + **SPI(카메라)** 의 “2링크”가 가장 현실적이다.
  - 개발/디버깅도 쉽고, 대역폭도 확보된다.

### 5.2 단일 링크로 가고 싶다면
- **SPI 단일 링크**로 제어+카메라를 멀티플렉싱하는 게 최종적으로 가장 깔끔하다.
  - 초기 bring‑up은 UART로 제어만 먼저 살리고 → 이후 SPI 카메라 추가하는 단계적 접근을 추천.

---

## 6) 칩간 프로토콜(초안: “프레이밍 + 타입”만 고정)
목표는 “손실/바이트 깨짐이 있어도 재동기화 가능”한 프레임 구조다.

### 6.1 공통 프레임 헤더(바이너리)
- **SYNC(4B)**: `0x44 0x43 0x4D 0x01` (`"DCM\x01"`) 같은 고정값
- **TYPE(1B)**:
  - `0x01` = CONTROL_TEXT
  - `0x02` = CONTROL_STATUS_JSON
  - `0x10` = CAMERA_JPEG_FRAME
  - `0x11` = CAMERA_DROP_NOTICE(프레임 드랍/오버런)
  - `0x20` = LINK_STATE(클라 접속 수/WS 연결 상태)
- **FLAGS(1B)**: 예약(압축/조각 여부 등)
- **SEQ(2B)**: 증가 시퀀스(16bit)
- **LEN(4B)**: payload length
- **CRC32(4B)**: payload CRC(선택이지만 강력 추천)

payload는 `LEN`만큼 뒤따른다.

### 6.2 Control 채널 정책
- WS에서 들어오는 문자열을 S3가 이미 파싱하고 있으므로,
  - RTL은 **CONTROL_TEXT payload = “그대로 전달”**
  - S3는 기존 `handleWebSocketMessage()` 로직을 재사용 가능(입력만 “serial에서 온 문자열”로 바꿈)
- S3 → RTL 방향으로는,
  - 상태(JSON) 또는 ACK를 `CONTROL_STATUS_JSON`으로 올린다.

### 6.3 Camera 채널 정책
- S3가 `esp_camera_fb_get()`으로 얻은 JPEG 프레임을 **프레임 단위로** RTL에 전달한다.
- RTL은 해당 payload를 **WS Binary message 1개 = JPEG 1프레임**으로 그대로 송출한다.
- 백프레셔:
  - RTL이 전송 지연이면 “수신 큐가 찰 것”이므로, S3는 최신 프레임 우선(`CAMERA_GRAB_LATEST`) + **프레임 드랍 허용**이 안정적.

---

## 7) LED/상태 표시 분리(현 v1.3 로직을 깨지 않기)
`DeepcoMini_v1.3`은 LED 상태에
- “SoftAP에 STA가 붙었는지(= WiFi 연결감지)”
- “/ws 웹소켓이 붙었는지”
를 사용한다.

Dual‑Chip에서는 S3가 더 이상 Wi‑Fi를 직접 모르므로, RTL이 아래 정보를 주기적으로 S3에게 알려준다:
- **ap_sta_count**(현재 연결된 클라이언트 수)
- **control_ws_connected**(제어 WS 연결 여부)
- **camera_ws_connected**(카메라 WS 연결 여부, 선택)

S3는 이 값을 기반으로 기존 LED 상태 머신을 유지한다.

---

## 8) 네트워크 운용 모드 제안(현장 기준)

### 8.1 1안(추천): RTL8720DN이 로봇 AP(5GHz) 제공
- 현장 운영이 단순(“PC가 로봇 AP에 붙는다” 구조 유지)
- **2.4GHz 혼잡 회피(핵심)**
- DeepCoConnector 변경 최소(기존처럼 `192.168.4.1`로 접속)

### 8.2 2안: RTL8720DN이 STA로 노트북 핫스팟/공유기에 붙음
- IP가 변할 수 있어 디스커버리(또는 고정 IP/예약) 문제가 생김
- DeepCoConnector의 `192.168.4.1` 전제를 깨기 쉬움

> 이번 문서의 1차 목표는 “수정 범위 최소”이므로 8.1을 기본으로 잡는다.

---

## 9) 개발 단계(가장 안전한 순서)

### Phase A: Control 먼저(영상 없이도 로봇이 움직이면 절반 성공)
1) RTL에서 `ws://192.168.4.1/ws` 서버 구현
2) WS 수신 문자열을 UART로 S3에 전달
3) S3는 기존 커맨드 파서를 재사용해 모터 구동
4) WS disconnect 시 RTL이 S3에 `stop` 전송(안전)

### Phase B: Camera 추가(대역폭/버퍼링 설계 핵심)
1) SPI 링크 bring‑up(또는 SPI 단일링크로 통합)
2) S3가 JPEG 프레임을 RTL로 전송
3) RTL에서 `:81` WS 서버로 바이너리 프레임 송출
4) 렌더링/지연/프레임 드랍 정책 튜닝

### Phase C: 상태/LED/진단 강화
- LINK_STATE 주기 송신, LED 정확도 개선
- 진단 페이지/로그(현재 채널, RSSI, client count, dropped frames 등)

---

## 10) 리포지토리/폴더 구성(권장)
`DeepcoMini/DeepCo_Dual/`
- `esp32s3/` : S3 펌웨어(카메라/모터/LED/IPC)
- `rtl8720dn/` : RTL 펌웨어(Wi‑Fi/AP + WS 서버 + IPC)
- `shared/` : 프로토콜 헤더/문서(프레임 정의, 타입)
- `dual_chip개발 전략.md` : 본 문서

---

## 11) 체크리스트(완성 기준)
- PC에서 로봇 AP(5GHz)에 연결 가능
- `ws://192.168.4.1/ws`로 “forward/left/right/stop/robot,{...}”가 정상 동작
- `ws://192.168.4.1:81/`에서 JPEG 바이너리 프레임이 연속 수신(DeepCoConnector 카메라 화면 정상)
- 끊김 시 자동 재연결 + 안전정지
- LED가 “연결 상태”를 사용자에게 정확히 보여줌

---

## 12) 메모(현 v1.3에서 실제로 ‘잘라낼’ 코드 영역 힌트)
`DeepcoMini_v1.3.ino` 기준으로,
- RTL로 이동(삭제/대체 대상)
  - `#include <WiFi.h>`, `esp_wifi.h` 및 SoftAP 설정/채널 스캔/HT20 설정
  - `AsyncWebServer/AsyncWebSocket`(제어 WS) / `WebSocketsServer`(카메라 WS) 서버 소켓 계층
- S3에 유지
  - `esp_camera` 초기화/캡처
  - 모터 제어/커맨드 파싱
  - LED task(단, “연결 정보 입력”을 RTL에서 받도록 수정)

