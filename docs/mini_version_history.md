# DeepcoMini 버전 히스토리

## 개요
DeepcoMini는 ESP32-S3 기반의 카메라 로봇 제어 시스템입니다. WebSocket을 통한 실시간 카메라 스트리밍과 스텝 모터 제어를 지원합니다.

---

## 버전 1.0 (DeepcoMini_v1.0)

### 릴리즈 날짜
2024 초기

### 주요 특징

#### 1. 기본 하드웨어 지원
- **MCU**: ESP32-S3 (ESP32S3_EYE 모델)
- **카메라**: QVGA 해상도, JPEG 포맷
  - 단순 hmirror 설정만 지원
  - 센서 자동 감지 없음
- **모터 드라이버**: FastAccelStepper 라이브러리
  - 좌측 모터: GPIO 21(STEP), GPIO 42(DIR), GPIO 41(EN)
  - 우측 모터: GPIO 40(STEP), GPIO 39(DIR), GPIO 38(EN)

#### 2. 네트워크 구성
- **SoftAP 모드**: 고정 SSID "DeepCoMini101"
- **Wi-Fi 채널**: 고정 채널 6
- **SoftAP IP**: 192.168.4.1
- **비밀번호**: "12345678"

#### 3. 모터 제어
- **회전 각도**: 하드코딩된 값 (stepPerAngle = 11.21111 * 2)
- **로봇 지오메트리**: 설정 불가 (값이 하드코딩됨)
- 기본 이동, 회전, 정지 기능

#### 4. 통신
- **시리얼 통신**: 115200 baud
- **카메라 스트리밍**: WebSocket (포트 81)
- **로봇 제어**: AsyncWebSocket (/ws, 포트 80)

#### 5. 제한사항
- **LED 상태 표시 없음** - 로봇 상태를 시각적으로 확인 불가
- 카메라 센서 타입에 따른 최적화 없음
- SSID가 고정되어 여러 로봇 사용 시 충돌 가능
- 로봇 크기가 바뀌면 코드 수정 필요

---

## 버전 1.1 (DeepcoMini_v1.1)

### 릴리즈 날짜
2024 중기

### v1.0 대비 변경사항

#### 주요 업그레이드

**1. LED 상태 표시 시스템 추가 🎨**
- **중앙 LED** (GPIO 2): 1픽셀 RGB LED
- **좌측 NeoPixel** (GPIO 48): 4픽셀 GRB
- **우측 NeoPixel** (GPIO 47): 4픽셀 GRB

**LED 상태**:
- `BOOTING`: 파랑 progress bar (좌/우) + 흰색 breathing (중앙, 3초간)
- `READY`: LED 꺼짐 (좌/우) + 은은한 빨강 breathing (중앙)
- `CONNECTED`: 파랑 고정 점등 (좌/우) + 은은한 초록 고정 (중앙)
- `DISCONNECT`: 빨강 1회 깜빡임 (좌/우, 300ms)

**2. 카메라 센서 자동 감지 및 최적화**
- OV2640/OV5640 PID 기반 자동 감지
- 센서별 최적 설정 자동 적용
  - OV2640: hmirror + vflip, quality 12
  - OV5640: hmirror only, quality 10

**3. 로봇 지오메트리 기반 회전 계산**
- `computeStepPerAngle()` 함수 추가
- 로봇 크기 변경 시 간단한 상수 수정만으로 대응 가능
  - WHEEL_DIAMETER_MM = 33.50mm
  - TRACK_WIDTH_MM = 73.5mm
  - STEPS_PER_WHEEL_REV = 2000.0f

**4. 기타 개선사항**
- **시리얼 통신**: 921600 baud로 향상 (8배 빠름)
- **SSID**: "DeepCoMini02"로 변경
- **Wi-Fi 채널**: 11로 변경
- **FreeRTOS**: LED 전용 태스크 추가 (논블로킹)

### v1.0와 동일하게 유지된 기능
- 모터 제어 기본 기능
- WebSocket 기반 통신
- 카메라 스트리밍 프로토콜

---

## 버전 1.2 (DeepcoMini_v1.2)

### 릴리즈 날짜
2024 후기

### v1.1 대비 변경사항

#### 주요 업그레이드

**1. 4단계 LED 상태 표시 시스템 🔄**
- v1.1의 3단계 상태에서 **4단계로 확장**
- 새로운 상태: `WIFI_CONNECTED` 추가
  - USB 동글이 로봇 AP에 연결되었지만 웹 UI는 아직 연결 안 된 상태 표시
  - LED: 좌/우 꺼짐, 중앙 은은한 초록 breathing

**LED 상태 (v1.2)**:
- `BOOTING`: 파랑 progress bar (좌/우) + 흰색 breathing (중앙, 3초간)
- `READY`: LED 꺼짐 (좌/우) + 은은한 빨강 breathing (중앙)
- `WIFI_CONNECTED`: LED 꺼짐 (좌/우) + 은은한 초록 breathing (중앙) ⭐ 신규
- `CONNECTED`: 파랑 고정 점등 (좌/우) + 은은한 초록 고정 (중앙)
- `DISCONNECT`: 빨강 1회 깜빡임 (좌/우, 300ms)

**2. MAC 주소 기반 자동 SSID 생성 🏷️**
- v1.1: 고정 SSID "DeepCoMini02" → 여러 로봇 사용 시 충돌
- v1.2: MAC 주소 기반 고유 SSID 자동 생성
  - 형식: `DCM-XXXXXXXX` (예: DCM-4ECDE594)
  - 로봇마다 고유한 SSID → 다중 로봇 환경에서 혼란 없음

**3. 연결 안정성 대폭 강화 🛡️**
- **SoftAP 스테이션 폴링** (1초 주기)
  - USB 동글 연결 상태 실시간 감지
  - 동글이 갑자기 제거되면 즉시 감지하여 LED 상태 변경
- **WebSocket 연결 상태 자동 추적**
  - `g_wsConnected` 플래그로 웹 UI 연결 상태 추적
  - 연결/해제 시 LED 상태 자동 업데이트

**4. 네트워크 설정 변경**
- **Wi-Fi 채널**: 11 → 6으로 변경 (더 보편적인 채널)
- **시리얼 통신**: 921600 → 115200 baud (안정성 우선)

### v1.1과 동일하게 유지된 기능
- LED 하드웨어 구성
- 카메라 센서 자동 감지 (OV2640/OV5640)
- 로봇 지오메트리 기반 회전 계산
- 모터 제어 기능
- WebSocket 기반 통신
- FreeRTOS 멀티태스킹

---

## 버전 1.3 (DeepcoMini_v1.3)

### 릴리즈 날짜
2024-2025

### v1.2 대비 변경사항

#### 주요 업그레이드: Wi-Fi 채널 자동 선택 기능 추가

**새로운 기능**:
- `pickBestChannel_6_or_11()` 함수 추가
  - 부팅 시 주변 AP를 자동 스캔
  - 채널 6과 채널 11 중 덜 혼잡한 채널 자동 선택
  - 채널 6 주변(4-8): count6
  - 채널 11 주변(9-13): count11
  - 더 적은 AP가 있는 채널 선택

**구현 상세**:
```cpp
// v1.2: 고정 채널 6
WiFi.softAP(softap_ssid, softap_password, 6);

// v1.3: 자동 채널 선택
int bestChannel = pickBestChannel_6_or_11();
WiFi.softAP(softap_ssid, softap_password, bestChannel);
```

**기술적 개선점**:
- Wi-Fi 모드를 임시로 `WIFI_AP_STA`로 전환하여 스캔 수행
- 스캔 완료 후 `WIFI_AP` 모드로 복귀
- 스캔 실패 시 기본값 채널 6 사용 (폴백 메커니즘)
- 시리얼 출력으로 채널 선택 과정 디버깅 가능

**사용자 경험 개선**:
- 혼잡한 Wi-Fi 환경에서 안정성 향상
- 자동으로 최적 채널 선택하여 수동 설정 불필요
- 카메라 스트리밍 품질 향상 가능

### v1.2와 동일하게 유지된 기능
- LED 상태 표시 시스템
- 모터 제어 기능 및 로봇 지오메트리
- 카메라 스트리밍 프로토콜
- SoftAP MAC 기반 SSID 생성
- WebSocket 기반 제어 인터페이스
- 안정성 기능 (폴링, 자동 재연결 등)

---

## 버전별 비교표

| 기능 | v1.0 | v1.1 | v1.2 | v1.3 |
|------|------|------|------|------|
| **LED 상태 표시** | ❌ 없음 | ✅ 3단계 | ✅ 4단계 | ✅ 4단계 |
| **카메라 센서 자동 감지** | ❌ 없음 | ✅ 지원 | ✅ 지원 | ✅ 지원 |
| **SSID** | 고정 (DeepCoMini101) | 고정 (DeepCoMini02) | MAC 기반 자동생성 | MAC 기반 자동생성 |
| **Wi-Fi 채널** | 6 (고정) | 11 (고정) | 6 (고정) | **6/11 자동선택** |
| **로봇 지오메트리** | 하드코딩 | 설정 가능 | 설정 가능 | 설정 가능 |
| **연결 안정성** | 기본 | 기본 | **폴링 강화** | **폴링 강화** |
| **시리얼 통신** | 115200 baud | 921600 baud | 115200 baud | 115200 baud |
| **다중 로봇** | ❌ 불가 | ❌ 불가 | ✅ 가능 | ✅ 가능 |

---

## 권장 사항

### 버전 선택 가이드
- **v1.0**: 사용 비권장 (초기 프로토타입, LED 없음)
- **v1.1**: 단일 로봇 환경, LED 표시 필요 시
- **v1.2**: 다중 로봇 환경, 안정적인 연결 필요 시
- **v1.3**: 혼잡한 Wi-Fi 환경, 최적 성능 필요 시 ⭐ **권장**

### 업그레이드 경로
```
v1.0 (기본 기능만)
  ↓
v1.1 (LED + 센서 최적화)
  ↓
v1.2 (안정성 강화 + 다중 로봇)
  ↓
v1.3 (Wi-Fi 최적화) ← 현재 최신
```

### 향후 개선 방향
- [ ] OV5640 사용 시 해상도 자동 상향 조정
- [ ] 배터리 상태 모니터링 추가
- [ ] Web UI 개선 (모바일 터치 지원)
- [ ] 센서 데이터 (거리, IMU) 통합
- [ ] OTA (무선 펌웨어 업데이트) 지원

---

## 기술 스펙 요약

### 하드웨어
| 항목 | 사양 | 비고 |
|------|------|------|
| MCU | ESP32-S3 (Dual-core Xtensa LX7) | ESP32S3_EYE 모델 |
| 카메라 | OV2640/OV5640 | QVGA (320x240), JPEG |
| 모터 드라이버 | TMC2225 (2개) | 스텝 모터 전용, 저소음 |
| 모터 | 스텝 모터 2개 | 좌/우 독립 제어 |
| LED | 1x 중앙 RGB + 2x NeoPixel(각 4픽셀) | v1.1부터 지원 |

### 소프트웨어
| 항목 | 사양 | 비고 |
|------|------|------|
| Wi-Fi | 2.4GHz SoftAP 모드 | 192.168.4.1 |
| 카메라 스트리밍 | WebSocket (포트 81) | 바이너리, JPEG |
| 로봇 제어 | AsyncWebSocket (포트 80, /ws) | 텍스트/JSON |
| 프레임워크 | Arduino-ESP32, FreeRTOS | 멀티태스킹 |
| 시리얼 통신 | 115200 / 921600 baud | 버전에 따라 다름 |

### 라이브러리
- `ESPAsyncWebServer` - 비동기 웹 서버
- `ArduinoWebSockets` - 카메라 스트리밍 (포트 81)
- `ArduinoJson` - JSON 파싱
- `Adafruit_NeoPixel` - LED 제어 (v1.1+)
- `FastAccelStepper` - 스텝 모터 제어

### 로봇 지오메트리 (v1.1+)
| 파라미터 | 값 |
|---------|-----|
| 휠 직경 | 33.50mm |
| 트랙 폭 (좌우 바퀴 간격) | 73.5mm |
| 휠당 스텝 수 | 2000 |
| 최대 속도 제한 | 2000 SPS |

---

## 참고 문서
- `led사용법.md`: LED 상태 표시 시스템 상세 설명
- [ESP32-S3 데이터시트](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [FastAccelStepper 라이브러리](https://github.com/gin66/FastAccelStepper)
- [TMC2225 드라이버 문서](https://www.trinamic.com/products/integrated-circuits/details/tmc2225/)

---

## 변경 이력
- **2026년 1월**: v1.3 Wi-Fi 채널 자동 선택 추가
- **2024년 후기**: v1.2 안정성 강화, MAC 기반 SSID
- **2024년 중기**: v1.1 LED 시스템, 센서 최적화
- **2024년 초기**: v1.0 초기 릴리즈

**마지막 업데이트**: 2026년 1월 26일
