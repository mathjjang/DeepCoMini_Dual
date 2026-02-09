# DeepCo_Dual Arduino IDE 빌드 방법(초안)

## 1) 생성된 스케치

- **RTL8720DN(=BW16 등, AmebaD)**: `DeepCo_Dual/rtl8720dn/DeepCoRTL_Bridge/DeepCoRTL_Bridge.ino`
  - `ws://192.168.4.1/ws` (port 80) 제공 + UART로 S3에 명령 전달
  - `ws://192.168.4.1:81/` (port 81) 카메라 WS 제공
  - 카메라 데이터는 **SPI(Master=RTL, Slave=S3)** 로 가져옴

- **ESP32‑S3**: `DeepCo_Dual/esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino`
  - UART로 받은 `forward,100` / `robot,{...}` 명령을 기존 로직으로 실행
  - Wi‑Fi/웹서버 제거(네트워크는 RTL이 담당)
  - 카메라 JPEG 프레임을 **SPI Slave로 블록 단위 송출**

---

## 2) Arduino IDE 준비

### 2.1 ESP32‑S3
- Boards Manager에서 **ESP32 by Espressif** 설치
- 보드 선택: 사용 중인 S3 보드에 맞게 선택

### 2.2 RTL8720DN (AmebaD)
- Boards Manager에서 **Realtek AmebaD (RTL8720DN)** 설치(가이드에 따라 URL 추가 필요)
- 보드 선택: BW16/AMBxx 등 실제 보드에 맞게 선택

---

## 3) 라이브러리 설치(필수)

### 3.1 RTL8720DN 스케치
- Arduino Library Manager에서 **`WebSockets2_Generic`** 설치
  - 이 라이브러리는 `#include <ArduinoWebsockets.h>` API를 제공

### 3.2 ESP32‑S3 스케치
- `ArduinoJson`
- `Adafruit NeoPixel`
- `FastAccelStepper`
- (카메라) ESP32 보드 패키지에 포함된 `esp_camera` 사용

---

## 4) 핀/배선(반드시 맞춰야 함)

### 4.1 ESP32‑S3 핀(현재 코드 기본값)
`DeepCo_Dual/esp32s3/DeepCoS3_Robot/DeepCoS3_Robot.ino` 기준 기본값:

- **UART(로봇 제어 링크, S3=Slave 역할)**  
  - `LINK_TX_PIN = 43` (S3 → RTL)  
  - `LINK_RX_PIN = 44` (RTL → S3)

- **SPI Slave(카메라 링크, S3=Slave)**  
  - `SPI_CS_PIN   = 34` (RTL SS → S3 CS)  
  - `SPI_MISO_PIN = 35` (**S3 → RTL**, Slave MISO)  
  - `SPI_SCLK_PIN = 36` (RTL SCLK → S3 SCLK)  
  - `SPI_MOSI_PIN = 37` (**RTL → S3**, Master MOSI)

> ESP32‑S3는 IO‑Matrix로 핀 라우팅이 가능하므로, **PCB에서 위 핀으로 그대로 배선**하는 것을 권장합니다(특별한 이유가 없으면 변경 X).

### 4.2 RTL8720DN(BW16) 고정 핀(칩/보드 정보 기반)
BW16 Arduino variant(`rtl8720dn_bw16`) 기준:

- **USB 디버그 콘솔(Serial)**  
  - `D0 = PA7 (LOG_TX)`  
  - `D1 = PA8 (LOG_RX)`  
  → 보통 BW16 보드의 USB‑C 디버그 콘솔로 연결됨

- **UART 링크(Serial1)**  
  - `D4 = PB1 (SERIAL1_TX)` (RTL → S3)  
  - `D5 = PB2 (SERIAL1_RX)` (S3 → RTL)

- **SPI Master(카메라 링크)**  
  - `SPI_SS   = PA15 (D9)`  (RTL SS → S3 CS)  
  - `SPI_SCLK = PA14 (D10)` (RTL SCLK → S3 SCLK)  
  - `SPI_MISO = PA13 (D11)` (S3 → RTL)  
  - `SPI_MOSI = PA12 (D12)` (RTL → S3)

> RTL8720DN(AmebaD)에서는 `SPI_SS_PIN = SPI_SS`(보드 기본 SS)를 강력 추천합니다.

### 4.3 실제 배선 요약(빠짐 없이)
- **GND**: RTL ↔ S3 **공통 GND 필수**
- **UART(교차 연결)**:
  - RTL `D4 (Serial1 TX)` → S3 `LINK_RX_PIN(44)`
  - RTL `D5 (Serial1 RX)` ← S3 `LINK_TX_PIN(43)`
- **SPI(동일명 연결)**:
  - RTL `D9 (SPI_SS)`   → S3 `SPI_CS_PIN(34)`
  - RTL `D10 (SPI_SCLK)`→ S3 `SPI_SCLK_PIN(36)`
  - RTL `D12 (SPI_MOSI)`→ S3 `SPI_MOSI_PIN(37)`
  - RTL `D11 (SPI_MISO)`← S3 `SPI_MISO_PIN(35)`

---

## 5) 동작 확인(최소 성공 기준)

1) RTL에 펌웨어 업로드 → AP SSID가 `DCM-XXXXXXXX` 형태로 뜨는지 확인
2) PC가 해당 AP에 연결
3) DeepCoConnector가 `ws://192.168.4.1/ws`에 붙으면
4) S3가 모터 커맨드를 실행(전/좌/우/정지)

---

## 6) 다음 단계(카메라 브릿지)
카메라 브릿지는 기본으로 포함되어 있습니다.
- RTL은 **WS 연결 유무와 관계없이** SPI Master로 S3로부터 **항상 최신 프레임을 pull** 합니다(기본: WS 없을 때는 저속 갱신).
- WS 카메라 클라이언트가 연결된 경우에만 `ws://192.168.4.1:81/`로 프레임을 네트워크 송출합니다.
- (옵션) S3는 `@cam,0/@cam,1` UART 명령을 받으면 스트림을 끄고/켤 수 있게 코드는 남겨둠.

