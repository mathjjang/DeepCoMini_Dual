# AmebaD (RTL8720DN) SPI 버그 분석 및 해결책

> 작성일: 2026-02-12  
> 대상 보드: BW16 (RTL8720DN), Arduino SDK (ameba-arduino-d)  
> 관련 버전: DeepCo_Dual v0.2.0 → v0.2.1

---

## 1. 문제 현상

RTL8720DN(Master)이 ESP32-S3(Slave)로부터 SPI를 통해 카메라 프레임을 수신할 때,
Arduino `SPI.transfer(buf, len)` **버퍼 일괄 전송 함수**를 사용하면
**수신 데이터가 손상**되어 정상적인 JPEG 프레임을 받을 수 없음.

바이트 단위 `SPI.transfer(byte)` 함수는 정상 동작하지만,
**2048바이트 블록을 1바이트씩 전송하므로 극심한 성능 병목** 발생.

---

## 2. 근본 원인 분석

### 2.1 Arduino SPI.cpp 소스코드 확인

소스 위치: `ameba-arduino-d/Arduino_package/hardware/libraries/SPI/src/SPI.cpp`

#### 버그 1: 동일 버퍼를 TX/RX에 공유

```cpp
// SPI.cpp — 버퍼 전송 (버그)
void SPIClass::transfer(byte _pin, void *_buf, size_t _count, SPITransferMode _mode) {
    if (_pin != pinSS) { pinMode(_pin, OUTPUT); digitalWrite(_pin, 0); }

    // ★ 버그: _buf를 TX와 RX 양쪽 모두에 전달!
    spi_master_write_read_stream(
        (spi_t *)pSpiMaster,
        (char *)_buf,     // TX buffer ← _buf
        (char *)_buf,     // RX buffer ← 동일한 _buf !!
        (uint32_t)_count
    );

    if ((_pin != pinSS) && (_mode == SPI_LAST)) { digitalWrite(_pin, 1); }
}
```

`spi_master_write_read_stream()`은 **인터럽트 기반 비동기 함수**로,
TX 인터럽트가 `_buf`에서 데이터를 읽어 전송하는 동시에
RX 인터럽트가 `_buf`에 수신 데이터를 덮어씀 → **TX 데이터가 중간에 오염**.

#### 버그 2: 비동기 전송인데 완료 대기 없음

```cpp
// spi_master_write_read_stream()은 인터럽트 기반 = Non-blocking
// 호출 즉시 리턴됨

spi_master_write_read_stream(...);  // 전송 시작만 하고 리턴

// ★ 버그: 전송 완료 전에 CS를 해제!
if ((_pin != pinSS) && (_mode == SPI_LAST)) {
    digitalWrite(_pin, 1);  // CS HIGH — 통신 중단됨
}
```

`spi_master_write_read_stream()`이 리턴된 시점에는 아직 데이터 전송이 진행 중.
그런데 Arduino 래퍼는 리턴 직후 **CS를 HIGH로 올리고** 다음 코드로 진행.
→ SPI 통신이 완료되기 전에 CS가 해제되어 **전송이 중단**됨.

#### 참고: spi_master_write_read_stream() 내부 구현

소스 위치: `ameba-rtos-d/component/common/mbed/targets/hal/rtl8721d/spi_api.c`

```c
int32_t spi_master_write_read_stream(spi_t *obj, char *tx_buffer,
    char *rx_buffer, uint32_t length)
{
    // ...
    // 별도의 TX/RX 버퍼를 지원하는 구조!
    if ((ret = ssi_int_read(ssi_adapter, rx_buffer, length)) == _TRUE) {
        obj->state |= SPI_STATE_TX_BUSY;
        if ((ret = ssi_int_write(ssi_adapter, (u8 *)tx_buffer, length)) != _TRUE) {
            // ...
        }
    }
    return (ret == _TRUE) ? HAL_OK : HAL_BUSY;
    // ★ 인터럽트만 설정하고 즉시 리턴 (Non-blocking!)
}
```

이 함수 자체는 **별도의 TX/RX 버퍼를 정상 지원**하지만,
Arduino 래퍼(`SPI.cpp`)가 동일 버퍼를 전달하고 완료 대기도 하지 않아서 문제 발생.

### 2.2 정상 동작하는 바이트 단위 전송

```cpp
// SPI.cpp — 바이트 단위 (정상 동작)
byte SPIClass::transfer(uint8_t _data, SPITransferMode _mode) {
    // 레지스터 직접 접근 + 동기식 대기
    while (!(HAL_READ32(spi_addr, 0x28) & 0x02));  // TX FIFO Not Full 대기
    HAL_WRITE32(spi_addr, 0x60, _data & 0xFFFF);    // DR에 1바이트 쓰기
    while (!(HAL_READ32(spi_addr, 0x28) & 0x08));  // RX FIFO Not Empty 대기
    d = HAL_READ32(spi_addr, 0x60);                  // DR에서 1바이트 읽기
    return d;
}
```

- 하드웨어 레지스터를 직접 접근하여 **1바이트씩 동기적으로** TX/RX
- 인터럽트 미사용, 동일 버퍼 문제 없음 → 항상 정확
- 단점: **매 바이트마다 대기 루프** → 2048바이트 기준 2048회 개별 사이클

---

## 3. 하드웨어 스펙 (rtl8721d_ssi.h)

| 항목 | SPI0 | SPI1 (BW16 사용) |
|------|------|------------------|
| Bus Clock | 100MHz | 50MHz |
| Max Baud Rate | ≤50MHz | ≤25MHz |
| **TX FIFO Depth** | **64 entries** | **64 entries** |
| **RX FIFO Depth** | **64 entries** | **64 entries** |
| DMA | GDMA 지원 | GDMA 지원 |
| Role | Master/Slave | **Master only** |

**핵심 포인트**: TX/RX FIFO가 각각 **64 entries**나 되지만,
바이트 단위 전송에서는 매번 1개씩만 사용하여 FIFO를 전혀 활용하지 못함.

### 레지스터 맵 (DW_apb_ssi)

| Offset | 이름 | 설명 |
|--------|------|------|
| 0x28 | SR (Status Register) | SPI 상태 비트 |
| 0x60 | DR (Data Register) | TX 쓰기 / RX 읽기 공용 |

#### SR 비트 필드

| Bit | 이름 | Mask | 설명 |
|-----|------|------|------|
| 0 | BUSY | 0x01 | SPI 전송 진행 중 |
| 1 | TFNF | 0x02 | TX FIFO Not Full (쓰기 가능) |
| 2 | TFE  | 0x04 | TX FIFO Empty |
| 3 | RFNE | 0x08 | RX FIFO Not Empty (읽기 가능) |
| 4 | RFF  | 0x10 | RX FIFO Full |

---

## 4. 해결 방안

### 방안 A: FIFO-Batch 레지스터 전송 (채택, v0.2.1)

바이트 단위와 동일한 **레지스터 직접 접근** 방식을 유지하되,
64-entry FIFO를 **파이프라인**으로 활용:

```cpp
static bool spiReadBlock(uint8_t* dst, size_t len) {
  if (!dst || len == 0) return false;

  const u8 spi_idx = spi_obj0.spi_idx & 0x0F;
  const u32 spi_addr = (spi_idx == MBED_SPI0) ? SPI0_REG_BASE : SPI1_REG_BASE;

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_SS_PIN, LOW);

  size_t txSent = 0, rxRecv = 0;

  while (rxRecv < len) {
    // Phase 1: TX FIFO에 0x00을 최대 64바이트 일괄 채움
    while (txSent < len && (txSent - rxRecv) < 64) {
      if (HAL_READ32(spi_addr, 0x28) & 0x02) {   // TFNF
        HAL_WRITE32(spi_addr, 0x60, 0x00);
        txSent++;
      } else break;
    }
    // Phase 2: RX FIFO에서 수신 데이터 일괄 회수
    while (rxRecv < txSent) {
      if (HAL_READ32(spi_addr, 0x28) & 0x08) {   // RFNE
        dst[rxRecv++] = (uint8_t)HAL_READ32(spi_addr, 0x60);
      } else break;
    }
  }

  digitalWrite(SPI_SS_PIN, HIGH);
  SPI.endTransaction();
  return true;
}
```

**필수 선언** (파일 상단):
```cpp
extern "C" {
  #include "spi_api.h"   // spi_t, MBED_SPIx, SPI_REG_BASE 등
}
extern spi_t spi_obj0;   // SPI.cpp에서 정의된 전역 (BW16용)
```

#### 동작 비교

```
기존: [TX 1B → wait → RX 1B] × 2048 = 많은 gap, SPI 클럭 불연속
신규: [TX 64B burst] → [RX drain] → [TX 64B burst] → ... × 32 = 연속 클럭
```

#### FIFO 오버플로 안전성

- `(txSent - rxRecv) < 64` 조건으로 TX ahead를 FIFO 깊이(64) 이내로 제한
- TX 64바이트 완료 시점에 RX FIFO에 최대 64바이트 — 정확히 FIFO 용량과 동일
- Phase 2에서 RX를 드레인한 후 다음 Phase 1 진행 → 오버플로 불가

#### 예상 성능

| 항목 | 바이트 단위 (v0.2.0) | FIFO-Batch (v0.2.1) |
|------|---------------------|---------------------|
| 2048B 블록 기준 | 2048회 개별 사이클 | ~32회 버스트 (64B씩) |
| 바이트당 대기 | 매 바이트 TX/RX 대기 | FIFO 단위 일괄 처리 |
| SPI 클럭 연속성 | 바이트 사이 gap 발생 | FIFO 내 연속 유지 |
| 예상 속도 향상 | - | **~10-30배** |
| 코드 변경 범위 | - | spiReadBlock 1개 함수만 교체 |
| SDK 수정 여부 | - | **불필요** |

---

### 방안 B: spi_master_write_read_stream() 별도 버퍼 + 완료 대기

Arduino 래퍼를 우회, 저수준 mbed 함수를 **별도 TX/RX 버퍼**로 직접 호출:

```cpp
extern spi_t spi_obj0;
static uint8_t g_spiTxZero[SPI_BLOCK_BYTES]; // 0으로 미리 초기화

static bool spiReadBlockStream(uint8_t* dst, size_t len) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_SS_PIN, LOW);

  memset(g_spiTxZero, 0, len);
  spi_master_write_read_stream(&spi_obj0, (char*)g_spiTxZero, (char*)dst, len);

  // 인터럽트 완료 대기
  while (spi_obj0.state & (SPI_STATE_RX_BUSY | SPI_STATE_TX_BUSY)) {
    vTaskDelay(1);
  }

  digitalWrite(SPI_SS_PIN, HIGH);
  SPI.endTransaction();
  return true;
}
```

| 장점 | 단점 |
|------|------|
| 인터럽트 기반 = CPU 유휴 활용 가능 | SDK 내부 state 의존 (검증 필요) |
| DMA에 가까운 효율 | extern spi_obj0 심볼 접근 필요 |
| TX 제로버퍼 별도 할당 필요 | |

---

### 방안 C: Full GDMA DMA 전송

SDK의 `ssi_dma_send()` + `ssi_dma_recv()`를 별도 TX/RX 버퍼로 사용:

```cpp
// DMA — CPU 부하 0에 가까움
// 단, spi_master_write_read_stream_dma()는 SDK에 없음
// TX DMA + RX DMA를 각각 설정해야 함
static bool spiReadBlockDMA(uint8_t* txBuf, uint8_t* rxBuf, size_t len) {
  // DMA-capable 메모리 필요
  ssi_dma_send(adapter, txBuf, len);
  ssi_dma_recv(adapter, rxBuf, len);
  // 완료 대기...
}
```

| 장점 | 단점 |
|------|------|
| 전송 중 CPU 완전 자유 | 가장 복잡한 구현 |
| 최고 throughput | DMA-capable 메모리 요구 |
| | spi_master_write_read_stream_dma() 미제공 |
| | TX/RX DMA 개별 설정 필요 |

---

## 5. 방안 종합 비교

| 항목 | 바이트 단위 (현재) | A: FIFO-Batch | B: Interrupt Stream | C: DMA |
|------|-------------------|---------------|--------------------|----|
| 예상 속도 (2KB) | ~2ms @10MHz | ~0.1-0.2ms | ~0.2ms | ~0.2ms |
| CPU 점유 | 100% busy wait | ~50% 파이프라인 | ~5% 인터럽트 대기 | ~0% |
| 구현 난이도 | - | **낮음** | 중간 | 높음 |
| SDK 수정 | 없음 | **없음** | 없음 | 없음 |
| 안정성 | 검증됨 | **높음** (동일 방식) | 미검증 | 미검증 |
| 코드 변경량 | - | 함수 1개 | 함수 1개 + 버퍼 | 대규모 |

**권장 순서**: A → B → C (성능 부족 시 순차 적용)

---

## 6. 참조 소스코드 위치

| 파일 | 경로 (ameba-arduino-d repo) |
|------|---------------------------|
| Arduino SPI 래퍼 | `Arduino_package/hardware/libraries/SPI/src/SPI.cpp` |
| Arduino SPI 헤더 | `Arduino_package/hardware/libraries/SPI/src/SPI.h` |
| mbed SPI HAL | `system/libameba/.../spi_api.c` (또는 ameba-rtos-d repo) |
| SPI 레지스터 정의 | `system/.../rtl8721d_ssi.h` |
| SPI 디바이스 테이블 | `SPI_DEV_TABLE[2]` in rtl8721d_ssi.h |

### GitHub 참조
- Arduino SDK: https://github.com/Ameba-AIoT/ameba-arduino-d
- RTOS SDK: https://github.com/Ameba-AIoT/ameba-rtos-d

---

## 7. 부록: SPI.transfer() 의 두 가지 오버로드 비교

### (a) 단일 바이트 (정상 동작)

```
호출: SPI.transfer(0x00)
내부: HAL_WRITE32(DR, 0x00) → HAL_READ32(DR) 
방식: 레지스터 직접 접근, 동기식
결과: 항상 정확
```

### (b) 버퍼 일괄 (버그)

```
호출: SPI.transfer(pin, buf, len, mode)
내부: spi_master_write_read_stream(spi, buf, buf, len)  ← 동일 버퍼!
방식: 인터럽트 기반, 비동기 (완료 대기 없음)
결과: 데이터 손상 + CS 조기 해제
```

### (c) FIFO-Batch (방안 A)

```
호출: spiReadBlock(dst, len)  — 커스텀 함수
내부: HAL_WRITE32(DR, 0x00) × 64 → HAL_READ32(DR) × 64 → 반복
방식: 레지스터 직접 접근, 동기식, FIFO 파이프라인
결과: 정확 + 고속 (~10-30x 향상)
```
