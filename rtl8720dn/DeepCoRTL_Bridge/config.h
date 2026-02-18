/*
  config.h — RTL8720DN (DeepCoRTL_Bridge) 설정 파일
  v0.1.1: 하드코딩된 핀/파라미터를 한 곳에서 관리
  v0.1.5: 옵션 E (RTL 하이브리드) — S3 패스스루 플래시 GPIO 추가

  Wi-Fi 채널/비밀번호는 시리얼 명령으로도 변경 가능 (v0.1.1+)
*/
#pragma once

// =============================================
// 펌웨어 버전 (GitHub 태그와 반드시 일치시킬 것)
// =============================================
#define CFG_FW_VERSION    "0.2.2"
#define CFG_FW_CHIP       "RTL"

// =============================================
// Wi-Fi AP 설정 (기본값)
// 런타임 시리얼 명령으로 변경 가능:
//   @set,channel,149    → 채널 변경 (재부팅 필요)
//   @set,password,xxxx  → 비밀번호 변경 (재부팅 필요)
// =============================================
#define CFG_AP_PASSWORD_DEFAULT   "12345678"

// ---- 채널 자동 분산 (교실 다수 로봇 대응) ----
// 1 = MAC 주소 기반 자동 채널 분산 (교실 권장)
// 0 = CFG_AP_CHANNEL_DEFAULT 고정 채널 사용
#define CFG_AP_CHANNEL_AUTO       1

// 자동 분산 시 사용할 채널 풀 (5GHz 비중첩 8채널)
// MAC 마지막 바이트 % 채널수 → 인덱스로 선택
// 교실 30대 → 8채널에 ~4대씩 분산 → 간섭 1/8로 감소
// 필요 시 2.4GHz 채널(1,6,11)로 변경 가능
#define CFG_AP_CHANNEL_POOL       {36, 40, 44, 48, 149, 153, 157, 161}
#define CFG_AP_CHANNEL_POOL_SIZE  8

// 고정 채널 (CFG_AP_CHANNEL_AUTO=0일 때 사용)
// 5GHz 채널: 36/40/44/48 또는 149/153/157/161
// 2.4GHz 채널: 1~13
#define CFG_AP_CHANNEL_DEFAULT    36

// 로봇 고정 IP (DeepCoConnector 호환)
#define CFG_AP_IP_A   192
#define CFG_AP_IP_B   168
#define CFG_AP_IP_C   4
#define CFG_AP_IP_D   1

#define CFG_AP_SUBNET_A  255
#define CFG_AP_SUBNET_B  255
#define CFG_AP_SUBNET_C  255
#define CFG_AP_SUBNET_D  0

#define CFG_AP_DNS_A  8
#define CFG_AP_DNS_B  8
#define CFG_AP_DNS_C  8
#define CFG_AP_DNS_D  8

// =============================================
// 옵션 E: S3 패스스루 플래시 (RTL 하이브리드)
// RTL이 S3의 BOOT/EN을 GPIO로 제어하여 부트모드 진입 후
// USB↔UART 투명 브릿지(패스스루)로 esptool이 S3를 직접 플래시
// =============================================
// 1 = 패스스루 기능 활성화, 0 = 비활성화
#define CFG_PASSTHRU_ENABLE     1

// RTL GPIO → S3 핀 연결 (보드 배선에 맞게 수정)
// BW16 variant.h 기준 사용 가능 GPIO:
//   AMB_D0(PA7), AMB_D1(PA8), AMB_D2(PA27), AMB_D3(PA30),
//   AMB_D7(PA25), AMB_D8(PA26)
// (AMB_D4/D5=Serial1, AMB_D9~D12=SPI에 사용 중, AMB_D6=PB3/SWD_CLK)
#define CFG_S3_EN_PIN           AMB_D8  // PA26 → S3 EN (RESET)
#define CFG_S3_BOOT_PIN         AMB_D7  // PA25 → S3 GPIO0 (BOOT)

// 패스스루 UART 보드레이트 (esptool 기본값)
#define CFG_PASSTHRU_BAUD       115200

// 패스스루 자동 종료 타임아웃 (ms)
// esptool 통신이 이 시간 동안 없으면 정상 모드로 복귀
#define CFG_PASSTHRU_TIMEOUT_MS 10000

// =============================================
// UART 링크 (RTL ↔ S3)
// =============================================
#define CFG_LINK_BAUD  115200

// BW16 고정 핀 (variant에 의해 결정, 참고용)
// Serial1 TX = AMB_D4 (PB1), Serial1 RX = AMB_D5 (PB2)

// =============================================
// SPI 마스터 (카메라 링크)
// =============================================
// BW16 기본 SPI 핀: SCLK=AMB_D10(PA14), MISO=AMB_D11(PA13), MOSI=AMB_D12(PA12)
// SS는 보드/부트 안정성에 맞춰 변경 가능 (현재: AMB_D3/PA30)
#define CFG_SPI_SS_PIN        AMB_D3  // PA30
#define CFG_SPI_HZ            10000000  // 10MHz
#define CFG_SPI_BLOCK_BYTES   2048      // S3과 동일하게 맞춰야 함

// =============================================
// 카메라 브릿지
// =============================================
#define CFG_ENABLE_CAMERA_BRIDGE  1

// 프레임 버퍼 최대 크기
// RTL8720DN RAM 제약: BD_RAM_NS 영역 한계
// v0.2.1: 더블 버퍼(2개) 적용으로 8KB→6KB 축소
//   QVGA(320x240) JPEG @Q10~12 = 3~5KB (복잡한 장면 ~6KB)
//   6KB 초과 프레임은 자동 드롭 (bounds check 처리)
//   2×6KB=12KB — BD_RAM_NS에 ~1.4KB 여유 확보
// VGA 해상도는 RTL8720DN에서 지원 불가 (RAM 부족)
#define CFG_FRAME_BUF_SIZE    (6 * 1024)  // 6KB (QVGA 전용, 더블 버퍼)

// =============================================
// WebSocket 서버 포트
// =============================================
#define CFG_WS_CONTROL_PORT   80    // ws://ip/ws
#define CFG_WS_CAMERA_PORT    81    // ws://ip:81/

// =============================================
// 성능 로그 간격
// =============================================
#define CFG_STAT_INTERVAL_MS  2000  // 2초마다 FPS/KB 로그

// SPI pull 간격 (WS 클라이언트 없을 때)
#define CFG_SPI_IDLE_PULL_MS  200   // ~5fps
