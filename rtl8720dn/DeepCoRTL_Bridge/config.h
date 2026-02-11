/*
  config.h — RTL8720DN (DeepCoRTL_Bridge) 설정 파일
  v0.1.1: 하드코딩된 핀/파라미터를 한 곳에서 관리
  v0.1.5: 옵션 E (RTL 하이브리드) — S3 패스스루 플래시 GPIO 추가

  Wi-Fi 채널/비밀번호는 시리얼 명령으로도 변경 가능 (v0.1.1+)
*/
#pragma once

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
// BW16 사용 가능 GPIO: D0(PB0), D1(PB3), D6(PA26), D7(PA25), D8(PA27)
// (D4/D5는 Serial1, D9~D12는 SPI에 사용 중)
#define CFG_S3_EN_PIN           D6    // PA26 → S3 EN (RESET)
#define CFG_S3_BOOT_PIN         D7    // PA25 → S3 GPIO0 (BOOT)

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
// Serial1 TX = D4 (PB1), Serial1 RX = D5 (PB2)

// =============================================
// SPI 마스터 (카메라 링크)
// =============================================
// BW16 고정 핀: SS=D9(PA15), SCLK=D10(PA14), MISO=D11(PA13), MOSI=D12(PA12)
#define CFG_SPI_SS_PIN        D9    // PA15
#define CFG_SPI_HZ            30000000  // 30MHz (배선 품질에 따라 조정)
#define CFG_SPI_BLOCK_BYTES   2048      // S3과 동일하게 맞춰야 함

// =============================================
// 카메라 브릿지
// =============================================
#define CFG_ENABLE_CAMERA_BRIDGE  1

// 프레임 버퍼 최대 크기
// v0.1.2: 64KB→128KB로 확대 (VGA JPEG도 수용 가능)
#define CFG_FRAME_BUF_SIZE    (128 * 1024)  // 128KB

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
