/*
  config.h — RTL8720DN (DeepCoRTL_Bridge) 설정 파일
  v0.1.1: 하드코딩된 핀/파라미터를 한 곳에서 관리

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

// 프레임 버퍼 최대 크기 (QVGA JPEG 기준)
#define CFG_FRAME_BUF_SIZE    (64 * 1024)  // 64KB

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
