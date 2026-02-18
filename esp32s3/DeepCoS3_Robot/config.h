/*
  config.h — ESP32-S3 (DeepCoS3_Robot) 설정 파일
  v0.1.1: 하드코딩된 핀/파라미터를 한 곳에서 관리

  보드나 PCB가 바뀌면 이 파일만 수정하세요.
*/
#pragma once

// =============================================
// 펌웨어 버전 (GitHub 태그와 반드시 일치시킬 것)
// =============================================
#define CFG_FW_VERSION    "0.2.2"
#define CFG_FW_CHIP       "S3"

// =============================================
// 로그 레벨 설정
// =============================================
// LOG_NONE  (0) = 출력 완전 차단 (배포/생산)
// LOG_INFO  (1) = 핵심 이벤트만 (부팅, 연결, OTA, 에러)
// LOG_DEBUG (2) = 전부 출력 (SPI, 모터, 프레임, 워치독 등)
//
// 런타임 변경: RTL에서 @s3debug,0/1/2 또는 S3 UART에서 @debug,0/1/2
#define LOG_NONE   0
#define LOG_INFO   1
#define LOG_DEBUG  2

#define CFG_LOG_LEVEL   LOG_INFO    // 기본 로그 레벨
#define CFG_LOG_USB     true        // USB Serial(CDC) 출력 활성화 (devkit 단독 테스트 시 true)

// FreeRTOS 태스크 분리 모드 (false면 loop() 폴링)
#define CFG_ENABLE_TASK_SPLIT         true

// =============================================
// UART 링크 (RTL ↔ S3)
// =============================================
#define CFG_LINK_BAUD       115200
#define CFG_LINK_RX_PIN     44      // S3 RX ← RTL TX
#define CFG_LINK_TX_PIN     43      // S3 TX → RTL RX

// =============================================
// SPI 슬레이브 (카메라 → RTL)
// =============================================
#define CFG_SPI_SCLK_PIN    21      // v0.2.1: GPIO45(VDD_SPI strapping)에서 변경
#define CFG_SPI_MOSI_PIN    14      // Master(RTL) → Slave(S3)  — v0.2.1: GPIO20(USB D-)에서 변경
#define CFG_SPI_MISO_PIN    3       // Slave(S3) → Master(RTL)  — v0.2.1: GPIO19(USB D+)에서 변경
#define CFG_SPI_CS_PIN      46      // Slave Select
#define CFG_SPI_BLOCK_BYTES 2048    // DMA 블록 크기
#define CFG_SPI_QUEUE_SIZE  2       // SPI 큐 깊이

// =============================================
// 카메라 핀 (OV2640/OV5640)
// =============================================
#define CFG_CAM_PWDN_PIN    (-1)
#define CFG_CAM_RESET_PIN   (-1)
#define CFG_CAM_XCLK_PIN    15
#define CFG_CAM_SIOD_PIN    4       // I2C SDA
#define CFG_CAM_SIOC_PIN    5       // I2C SCL

#define CFG_CAM_Y2_PIN      11
#define CFG_CAM_Y3_PIN      9
#define CFG_CAM_Y4_PIN      8
#define CFG_CAM_Y5_PIN      10
#define CFG_CAM_Y6_PIN      12
#define CFG_CAM_Y7_PIN      18
#define CFG_CAM_Y8_PIN      17
#define CFG_CAM_Y9_PIN      16

#define CFG_CAM_VSYNC_PIN   6
#define CFG_CAM_HREF_PIN    7
#define CFG_CAM_PCLK_PIN    13

#define CFG_CAM_XCLK_FREQ   20000000  // 20MHz
#define CFG_CAM_JPEG_QUALITY 25       // 1(최고)~63(최저), RTL 버퍼 8KB 이내 유지
#define CFG_CAM_FB_COUNT     2        // 프레임 버퍼 개수

// =============================================
// LED 핀
// =============================================
#define CFG_LED_CENTER_PIN          2
#define CFG_LED_RIGHT_NEOPIXEL_PIN  47
#define CFG_LED_LEFT_NEOPIXEL_PIN   48
#define CFG_LED_PIXELS_PER_SIDE     4
#define CFG_LED_BRIGHTNESS_SIDE     128
#define CFG_LED_BRIGHTNESS_CENTER   255

// =============================================
// 모터 핀 (스텝퍼)
// =============================================
// 왼쪽 모터
#define CFG_MOTOR_L_STEP_PIN  1   //1 <-> 21
#define CFG_MOTOR_L_DIR_PIN   42
#define CFG_MOTOR_L_EN_PIN    41

// 오른쪽 모터
#define CFG_MOTOR_R_STEP_PIN  40
#define CFG_MOTOR_R_DIR_PIN   39
#define CFG_MOTOR_R_EN_PIN    38

// =============================================
// 로봇 물리 파라미터
// =============================================
#define CFG_WHEEL_DIAMETER_MM     33.50f
#define CFG_TRACK_WIDTH_MM        73.5f
#define CFG_STEPS_PER_WHEEL_REV   2000.0f
#define CFG_MAX_SPEED_LIMIT       2000.0f
#define CFG_MOTOR_ACCELERATION    10000
#define CFG_MOTOR_DEFAULT_SPEED   1000
#define CFG_MOTOR_TURN_SLOW_SPEED 700
