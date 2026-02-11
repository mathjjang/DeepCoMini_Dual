#ifndef DCM_SPI_PROTOCOL_H
#define DCM_SPI_PROTOCOL_H
// WARNING: Keep this file identical in both RTL and S3 folders.
// Update both copies together when protocol fields/constants change.

#include <stddef.h>
#include <stdint.h>

#pragma pack(push, 1)
struct DcmSpiHdr {
  char magic[4];        // "DCM2"
  uint8_t type;         // 0=idle, 1=jpeg
  uint8_t flags;        // bit0=start, bit1=end
  uint16_t seq;         // frame seq
  uint32_t total_len;   // JPEG total length
  uint32_t offset;      // payload offset in frame
  uint16_t payload_len; // payload bytes in this block
  uint16_t crc16;       // payload CRC16-CCITT (XModem poly)
};
#pragma pack(pop)

static constexpr uint8_t SPI_TYPE_IDLE     = 0;
static constexpr uint8_t SPI_TYPE_JPEG     = 1;
static constexpr uint8_t SPI_TYPE_OTA      = 2;  // v0.1.9: RTL→S3 OTA data
static constexpr uint8_t SPI_TYPE_OTA_ACK  = 3;  // v0.1.9: S3→RTL OTA ack
static constexpr uint8_t SPI_FLAG_START    = 0x01;
static constexpr uint8_t SPI_FLAG_END      = 0x02;
static constexpr uint8_t SPI_FLAG_OTA_ERR  = 0x04;  // v0.1.9: S3 OTA error

static inline uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0x0000; // XModem/CCITT initial value
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

#endif
