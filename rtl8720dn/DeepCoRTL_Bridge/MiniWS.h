/*
  MiniWS.h — Minimal WebSocket server/client for Ameba RTL8720DN
  
  외부 라이브러리 없이 Ameba 내장 WiFiServer/WiFiClient만 사용.
  SHA-1, Base64 자체 구현 (mbedtls 링크 불필요).
  
  지원 기능:
  - WebSocket 서버 (listen/accept)
  - Text/Binary 메시지 송수신
  - Ping/Pong 자동 처리
  - 연결 해제 감지 (콜백)
  
  v2: String 사용 최소화 (char[] 기반 handshake/readLine),
      헤더 라인 길이 상한, Upgrade/Connection 헤더 검증 추가
*/
#pragma once
#include <WiFi.h>

// ============================================================
// SHA-1 (WebSocket handshake 전용, self-contained)
// ============================================================
static void _ws_sha1(const uint8_t* msg, size_t len, uint8_t digest[20]) {
  uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE;
  uint32_t h3 = 0x10325476, h4 = 0xC3D2E1F0;

  // Padding: msg + 0x80 + zeros + 8-byte big-endian bit length
  size_t padLen = ((len + 8) / 64 + 1) * 64;
  uint8_t pad[192]; // WS key+magic is ~60 bytes -> padLen <= 128
  if (padLen > sizeof(pad)) return;
  memset(pad, 0, padLen);
  memcpy(pad, msg, len);
  pad[len] = 0x80;
  uint64_t bits = (uint64_t)len * 8;
  for (int i = 0; i < 8; i++) pad[padLen - 1 - i] = (uint8_t)(bits >> (i * 8));

  for (size_t off = 0; off < padLen; off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
      w[i] = ((uint32_t)pad[off + i * 4] << 24) | ((uint32_t)pad[off + i * 4 + 1] << 16) |
             ((uint32_t)pad[off + i * 4 + 2] << 8) | pad[off + i * 4 + 3];
    }
    for (int i = 16; i < 80; i++) {
      uint32_t t = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
      w[i] = (t << 1) | (t >> 31);
    }
    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    for (int i = 0; i < 80; i++) {
      uint32_t f, k;
      if      (i < 20) { f = (b & c) | (~b & d);           k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d;                     k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d);  k = 0x8F1BBCDC; }
      else              { f = b ^ c ^ d;                     k = 0xCA62C1D6; }
      uint32_t t = ((a << 5) | (a >> 27)) + f + e + k + w[i];
      e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = t;
    }
    h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
  }
  uint32_t hh[5] = {h0, h1, h2, h3, h4};
  for (int i = 0; i < 5; i++) {
    digest[i * 4]     = (hh[i] >> 24) & 0xFF;
    digest[i * 4 + 1] = (hh[i] >> 16) & 0xFF;
    digest[i * 4 + 2] = (hh[i] >> 8) & 0xFF;
    digest[i * 4 + 3] = hh[i] & 0xFF;
  }
}

// ============================================================
// Base64 encoder (WebSocket handshake 전용)
// ============================================================
static void _ws_base64(const uint8_t* in, size_t inLen, char* out) {
  static const char T[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t i = 0, j = 0;
  for (; i + 2 < inLen; i += 3) {
    uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
    out[j++] = T[(v >> 18) & 0x3F];
    out[j++] = T[(v >> 12) & 0x3F];
    out[j++] = T[(v >> 6) & 0x3F];
    out[j++] = T[v & 0x3F];
  }
  if (i < inLen) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < inLen) v |= (uint32_t)in[i + 1] << 8;
    out[j++] = T[(v >> 18) & 0x3F];
    out[j++] = T[(v >> 12) & 0x3F];
    out[j++] = (i + 1 < inLen) ? T[(v >> 6) & 0x3F] : '=';
    out[j++] = '=';
  }
  out[j] = '\0';
}

// ============================================================
// Internal string helpers (case-insensitive, no heap)
// ============================================================
static inline char _ws_lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c;
}

// Case-insensitive prefix match: does `str` start with `prefix`?
static bool _ws_prefixEq(const char* str, const char* prefix) {
  while (*prefix) {
    if (_ws_lower(*str) != _ws_lower(*prefix)) return false;
    str++; prefix++;
  }
  return true;
}

// Case-insensitive substring search: does `haystack` contain `needle`?
static bool _ws_containsEq(const char* haystack, const char* needle) {
  size_t nLen = strlen(needle);
  size_t hLen = strlen(haystack);
  if (nLen > hLen) return false;
  for (size_t i = 0; i <= hLen - nLen; i++) {
    if (_ws_prefixEq(haystack + i, needle)) return true;
  }
  return false;
}

// Trim trailing whitespace in-place
static void _ws_trimEnd(char* s) {
  int len = (int)strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' ||
         s[len - 1] == '\r' || s[len - 1] == '\n')) {
    s[--len] = '\0';
  }
}

// ============================================================
// MiniWsClient — WebSocket 클라이언트 (서버에서 accept 후 사용)
// ============================================================
class MiniWsClient {
public:
  // Callback types (전역/static 함수 포인터, 캡처 불필요)
  typedef void (*TextMsgCb)(const String& msg);
  typedef void (*CloseCb)();

  MiniWsClient() : _connected(false), _onMsg(nullptr), _onClose(nullptr) {}

  void setCallbacks(TextMsgCb msgCb, CloseCb closeCb) {
    _onMsg = msgCb;
    _onClose = closeCb;
  }

  bool available() {
    if (!_connected) return false;
    if (!_tcp.connected()) {
      _connected = false;
      if (_onClose) _onClose();
      return false;
    }
    return true;
  }

  void poll() {
    if (!available()) return;
    // Process all available incoming frames
    while (_tcp.available() >= 2) {
      if (!_readFrame()) break;
    }
  }

  bool send(const char* text) {
    return _sendFrame(0x01, (const uint8_t*)text, strlen(text));
  }
  bool send(const String& s) { return send(s.c_str()); }

  bool sendBinary(const char* data, size_t len) {
    return _sendFrame(0x02, (const uint8_t*)data, len);
  }

  void close() {
    if (_connected) {
      _sendFrame(0x08, nullptr, 0);
      delay(10);
      _tcp.stop();
      _connected = false;
      if (_onClose) _onClose();
    }
  }

  // Server's accept() 에서 호출 (handshake 성공 후)
  void _initFromServer(const WiFiClient& tcp) {
    _tcp = tcp;
    _connected = true;
    _onMsg = nullptr;
    _onClose = nullptr;
  }

private:
  WiFiClient _tcp;
  bool _connected;
  TextMsgCb _onMsg;
  CloseCb _onClose;

  // ---- WebSocket frame send (server->client: no mask) ----
  bool _sendFrame(uint8_t opcode, const uint8_t* data, size_t len) {
    if (!_tcp.connected()) return false;
    uint8_t hdr[10];
    size_t hl = 2;
    hdr[0] = 0x80 | opcode; // FIN + opcode
    if (len < 126) {
      hdr[1] = (uint8_t)len;
    } else if (len < 65536) {
      hdr[1] = 126;
      hdr[2] = (uint8_t)(len >> 8);
      hdr[3] = (uint8_t)(len);
      hl = 4;
    } else {
      hdr[1] = 127;
      hdr[2] = 0; hdr[3] = 0; hdr[4] = 0; hdr[5] = 0;
      hdr[6] = (uint8_t)(len >> 24);
      hdr[7] = (uint8_t)(len >> 16);
      hdr[8] = (uint8_t)(len >> 8);
      hdr[9] = (uint8_t)(len);
      hl = 10;
    }
    _tcp.write(hdr, hl);
    if (data && len > 0) {
      // Send in chunks to avoid buffer overflow on large frames
      const size_t CHUNK = 1024;
      size_t sent = 0;
      while (sent < len) {
        size_t n = (len - sent > CHUNK) ? CHUNK : (len - sent);
        _tcp.write(data + sent, n);
        sent += n;
      }
    }
    return true;
  }

  // ---- WebSocket frame read (client->server: masked) ----
  bool _readFrame() {
    if (_tcp.available() < 2) return false;

    uint8_t b0 = _tcp.read();
    uint8_t b1 = _tcp.read();
    uint8_t op = b0 & 0x0F;
    bool masked = (b1 & 0x80) != 0;
    size_t plen = b1 & 0x7F;

    if (plen == 126) {
      if (!_waitBytes(2)) return false;
      plen = ((size_t)_tcp.read() << 8) | _tcp.read();
    } else if (plen == 127) {
      if (!_waitBytes(8)) return false;
      plen = 0;
      for (int i = 0; i < 8; i++) plen = (plen << 8) | _tcp.read();
    }

    uint8_t mask[4] = {0};
    if (masked) {
      if (!_waitBytes(4)) return false;
      for (int i = 0; i < 4; i++) mask[i] = _tcp.read();
    }

    // Guard against oversized frames
    if (plen > 32768) {
      size_t skip = plen;
      while (skip > 0 && _tcp.connected()) {
        if (_tcp.available()) { _tcp.read(); skip--; }
        else delay(1);
      }
      return true;
    }

    if (!_waitBytes(plen)) return false;

    if (op == 0x01) { // Text — char[] 기반 수신 (String 파편화 방지)
      static const size_t MAX_TEXT_MSG = 512;
      if (plen > MAX_TEXT_MSG) {
        // 초과 텍스트 프레임 → 스킵
        for (size_t i = 0; i < plen; i++) _tcp.read();
        return true;
      }
      char tbuf[MAX_TEXT_MSG + 1];
      for (size_t i = 0; i < plen; i++) {
        uint8_t c = _tcp.read();
        if (masked) c ^= mask[i & 3];
        tbuf[i] = (char)c;
      }
      tbuf[plen] = '\0';
      if (_onMsg) {
        String s(tbuf); // 콜백 호출용 최소 수명 String
        _onMsg(s);
      }
    } else if (op == 0x08) { // Close
      for (size_t i = 0; i < plen; i++) _tcp.read(); // drain
      close();
    } else if (op == 0x09) { // Ping -> Pong
      uint8_t buf[125];
      size_t n = (plen < sizeof(buf)) ? plen : sizeof(buf);
      for (size_t i = 0; i < plen; i++) {
        uint8_t c = _tcp.read();
        if (masked) c ^= mask[i & 3];
        if (i < n) buf[i] = c;
      }
      _sendFrame(0x0A, buf, n);
    } else { // Pong or unknown -- skip payload
      for (size_t i = 0; i < plen; i++) _tcp.read();
    }
    return true;
  }

  bool _waitBytes(size_t n) {
    uint32_t t = millis();
    while ((size_t)_tcp.available() < n) {
      if (millis() - t > 2000 || !_tcp.connected()) return false;
      delay(1);
    }
    return true;
  }
};

// ============================================================
// MiniWsServer — WebSocket 서버 (listen -> poll -> accept)
// ============================================================
class MiniWsServer {
public:
  MiniWsServer() : _srv(80), _listening(false) {}

  void listen(uint16_t port) {
    _srv = WiFiServer(port);
    _srv.begin();
    _listening = true;
  }

  // Check for new TCP connections (call every loop iteration)
  bool poll() {
    if (!_listening) return false;
    _pending = _srv.available();
    return (_pending && _pending.connected());
  }

  bool available() {
    return (_pending && _pending.connected());
  }

  // Accept: perform WebSocket handshake and return client
  MiniWsClient accept() {
    MiniWsClient ws;
    if (!_pending || !_pending.connected()) return ws;

    WiFiClient tcp = _pending;
    _pending = WiFiClient(); // clear

    if (_doHandshake(tcp)) {
      ws._initFromServer(tcp);
    } else {
      tcp.stop();
    }
    return ws;
  }

private:
  WiFiServer _srv;
  WiFiClient _pending;
  bool _listening;

  // 라인 길이 상한: 악성/오류 클라이언트의 무한 헤더 방지
  static const size_t MAX_HEADER_LINE = 256;

  // ---- HTTP 헤더 라인 읽기 (char[] 기반, 길이 상한 적용) ----
  // 반환: 읽은 바이트 수, -1 = 길이 초과 또는 타임아웃
  static int _readLine(WiFiClient& cl, char* buf, size_t bufSz) {
    size_t pos = 0;
    uint32_t t = millis();
    while (cl.connected() && millis() - t < 2000) {
      if (cl.available()) {
        char c = cl.read();
        if (c == '\n') break;
        if (c != '\r') {
          if (pos < bufSz - 1) {
            buf[pos++] = c;
          } else {
            // 라인 길이 초과 → handshake 거부
            return -1;
          }
        }
      } else {
        delay(1);
      }
    }
    buf[pos] = '\0';
    return (int)pos;
  }

  // ---- WebSocket HTTP upgrade handshake (char[] 기반 + 헤더 검증) ----
  bool _doHandshake(WiFiClient& cl) {
    char lineBuf[MAX_HEADER_LINE];
    char wsKey[64] = {0};
    bool hasUpgrade   = false;
    bool hasConnection = false;
    uint32_t t = millis();

    // Read HTTP headers (timeout 3s)
    while (cl.connected() && millis() - t < 3000) {
      if (!cl.available()) { delay(1); continue; }
      int len = _readLine(cl, lineBuf, sizeof(lineBuf));
      if (len < 0) return false;  // 라인 길이 초과 → 거부
      if (len == 0) break;        // 빈 줄 = 헤더 종료

      // 필수 헤더 파싱 (case-insensitive)
      if (_ws_prefixEq(lineBuf, "Sec-WebSocket-Key: ")) {
        // wsKey 추출 (19바이트 접두어 이후)
        size_t valLen = strlen(lineBuf + 19);
        if (valLen > 0 && valLen < sizeof(wsKey)) {
          memcpy(wsKey, lineBuf + 19, valLen + 1);
          _ws_trimEnd(wsKey);
        }
      } else if (_ws_prefixEq(lineBuf, "Upgrade:")) {
        if (_ws_containsEq(lineBuf + 8, "websocket")) hasUpgrade = true;
      } else if (_ws_prefixEq(lineBuf, "Connection:")) {
        if (_ws_containsEq(lineBuf + 11, "upgrade")) hasConnection = true;
      }
    }

    // 필수 조건 검증: Sec-WebSocket-Key + Upgrade: websocket + Connection: Upgrade
    if (wsKey[0] == '\0' || !hasUpgrade || !hasConnection) return false;

    // Compute Sec-WebSocket-Accept: SHA1(key + magic) → Base64
    static const char magic[] = "258EAFA5-E914-47DA-95CA-5AB5B14F89E3";
    char toHash[128];
    size_t keyLen = strlen(wsKey);
    if (keyLen + sizeof(magic) > sizeof(toHash)) return false;
    memcpy(toHash, wsKey, keyLen);
    memcpy(toHash + keyLen, magic, sizeof(magic)); // null 포함

    uint8_t sha[20];
    _ws_sha1((const uint8_t*)toHash, keyLen + sizeof(magic) - 1, sha);

    char b64[32]; // SHA1=20 bytes → Base64=28 chars + null
    _ws_base64(sha, 20, b64);

    // Send HTTP 101 Switching Protocols
    cl.print("HTTP/1.1 101 Switching Protocols\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             "Sec-WebSocket-Accept: ");
    cl.print(b64);
    cl.print("\r\n\r\n");

    return true;
  }
};
