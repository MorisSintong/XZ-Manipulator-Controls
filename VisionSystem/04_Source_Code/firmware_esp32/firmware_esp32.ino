/*
 * ==============================================================================
 * FIRMWARE ESP32: PENERIMA KOREKSI SUDUT UART & KONTROL SERVO REORIENTASI
 * Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
 * Husein Alhamid (4212301035)
 * ==============================================================================
 *
 * REVISION NOTES (Remediation v2.0):
 *   - REMOVED all NVS Flash writes from real-time packet loop to prevent
 *     flash memory wear-out. Logs are now stored in volatile RAM only.
 *   - REMOVED all blocking delay() calls. Replaced with non-blocking
 *     millis()-based state machine for LED/servo timing.
 *   - ADDED support for new binary framed UART protocol with CRC16-CCITT
 *     verification (header 0xAA 0x55, 18-byte packet).
 *   - RETAINED legacy ASCII protocol parsing as fallback.
 *   - Documented C struct for STM32F446RE FreeRTOS queue integration.
 *
 * Hardware ESP32:
 * - Pin Servo PWM : GPIO 18 (ke kabel sinyal Servo SG90/MG90S)
 * - LED Indikator :
 *     - GPIO 2  : LED Hijau (Status: PASS / Polaritas Benar)
 *     - GPIO 4  : LED Merah (Status: REORIENTASI / Polaritas Salah)
 * - Komunikasi    : UART via USB (Serial @ 115200 bps, TX=GPIO1, RX=GPIO3)
 *
 * Binary Packet Format (18 bytes):
 *   Byte 0-1  : Header        (0xAA 0x55)
 *   Byte 2    : Message Type   (0x01 = Detection, 0x02 = Heartbeat)
 *   Byte 3-4  : Object ID      (uint16_t, little-endian)
 *   Byte 5    : Class ID       (0 = Non-Elco, 1 = Benar, 2 = Salah)
 *   Byte 6-7  : X position mm  (int16_t * 10, little-endian)
 *   Byte 8-9  : Y position mm  (int16_t * 10, little-endian)
 *   Byte 10-11: Angle deg      (int16_t * 10, little-endian, 0-3600)
 *   Byte 12-13: Servo corr deg (int16_t * 10, little-endian, 0-1800)
 *   Byte 14-15: CRC16-CCITT    (little-endian)
 *   Byte 16-17: Tail           (0x0D 0x0A)
 *
 * Legacy ASCII Format (still supported):
 *   "K<class_id>,S<angle>,R<correction>\n"
 *
 * Serial Monitor Commands:
 *   - "LOG\n"    -> Display runtime log summary (RAM only)
 *   - "CLEAR\n"  -> Clear RAM log
 *   - "STATUS\n" -> Show system status
 *   - "PING\n"   -> Connection test
 *
 * ==============================================================================
 * STM32F446RE FreeRTOS Integration Reference:
 *
 *   // C struct matching the binary packet payload (for xQueueReceive):
 *   typedef struct __attribute__((packed)) {
 *       uint8_t  msg_type;       // 0x01 = Detection, 0x02 = Heartbeat
 *       uint16_t object_id;      // Tracking ID from vision system
 *       uint8_t  class_id;       // 0 = Non-Elco, 1 = Benar, 2 = Salah
 *       int16_t  x_mm_x10;      // X position in mm * 10
 *       int16_t  y_mm_x10;      // Y position in mm * 10
 *       int16_t  angle_x10;     // Orientation angle in deg * 10 (0-3600)
 *       int16_t  servo_corr_x10;// Servo correction in deg * 10 (0-1800)
 *   } VisionPacket_t;
 *
 *   // Usage with FreeRTOS:
 *   // QueueHandle_t xVisionQueue = xQueueCreate(16, sizeof(VisionPacket_t));
 *   // In DMA UART RX callback, parse frame, verify CRC, xQueueSend.
 *   // In motion task, xQueueReceive and execute pick-place sequence.
 *
 * ==============================================================================
 */

#include <ESP32Servo.h>

// =============================================================================
// Pin Configuration
// =============================================================================
#define PIN_SERVO     18    // Servo signal pin
#define PIN_LED_PASS  2     // Green LED (PASS)
#define PIN_LED_FAIL  4     // Red LED (REORIENTASI / FAIL)

// =============================================================================
// Servo
// =============================================================================
Servo servoReorientasi;
const int POS_HOME = 90;  // Neutral home position (degrees)

// =============================================================================
// Non-Blocking State Machine
// =============================================================================
enum ServoState {
  STATE_IDLE,
  STATE_LED_PASS_ON,      // Green LED on, waiting to turn off
  STATE_SERVO_MOVING,     // Servo at correction angle, waiting to return home
};

ServoState currentState = STATE_IDLE;
unsigned long stateStartMs = 0;

// Timing constants (milliseconds) — non-blocking
const unsigned long LED_PASS_DURATION_MS = 300;
const unsigned long SERVO_HOLD_DURATION_MS = 800;

// =============================================================================
// Serial Input Buffer
// =============================================================================
String inputString = "";
bool dataLengkap = false;

// Binary packet ring buffer
#define BIN_BUF_SIZE 64
uint8_t binBuffer[BIN_BUF_SIZE];
int binBufIdx = 0;

// =============================================================================
// RAM-Only Log System (NO Flash writes in real-time loop)
// =============================================================================
#define MAX_LOG_ENTRIES 50

struct LogEntry {
  char perintah[48];  // Received command string
  char respon[64];    // ACK response sent
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
int logCount = 0;
int totalDiterima = 0;
int countPass = 0;
int countReorientasi = 0;
int countReject = 0;

// =============================================================================
// RAM-only log: store entry to circular buffer (NO NVS / NO Flash writes)
// =============================================================================
void simpanLog(const char* cmd, const char* ack) {
  int idx;
  if (logCount < MAX_LOG_ENTRIES) {
    idx = logCount;
    logCount++;
  } else {
    // Circular buffer: shift all entries up (discard oldest)
    for (int i = 0; i < MAX_LOG_ENTRIES - 1; i++) {
      logBuffer[i] = logBuffer[i + 1];
    }
    idx = MAX_LOG_ENTRIES - 1;
  }

  strncpy(logBuffer[idx].perintah, cmd, sizeof(logBuffer[idx].perintah) - 1);
  logBuffer[idx].perintah[sizeof(logBuffer[idx].perintah) - 1] = '\0';
  strncpy(logBuffer[idx].respon, ack, sizeof(logBuffer[idx].respon) - 1);
  logBuffer[idx].respon[sizeof(logBuffer[idx].respon) - 1] = '\0';

  totalDiterima++;
}

// =============================================================================
// Display log summary to Serial Monitor
// =============================================================================
void tampilkanLog() {
  Serial.println();
  Serial.println("================================================================");
  Serial.println("   REKAP LOG PENGUJIAN ESP32 — RAM BUFFER (VOLATILE)");
  Serial.println("================================================================");
  Serial.printf("  Total Paket Diterima : %d\n", totalDiterima);
  Serial.printf("  Log Tersimpan        : %d / %d\n", logCount, MAX_LOG_ENTRIES);
  Serial.printf("  PASS (Benar)         : %d\n", countPass);
  Serial.printf("  REORIENTASI (Salah)  : %d\n", countReorientasi);
  Serial.printf("  REJECT (Bukan Elco)  : %d\n", countReject);
  Serial.printf("  Uptime ESP32         : %lu detik\n", millis() / 1000);
  Serial.println("================================================================");
  Serial.println();

  if (logCount == 0) {
    Serial.println("  (Belum ada data log.)");
    Serial.println("  Jalankan script Python terlebih dahulu,");
    Serial.println("  lalu buka Serial Monitor dan ketik LOG.");
    Serial.println();
    return;
  }

  Serial.println("  No | Perintah Diterima                   | Respon ESP32");
  Serial.println("  ---+----------------------------------------+--------------------------------------------");

  for (int i = 0; i < logCount; i++) {
    Serial.printf("  %3d| %-38s | %s\n",
      i + 1,
      logBuffer[i].perintah,
      logBuffer[i].respon
    );
  }

  Serial.println();
  Serial.println("================================================================");
  Serial.printf("  RINGKASAN: %d PASS | %d REORIENTASI | %d REJECT\n",
    countPass, countReorientasi, countReject);
  Serial.println("================================================================");
  Serial.println("  Ketik CLEAR untuk hapus log | STATUS untuk info");
  Serial.println("================================================================");
  Serial.println();
}

// =============================================================================
// Clear all logs (RAM only — no flash erase needed)
// =============================================================================
void clearLog() {
  logCount = 0;
  totalDiterima = 0;
  countPass = 0;
  countReorientasi = 0;
  countReject = 0;
  Serial.println("[ESP32] Semua log telah dihapus (RAM).");
}

// =============================================================================
// Display system status
// =============================================================================
void tampilkanStatus() {
  Serial.println();
  Serial.println("--- STATUS ESP32 ---");
  Serial.printf("  Uptime       : %lu detik\n", millis() / 1000);
  Serial.printf("  Total Paket  : %d\n", totalDiterima);
  Serial.printf("  Log Buffer   : %d / %d\n", logCount, MAX_LOG_ENTRIES);
  Serial.printf("  PASS         : %d\n", countPass);
  Serial.printf("  REORIENTASI  : %d\n", countReorientasi);
  Serial.printf("  REJECT       : %d\n", countReject);
  Serial.printf("  Free Heap    : %d bytes\n", ESP.getFreeHeap());
  Serial.printf("  Storage      : RAM only (no flash wear)\n");
  Serial.printf("  State Machine: %s\n",
    currentState == STATE_IDLE ? "IDLE" :
    currentState == STATE_LED_PASS_ON ? "LED_PASS" : "SERVO_MOVING");
  Serial.println("--------------------");
  Serial.println("  LOG   -> Lihat rekap lengkap");
  Serial.println("  CLEAR -> Hapus semua log");
  Serial.println("--------------------");
  Serial.println();
}

// =============================================================================
// CRC16-CCITT calculation (must match Python side)
// =============================================================================
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc & 0xFFFF;
}

// =============================================================================
// Process a verified binary detection packet (14 payload bytes, after CRC ok)
// =============================================================================
void processBinaryPacket(const uint8_t* pkt) {
  // Parse fields (little-endian)
  uint8_t  msgType   = pkt[2];
  uint16_t objectId  = pkt[3] | (pkt[4] << 8);
  uint8_t  classId   = pkt[5];
  int16_t  xRaw      = (int16_t)(pkt[6] | (pkt[7] << 8));
  int16_t  yRaw      = (int16_t)(pkt[8] | (pkt[9] << 8));
  int16_t  angleRaw  = (int16_t)(pkt[10] | (pkt[11] << 8));
  int16_t  servoRaw  = (int16_t)(pkt[12] | (pkt[13] << 8));

  float x_mm   = xRaw / 10.0f;
  float y_mm   = yRaw / 10.0f;
  float angle  = angleRaw / 10.0f;
  float koreksi = servoRaw / 10.0f;

  // Handle heartbeat
  if (msgType == 0x02) {
    Serial.printf("ACK:HEARTBEAT,ID:%u\n", objectId);
    return;
  }

  // Detection packet
  char ackMsg[80] = "";
  char cmdStr[48] = "";

  snprintf(cmdStr, sizeof(cmdStr), "BIN:ID%u,K%u,X%.1f,Y%.1f,A%.1f,R%.1f",
    objectId, classId, x_mm, y_mm, angle, koreksi);

  // Only act if state machine is idle (non-blocking)
  if (currentState != STATE_IDLE) {
    snprintf(ackMsg, sizeof(ackMsg), "ACK:ID%u,STATUS:BUSY", objectId);
    Serial.println(ackMsg);
    simpanLog(cmdStr, ackMsg);
    return;
  }

  if (classId == 1 && koreksi < 1.0f) {
    // === ELCO BENAR (PASS) ===
    digitalWrite(PIN_LED_PASS, HIGH);
    digitalWrite(PIN_LED_FAIL, LOW);
    servoReorientasi.write(POS_HOME);

    snprintf(ackMsg, sizeof(ackMsg),
      "ACK:ID%u,K%u,STATUS:PASS,SERVO:DIAM,X:%.1f,Y:%.1f",
      objectId, classId, x_mm, y_mm);
    Serial.println(ackMsg);
    countPass++;

    // Start non-blocking LED timer
    currentState = STATE_LED_PASS_ON;
    stateStartMs = millis();

  } else if (classId == 2 || koreksi >= 1.0f) {
    // === ELCO SALAH (REORIENTASI) ===
    digitalWrite(PIN_LED_PASS, LOW);
    digitalWrite(PIN_LED_FAIL, HIGH);

    int targetServo = POS_HOME;
    if (koreksi >= 135.0f) {
      targetServo = 180;
    } else if (koreksi <= -135.0f) {
      targetServo = 0;
    } else {
      targetServo = constrain(POS_HOME + (int)koreksi, 0, 180);
    }

    servoReorientasi.write(targetServo);
    snprintf(ackMsg, sizeof(ackMsg),
      "ACK:ID%u,K%u,STATUS:REORI,KOR:%.1f,SERVO:%d,X:%.1f,Y:%.1f",
      objectId, classId, koreksi, targetServo, x_mm, y_mm);
    Serial.println(ackMsg);
    countReorientasi++;

    // Start non-blocking servo hold timer
    currentState = STATE_SERVO_MOVING;
    stateStartMs = millis();

  } else {
    // === BUKAN ELCO / REJECT ===
    digitalWrite(PIN_LED_PASS, LOW);
    digitalWrite(PIN_LED_FAIL, LOW);
    snprintf(ackMsg, sizeof(ackMsg),
      "ACK:ID%u,K%u,STATUS:REJECT", objectId, classId);
    Serial.println(ackMsg);
    countReject++;
  }

  // Store to RAM log (NO flash writes)
  simpanLog(cmdStr, ackMsg);
}

// =============================================================================
// Try to find and process a binary frame in the ring buffer
// Returns true if a valid frame was consumed
// =============================================================================
bool tryParseBinaryFrame() {
  // Need at least 18 bytes for a complete frame
  while (binBufIdx >= 18) {
    // Search for header 0xAA 0x55
    if (binBuffer[0] != 0xAA || binBuffer[1] != 0x55) {
      // Shift buffer by 1 byte (discard)
      memmove(binBuffer, binBuffer + 1, binBufIdx - 1);
      binBufIdx--;
      continue;
    }

    // Check tail delimiter
    if (binBuffer[16] != 0x0D || binBuffer[17] != 0x0A) {
      // Bad frame, skip this header
      memmove(binBuffer, binBuffer + 1, binBufIdx - 1);
      binBufIdx--;
      continue;
    }

    // Verify CRC16 over bytes 0-13 (header + payload, 14 bytes)
    uint16_t calcCrc = crc16_ccitt(binBuffer, 14);
    uint16_t pktCrc = binBuffer[14] | (binBuffer[15] << 8);

    if (calcCrc != pktCrc) {
      Serial.printf("ERR:CRC_MISMATCH,CALC:%04X,PKT:%04X\n", calcCrc, pktCrc);
      memmove(binBuffer, binBuffer + 1, binBufIdx - 1);
      binBufIdx--;
      continue;
    }

    // Valid frame! Process it.
    processBinaryPacket(binBuffer);

    // Consume the 18 bytes
    memmove(binBuffer, binBuffer + 18, binBufIdx - 18);
    binBufIdx -= 18;
    return true;
  }
  return false;
}

// =============================================================================
// Process legacy ASCII packet (K<id>,S<angle>,R<correction>)
// =============================================================================
void prosesPaketLegacy(String paket) {
  // --- Serial Monitor Commands ---
  if (paket.equalsIgnoreCase("PING") || paket.equalsIgnoreCase("TEST")) {
    Serial.println("ACK:PONG,ESP32_CONNECTED");
    return;
  }
  if (paket.equalsIgnoreCase("LOG") || paket.equalsIgnoreCase("REPORT")) {
    tampilkanLog();
    return;
  }
  if (paket.equalsIgnoreCase("CLEAR") || paket.equalsIgnoreCase("RESET")) {
    clearLog();
    return;
  }
  if (paket.equalsIgnoreCase("STATUS") || paket.equalsIgnoreCase("INFO")) {
    tampilkanStatus();
    return;
  }

  // --- Legacy data format: K%d,S%f,R%f ---
  int class_id = -1;
  float sudut_act = 0.0;
  float koreksi = 0.0;

  int parsed = sscanf(paket.c_str(), "K%d,S%f,R%f", &class_id, &sudut_act, &koreksi);

  if (parsed >= 3) {
    char ackMsg[64] = "";

    // Only act if state machine is idle (non-blocking)
    if (currentState != STATE_IDLE) {
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:BUSY", class_id);
      Serial.println(ackMsg);
      char cmdStr[32];
      strncpy(cmdStr, paket.c_str(), sizeof(cmdStr) - 1);
      cmdStr[sizeof(cmdStr) - 1] = '\0';
      simpanLog(cmdStr, ackMsg);
      return;
    }

    if (class_id == 1 && abs(koreksi) < 1.0) {
      // === ELCO BENAR (PASS) ===
      digitalWrite(PIN_LED_PASS, HIGH);
      digitalWrite(PIN_LED_FAIL, LOW);
      servoReorientasi.write(POS_HOME);

      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:PASS,SERVO:DIAM(0deg)", class_id);
      Serial.println(ackMsg);
      countPass++;

      // Non-blocking LED timer
      currentState = STATE_LED_PASS_ON;
      stateStartMs = millis();

    } else if (class_id == 2 || abs(koreksi) >= 1.0) {
      // === ELCO SALAH (REORIENTASI) ===
      digitalWrite(PIN_LED_PASS, LOW);
      digitalWrite(PIN_LED_FAIL, HIGH);

      int targetServo = POS_HOME;
      if (koreksi >= 135.0) {
        targetServo = 180;
      } else if (koreksi <= -135.0) {
        targetServo = 0;
      } else if (koreksi > 0) {
        targetServo = constrain(POS_HOME + (int)koreksi, 0, 180);
      } else {
        targetServo = constrain(POS_HOME + (int)koreksi, 0, 180);
      }

      servoReorientasi.write(targetServo);
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:REORIENTASI,KOR:%.1f,SERVO:%d",
        class_id, koreksi, targetServo);
      Serial.println(ackMsg);
      countReorientasi++;

      // Non-blocking servo hold timer
      currentState = STATE_SERVO_MOVING;
      stateStartMs = millis();

    } else {
      // === BUKAN ELCO / REJECT ===
      digitalWrite(PIN_LED_PASS, LOW);
      digitalWrite(PIN_LED_FAIL, LOW);
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:REJECT", class_id);
      Serial.println(ackMsg);
      countReject++;
    }

    // Store to RAM log (NO flash writes)
    char cmdStr[32];
    strncpy(cmdStr, paket.c_str(), sizeof(cmdStr) - 1);
    cmdStr[sizeof(cmdStr) - 1] = '\0';
    simpanLog(cmdStr, ackMsg);

  } else {
    Serial.printf("ERR:FORMAT_INVALID,RAW:%s\n", paket.c_str());
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // Initialize UART Serial to PC
  Serial.begin(115200);
  // NOTE: No delay() here — use non-blocking approach
  unsigned long setupStart = millis();
  while (millis() - setupStart < 500) {
    // Brief non-blocking wait for serial stability
    yield();
  }

  // Initialize LED pins
  pinMode(PIN_LED_PASS, OUTPUT);
  pinMode(PIN_LED_FAIL, OUTPUT);
  digitalWrite(PIN_LED_PASS, LOW);
  digitalWrite(PIN_LED_FAIL, LOW);

  // Initialize Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoReorientasi.setPeriodHertz(50);
  servoReorientasi.attach(PIN_SERVO, 500, 2400);
  servoReorientasi.write(POS_HOME);

  // Allocate string buffer
  inputString.reserve(64);

  Serial.println("=================================================");
  Serial.println("[ESP32 READY] Receiver Koreksi UART v2.0 Aktif");
  Serial.println("Baudrate: 115200 bps | Pin Servo: GPIO 18");
  Serial.println("Protocol: Binary (CRC16) + Legacy ASCII");
  Serial.println("Storage: RAM only (no flash wear)");
  Serial.println("=================================================");
  Serial.println("Perintah Serial Monitor:");
  Serial.println("  LOG    -> Lihat rekap hasil pengujian");
  Serial.println("  CLEAR  -> Hapus semua log");
  Serial.println("  STATUS -> Info status ESP32");
  Serial.println("  PING   -> Uji koneksi");
  Serial.println("=================================================");
}

// =============================================================================
// MAIN LOOP — Fully Non-Blocking
// =============================================================================
void loop() {
  // ─── 1. Read serial data (non-blocking) ───
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();

    // Feed into binary frame buffer
    if (binBufIdx < BIN_BUF_SIZE) {
      binBuffer[binBufIdx++] = inByte;
    } else {
      // Buffer overflow — reset
      binBufIdx = 0;
      binBuffer[0] = inByte;
      binBufIdx = 1;
    }

    // Also feed into ASCII line buffer
    char inChar = (char)inByte;
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        dataLengkap = true;
      }
    } else {
      inputString += inChar;
    }
  }

  // ─── 2. Try to parse binary frames first ───
  if (binBufIdx >= 18) {
    if (tryParseBinaryFrame()) {
      // Successfully parsed a binary frame — clear ASCII buffer
      // since it was binary data, not ASCII
      inputString = "";
      dataLengkap = false;
    }
  }

  // ─── 3. Process ASCII line if complete ───
  if (dataLengkap) {
    inputString.trim();
    prosesPaketLegacy(inputString);
    inputString = "";
    dataLengkap = false;
  }

  // ─── 4. Non-blocking state machine for LED/servo timing ───
  unsigned long now = millis();

  switch (currentState) {
    case STATE_IDLE:
      // Nothing to do
      break;

    case STATE_LED_PASS_ON:
      // Turn off green LED after duration
      if (now - stateStartMs >= LED_PASS_DURATION_MS) {
        digitalWrite(PIN_LED_PASS, LOW);
        currentState = STATE_IDLE;
      }
      break;

    case STATE_SERVO_MOVING:
      // Return servo to home and turn off red LED after hold duration
      if (now - stateStartMs >= SERVO_HOLD_DURATION_MS) {
        servoReorientasi.write(POS_HOME);
        digitalWrite(PIN_LED_FAIL, LOW);
        currentState = STATE_IDLE;
      }
      break;
  }
}
