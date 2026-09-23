/*
 * ==============================================================================
 * FIRMWARE ESP32: PENERIMA KOREKSI SUDUT UART & KONTROL SERVO REORIENTASI
 * Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
 * Husein Alhamid (4212301035)
 * ==============================================================================
 * 
 * Hardware ESP32:
 * - Pin Servo PWM : GPIO 18 (ke kabel sinyal Servo SG90/MG90S)
 * - LED Indikator : 
 *     - GPIO 2  : LED Hijau (Status: PASS / Polaritas Benar)
 *     - GPIO 4  : LED Merah (Status: REORIENTASI / Polaritas Salah)
 * - Komunikasi    : UART via USB (Serial @ 115200 bps, TX=GPIO1, RX=GPIO3)
 * 
 * Protokol Paket UART dari PC (Python):
 * Format: "K<class_id>,S<sudut_aktual>,R<koreksi_servo>\n"
 * Contoh:
 *   - "K1,S0.0,R0.0\n"       -> Elco Benar, Servo DIAM (0°), PASS ke Jig
 *   - "K2,S180.0,R180.0\n"   -> Elco Salah, Servo PUTAR 180°, Reorientasi
 *   - "K2,S90.0,R-90.0\n"    -> Elco Salah, Servo PUTAR CCW 90°
 *   - "K0,S0.0,R0.0\n"       -> Bukan Elco (Reject)
 * 
 * Perintah Serial Monitor:
 *   - "LOG\n"    -> Tampilkan seluruh rekap log hasil pengujian
 *   - "CLEAR\n"  -> Hapus semua log tersimpan
 *   - "STATUS\n" -> Tampilkan status ringkas
 *   - "PING\n"   -> Uji koneksi
 *   - "SAVE\n"   -> Simpan log dari RAM ke NVS
 *   - "LOAD\n"   -> Muat log dari NVS ke RAM
 * 
 * Packet struct for STM32 FreeRTOS integration:
 * typedef struct __attribute__((packed)) {
 *     uint8_t  header[2];    // 0xAA, 0x55
 *     uint8_t  msg_type;     // 0x01=Detection, 0x02=Heartbeat
 *     uint16_t obj_id;       // Tracking sequence number
 *     uint8_t  class_id;     // 0=NonElco, 1=Benar, 2=Salah
 *     int16_t  x_mm;         // Conveyor X in 0.1mm units
 *     int16_t  y_mm;         // Conveyor Y in 0.1mm units
 *     int16_t  angle_deg10;  // Orientation * 10 (0-3600)
 *     int16_t  correction_deg10; // Servo correction * 10
 *     uint16_t crc16;        // CRC16-CCITT over bytes 0-13
 *     uint8_t  tail[2];      // 0x0D, 0x0A
 * } VisionPacket_t;
 * ==============================================================================
 */

#include <ESP32Servo.h>
#include <Preferences.h>

// Konfigurasi Pin
#define PIN_SERVO     18    // Pin sinyal micro-servo
#define PIN_LED_PASS  2     // Pin LED Hijau (Built-in LED / Eksternal)
#define PIN_LED_FAIL  4     // Pin LED Merah (Indikator Koreksi)

// Objek Servo
Servo servoReorientasi;

// Posisi Netral (Home) Servo SG90 (Rentang 0 - 180 derajat)
const int POS_HOME = 90;    // Titik tengah netral 90 derajat

// State Machine Variables
enum ServoState { STATE_IDLE, STATE_LED_ON, STATE_SERVO_ACTIVE, STATE_RETURNING };
ServoState currentState = STATE_IDLE;
unsigned long stateStartMs = 0;
unsigned long stateDurationMs = 0;

// Buffer Serial ASCII
String inputString = "";
bool dataLengkap = false;

// Buffer Serial Binary
uint8_t binBuffer[18];
int binIndex = 0;
bool receivingBinary = false;

// ============================================================================
// SISTEM LOG — NVS (Non-Volatile Storage) + RAM Buffer
// ============================================================================
Preferences preferences;

#define MAX_LOG_ENTRIES 50   // Maksimal log di NVS (hemat flash wear)

// Struct log di RAM (untuk akses cepat)
struct LogEntry {
  char perintah[32];    // Paket UART yang diterima
  char respon[64];      // Respon ACK yang dikirim
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
int logCount = 0;         // Jumlah log saat ini
int totalDiterima = 0;    // Total paket yang pernah diterima
int countPass = 0;
int countReorientasi = 0;
int countReject = 0;

// CRC16-CCITT
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// ─── Fungsi NVS: Simpan counter ke flash ───
void simpanCounterNVS() {
  preferences.begin("qclog", false);  // namespace "qclog", read-write
  preferences.putInt("total", totalDiterima);
  preferences.putInt("pass", countPass);
  preferences.putInt("reori", countReorientasi);
  preferences.putInt("reject", countReject);
  preferences.putInt("logCount", logCount);
  preferences.end();
}

// ─── Fungsi NVS: Simpan 1 entry log ke flash ───
void simpanEntryNVS(int idx) {
  preferences.begin("qclog", false);
  String keyCmd = "cmd" + String(idx);
  String keyAck = "ack" + String(idx);
  preferences.putString(keyCmd.c_str(), logBuffer[idx].perintah);
  preferences.putString(keyAck.c_str(), logBuffer[idx].respon);
  preferences.end();
}

// ─── Fungsi NVS: Muat semua log dari flash saat boot ───
void muatLogDariNVS() {
  preferences.begin("qclog", true);  // read-only
  totalDiterima = preferences.getInt("total", 0);
  countPass = preferences.getInt("pass", 0);
  countReorientasi = preferences.getInt("reori", 0);
  countReject = preferences.getInt("reject", 0);
  logCount = preferences.getInt("logCount", 0);
  
  // Batasi logCount agar tidak melebihi buffer
  if (logCount > MAX_LOG_ENTRIES) logCount = MAX_LOG_ENTRIES;
  
  for (int i = 0; i < logCount; i++) {
    String keyCmd = "cmd" + String(i);
    String keyAck = "ack" + String(i);
    String cmd = preferences.getString(keyCmd.c_str(), "");
    String ack = preferences.getString(keyAck.c_str(), "");
    strncpy(logBuffer[i].perintah, cmd.c_str(), sizeof(logBuffer[i].perintah) - 1);
    logBuffer[i].perintah[sizeof(logBuffer[i].perintah) - 1] = '\0';
    strncpy(logBuffer[i].respon, ack.c_str(), sizeof(logBuffer[i].respon) - 1);
    logBuffer[i].respon[sizeof(logBuffer[i].respon) - 1] = '\0';
  }
  preferences.end();
  Serial.println("[ESP32] Log dimuat dari NVS.");
}

// ─── Fungsi NVS: Hapus semua log dari flash ───
void hapusLogNVS() {
  preferences.begin("qclog", false);
  preferences.clear();
  preferences.end();
}

// ─── Fungsi simpan log ke NVS (Manual) ───
void simpanKeNVS() {
  for (int i = 0; i < logCount; i++) {
    simpanEntryNVS(i);
  }
  simpanCounterNVS();
  Serial.println("[ESP32] Log berhasil disimpan ke NVS.");
}

// ─── Fungsi simpan log ke buffer RAM ───
void simpanLog(const char* cmd, const char* ack) {
  int idx;
  if (logCount < MAX_LOG_ENTRIES) {
    idx = logCount;
    logCount++;
  } else {
    // Buffer penuh: geser semua ke atas (hapus yang paling lama)
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

// ─── Fungsi tampilkan log ke Serial Monitor ───
void tampilkanLog() {
  Serial.println();
  Serial.println("================================================================");
  Serial.println("   REKAP LOG PENGUJIAN ESP32 — SERIAL MONITOR");
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

  Serial.println("  No | Perintah Diterima        | Respon ESP32");
  Serial.println("  ---+--------------------------+--------------------------------------------");

  for (int i = 0; i < logCount; i++) {
    Serial.printf("  %3d| %-24s | %s\n",
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

// ─── Fungsi hapus semua log ───
void clearLog() {
  logCount = 0;
  totalDiterima = 0;
  countPass = 0;
  countReorientasi = 0;
  countReject = 0;
  hapusLogNVS();
  Serial.println("[ESP32] Semua log telah dihapus (RAM + NVS).");
}

// ─── Fungsi tampilkan status ───
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
  Serial.printf("  Storage      : RAM (Gunakan SAVE untuk simpan ke NVS)\n");
  Serial.println("--------------------");
  Serial.println("  LOG   -> Lihat rekap lengkap");
  Serial.println("  CLEAR -> Hapus semua log");
  Serial.println("  SAVE  -> Simpan ke NVS");
  Serial.println("  LOAD  -> Muat dari NVS");
  Serial.println("--------------------");
  Serial.println();
}

void setup() {
  // Inisialisasi UART Serial ke PC
  Serial.begin(115200);
  delay(500);

  // Inisialisasi Pin LED
  pinMode(PIN_LED_PASS, OUTPUT);
  pinMode(PIN_LED_FAIL, OUTPUT);
  digitalWrite(PIN_LED_PASS, LOW);
  digitalWrite(PIN_LED_FAIL, LOW);

  // Inisialisasi Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoReorientasi.setPeriodHertz(50);
  servoReorientasi.attach(PIN_SERVO, 500, 2400);
  servoReorientasi.write(POS_HOME);

  // Alokasi buffer string
  inputString.reserve(64);

  Serial.println("=================================================");
  Serial.println("[ESP32 READY] Receiver Koreksi UART YOLO26 Aktif");
  Serial.println("Baudrate: 115200 bps | Pin Servo: GPIO 18");
  Serial.println("Ketik LOAD untuk restore data log sebelumnya.");
  Serial.println("=================================================");
  Serial.println("Perintah Serial Monitor:");
  Serial.println("  LOG    -> Lihat rekap hasil pengujian");
  Serial.println("  CLEAR  -> Hapus semua log");
  Serial.println("  STATUS -> Info status ESP32");
  Serial.println("  PING   -> Uji koneksi");
  Serial.println("  SAVE   -> Simpan log ke NVS");
  Serial.println("  LOAD   -> Muat log dari NVS");
  Serial.println("=================================================");
}

void processDetection(int class_id, float koreksi, String raw_cmd) {
    char ackMsg[64] = "";

    if (class_id == 1 && abs(koreksi) < 1.0) {
      // === KASUS 1: ELCO BENAR (PASS) ===
      digitalWrite(PIN_LED_PASS, HIGH);
      digitalWrite(PIN_LED_FAIL, LOW);
      servoReorientasi.write(POS_HOME);
      
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:PASS,SERVO:DIAM(0deg)", class_id);
      Serial.println(ackMsg);
      countPass++;
      
      // State Machine
      currentState = STATE_LED_ON;
      stateStartMs = millis();
      stateDurationMs = 300;

    } else if (class_id == 2 || abs(koreksi) >= 1.0) {
      // === KASUS 2: ELCO SALAH (REORIENTASI) ===
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
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:REORIENTASI,KOR:%.1f,SERVO:%d", class_id, koreksi, targetServo);
      Serial.println(ackMsg);
      countReorientasi++;

      // State Machine
      currentState = STATE_SERVO_ACTIVE;
      stateStartMs = millis();
      stateDurationMs = 800;

    } else {
      // === KASUS 3: BUKAN ELCO / REJECT ===
      digitalWrite(PIN_LED_PASS, LOW);
      digitalWrite(PIN_LED_FAIL, LOW);
      snprintf(ackMsg, sizeof(ackMsg), "ACK:K%d,STATUS:REJECT", class_id);
      Serial.println(ackMsg);
      countReject++;
    }

    // Simpan log ke RAM
    char cmdStr[32];
    strncpy(cmdStr, raw_cmd.c_str(), sizeof(cmdStr) - 1);
    cmdStr[sizeof(cmdStr) - 1] = '\0';
    simpanLog(cmdStr, ackMsg);
}

void prosesPaketKoreksi(String paket) {
  // ─── Perintah Serial Monitor ───
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
  if (paket.equalsIgnoreCase("SAVE")) {
    simpanKeNVS();
    return;
  }
  if (paket.equalsIgnoreCase("LOAD")) {
    muatLogDariNVS();
    return;
  }

  // ─── Format data koreksi ASCII: K%d,S%f,R%f ───
  int class_id = -1;
  float sudut_act = 0.0;
  float koreksi = 0.0;

  int parsed = sscanf(paket.c_str(), "K%d,S%f,R%f", &class_id, &sudut_act, &koreksi);

  if (parsed >= 3) {
    processDetection(class_id, koreksi, paket);
  } else {
    Serial.printf("ERR:FORMAT_INVALID,RAW:%s\n", paket.c_str());
  }
}

void loop() {
  // Baca data serial UART dari PC secara non-blocking
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();

    if (!receivingBinary) {
      if (inByte == 0xAA) {
        // Potensi awal packet biner
        receivingBinary = true;
        binBuffer[0] = inByte;
        binIndex = 1;
      } else {
        // Proses sebagai ASCII
        char inChar = (char)inByte;
        if (inChar == '\n' || inChar == '\r') {
          if (inputString.length() > 0) {
            dataLengkap = true;
          }
        } else {
          inputString += inChar;
        }
      }
    } else {
      // Sedang menerima biner
      binBuffer[binIndex++] = inByte;

      // Cek apakah paket tidak valid 0xAA 0x55
      if (binIndex == 2 && binBuffer[1] != 0x55) {
        receivingBinary = false; // Batal, mungkin hanya 0xAA random
        // Kembalikan ke ASCII buffer jika perlu (opsional), untuk sekarang reset
        binIndex = 0;
      }

      if (binIndex >= 18) {
        // Selesai baca 18 bytes
        receivingBinary = false;
        binIndex = 0;
        
        uint16_t crc_calc = crc16_ccitt(binBuffer, 14);
        uint16_t crc_pkt = binBuffer[14] | (binBuffer[15] << 8);

        if (crc_calc == crc_pkt && binBuffer[16] == 0x0D && binBuffer[17] == 0x0A) {
          if (binBuffer[2] == 0x01) { // Detection
            int class_id = binBuffer[5];
            int16_t correction_raw = binBuffer[12] | (binBuffer[13] << 8);
            float koreksi = correction_raw / 10.0;
            String raw_cmd = "BIN_PACKET";
            processDetection(class_id, koreksi, raw_cmd);
          }
        }
      }
    }
  }

  // Jika paket baris data selesai diterima
  if (dataLengkap) {
    inputString.trim();
    if (inputString.length() > 0) {
      prosesPaketKoreksi(inputString);
    }
    inputString = "";
    dataLengkap = false;
  }

  // State Machine
  if (currentState != STATE_IDLE) {
    if (millis() - stateStartMs >= stateDurationMs) {
      if (currentState == STATE_LED_ON) {
        digitalWrite(PIN_LED_PASS, LOW);
        currentState = STATE_IDLE;
      } else if (currentState == STATE_SERVO_ACTIVE) {
        servoReorientasi.write(POS_HOME);
        digitalWrite(PIN_LED_FAIL, LOW);
        currentState = STATE_IDLE;
      }
    }
  }
}
