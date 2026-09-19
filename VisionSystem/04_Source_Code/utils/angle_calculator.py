"""
Modul Penghitung Sudut & Evaluasi Polaritas Kapasitor
Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
Husein Alhamid (4212301035)
"""

import math
import cv2
import numpy as np

# Definisi Warna BGR
COLOR_BENAR   = (34, 197, 94)    # Hijau (Sesuai Referensi / Target)
COLOR_SALAH   = (38, 38, 239)    # Merah (Polaritas Terbalik / Reorientasi)
COLOR_NONELCO = (0, 165, 255)    # Oranye (Bukan Elco / Reject)


def normalisasi_sudut_360(sudut: float) -> float:
    """
    Normalisasi sudut sembarang (termasuk negatif seperti -270°, atau >360°)
    ke dalam rentang standar [0.0°, 360.0°).
    Contoh: -270° -> 90.0°, -90° -> 270.0°, 450° -> 90.0°
    """
    return round((float(sudut) % 360.0 + 360.0) % 360.0, 2)


def hitung_koreksi_terpendek(sudut_aktual: float, target_ref: float = 0.0) -> tuple:
    """
    Menghitung sudut deviasi dan koreksi putaran servo SEARAH JARUM JAM (CW).

    Konvensi:
    - Koreksi selalu >= 0 (CW / searah jarum jam)
    - Koreksi  = 0   : Posisi Sesuai (DIAM)
    - Koreksi  = 90  : Putar CW 90°
    - Koreksi  = 180 : Putar CW 180°
    - Koreksi  = 270 : Putar CW 270°

    Deviasi = seberapa jauh posisi aktual dari referensi (bisa ±).
    """
    sudut_norm = normalisasi_sudut_360(sudut_aktual)
    ref_norm = normalisasi_sudut_360(target_ref)

    # Deviasi dari referensi (untuk pelaporan, range [-180, +180])
    deviasi = (sudut_norm - ref_norm + 180.0) % 360.0 - 180.0

    # Koreksi servo selalu CW (searah jarum jam)
    # = berapa derajat CW dari posisi aktual kembali ke referensi
    koreksi = (ref_norm - sudut_norm + 360.0) % 360.0

    if abs(koreksi) < 1e-2 or abs(koreksi - 360.0) < 1e-2:
        arah = "DIAM"
        koreksi = 0.0
    else:
        arah = f"PUTAR CW {koreksi:.1f}°"

    return round(sudut_norm, 2), round(deviasi, 2), round(koreksi, 2), arah


def hitung_sudut_kontur(roi_bgr: np.ndarray) -> float:
    """
    Menghitung sudut orientasi polaritas kapasitor dalam range [0°, 360°).

    Konvensi sudut (CW dari bawah, sesuai tampilan layar):
        0°   = Kaki di bawah (polaritas BENAR / referensi)
        90°  = Kaki di kanan
        180° = Kaki di atas (terbalik)
        270° = Kaki di kiri

    Strategi:
    1. Hitung sumbu geometris bodi kapasitor via minAreaRect (0°–180°)
    2. Deteksi posisi kaki (area terang/pin) untuk menentukan kuadran → 0°–360°
    """
    if roi_bgr.size == 0 or roi_bgr.shape[0] < 5 or roi_bgr.shape[1] < 5:
        return 0.0

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h_roi, w_roi = roi_bgr.shape[:2]

    if not contours:
        # Fallback: estimasi dari aspect ratio
        return 0.0 if h_roi > w_roi else 90.0

    cnt = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(cnt)  # ((cx, cy), (w, h), angle)
    (rcx, rcy), (rw, rh), angle_rect = rect

    # ─── Langkah 1: Sumbu geometris 0°–180° ───
    if rw < rh:
        angle_geom = (angle_rect + 90.0) % 180.0
    else:
        angle_geom = angle_rect % 180.0

    # ─── Langkah 2: Deteksi posisi kaki → perluas ke 0°–360° ───
    # Kaki kapasitor = pin metalik tipis yang memiliki banyak tepi (edges).
    # Gunakan Canny edge detection untuk mendeteksi sisi mana yang memiliki
    # kepadatan edge tertinggi = sisi kaki.
    # (Brightness analysis tidak reliable karena marking putih di bodi
    #  bisa lebih terang daripada kaki tipis)

    edges = cv2.Canny(blurred, 50, 150)

    mid_y = h_roi // 2
    mid_x = w_roi // 2
    # Ambil strip ~25% dari tepi untuk menghindari bodi di tengah
    strip_y = max(1, h_roi // 4)
    strip_x = max(1, w_roi // 4)

    # Hitung kepadatan edge di tiap zona tepi
    zona_bawah = np.sum(edges[h_roi - strip_y:h_roi, :])   # kaki di bawah → 0°
    zona_atas  = np.sum(edges[0:strip_y, :])                # kaki di atas  → 180°
    zona_kanan = np.sum(edges[:, w_roi - strip_x:w_roi])    # kaki di kanan → 90°
    zona_kiri  = np.sum(edges[:, 0:strip_x])                # kaki di kiri  → 270°

    # Tentukan apakah orientasi vertikal atau horizontal berdasarkan geometri
    is_vertikal = (45.0 <= angle_geom <= 135.0)

    if is_vertikal:
        # Sumbu utama vertikal → kaki bisa di atas (180°) atau bawah (0°)
        if zona_bawah >= zona_atas:
            sudut_360 = 0.0    # kaki di bawah = BENAR
        else:
            sudut_360 = 180.0  # kaki di atas = TERBALIK
    else:
        # Sumbu utama horizontal → kaki bisa di kanan (90°) atau kiri (270°)
        if zona_kanan >= zona_kiri:
            sudut_360 = 90.0   # kaki di kanan
        else:
            sudut_360 = 270.0  # kaki di kiri

    return round(sudut_360, 2)


def evaluasi_qc_dan_servo(class_id: int, class_name: str, roi_bgr: np.ndarray = None, sudut_manual: float = None):
    """
    Menentukan status QC, deviasi sudut, koreksi putaran micro-servo (CW),
    serta format framing UART ke ESP32.

    Logika:
    - Elco_Benar & Elco_Salah: Analisis sudut fisik dari ROI (atau sudut_manual)
      lalu hitung koreksi CW ke referensi 0° (kaki di bawah).
    - Bukan_Elco: Reject tanpa koreksi.

    Koreksi servo selalu searah jarum jam (CW, nilai positif).
    """
    nama_lower = class_name.lower()
    sudut_ref = 0.0

    if sudut_manual is not None:
        # ─── Mode manual: sudut diberikan langsung ───
        sudut_act, dev, kor_servo, arah_servo = hitung_koreksi_terpendek(sudut_manual, sudut_ref)

        if "benar" in nama_lower and abs(dev) < 15.0:
            status_qc = "BENAR (SESUAI REFERENSI)"
            aksi = "LANJUT KE JIG (PASS)"
        elif "salah" in nama_lower or abs(dev) >= 15.0:
            status_qc = "SALAH (PERLU REORIENTASI)"
            aksi = f"PUTAR SERVO CW {kor_servo:.1f}°"
        else:
            status_qc = "BUKAN ELCO (REJECT)"
            aksi = "ABAIKAN / REJECT"
            sudut_act, dev, kor_servo = 0.0, 0.0, 0.0
            arah_servo = "DIAM"

        uart_msg = f"K{class_id},S{sudut_act:.1f},R{kor_servo:.1f}\n"

    elif "benar" in nama_lower or "salah" in nama_lower:
        # ─── Mode otomatis: deteksi sudut dari ROI citra ───
        if roi_bgr is not None and roi_bgr.size > 0:
            sudut_deteksi = hitung_sudut_kontur(roi_bgr)
        else:
            # Fallback: jika ROI tidak tersedia, estimasi dari nama kelas
            sudut_deteksi = 0.0 if "benar" in nama_lower else 180.0

        sudut_act, dev, kor_servo, arah_servo = hitung_koreksi_terpendek(sudut_deteksi, sudut_ref)

        if abs(dev) < 15.0:
            status_qc = "BENAR (SESUAI REFERENSI)"
            aksi = "LANJUT KE JIG (PASS)"
        else:
            status_qc = "SALAH (PERLU REORIENTASI)"
            aksi = f"PUTAR SERVO CW {kor_servo:.1f}°"

        uart_msg = f"K{class_id},S{sudut_act:.1f},R{kor_servo:.1f}\n"

    else:
        # ─── Bukan Elco → Reject ───
        sudut_act = 0.0
        dev = 0.0
        kor_servo = 0.0
        arah_servo = "DIAM"
        status_qc = "BUKAN ELCO (REJECT)"
        aksi = "ABAIKAN / REJECT"
        uart_msg = f"K{class_id},S0.0,R0.0\n"

    return sudut_act, sudut_ref, dev, kor_servo, arah_servo, status_qc, aksi, uart_msg


def gambar_anotasi(
    image: np.ndarray,
    box: tuple,
    class_name: str,
    conf: float,
    sudut_aktual: float,
    deviasi: float,
    koreksi_servo: float,
    status_qc: str,
    aksi: str,
    sudut_ref: float = 0.0
) -> np.ndarray:
    """
    Menggambar bounding box, panah referensi (selalu ke bawah 0°),
    panah polaritas aktual, serta overlay informasi Quality Control.
    """
    img = image.copy()
    x1, y1, x2, y2 = map(int, box)
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    nama_lower = class_name.lower()
    if "benar" in nama_lower:
        warna = COLOR_BENAR
    elif "salah" in nama_lower:
        warna = COLOR_SALAH
    else:
        warna = COLOR_NONELCO

    # Bounding box
    cv2.rectangle(img, (x1, y1), (x2, y2), warna, 2)
    corner_len = 10
    for px, py, dx, dy in [(x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)]:
        cv2.line(img, (px, py), (px + dx * corner_len, py), warna, 3)
        cv2.line(img, (px, py), (px, py + dy * corner_len), warna, 3)

    # Proyeksi Trigonometri Vektor Panah (0° = Bawah, 90° = Kanan, 180° = Atas, 270° = Kiri)
    h_box = y2 - y1
    w_box = x2 - x1
    panjang_vektor = int(max(24, min(w_box, h_box) * 0.55))

    # 1. Panah Referensi Target (0.0° = Mengarah ke Bawah)
    # Tampilkan panah referensi pembanding (Cyan) terutama jika ada deviasi/salah
    if "salah" in nama_lower or abs(deviasi) >= 15.0:
        rad_ref = math.radians(normalisasi_sudut_360(sudut_ref))
        dx_ref = math.sin(rad_ref)
        dy_ref = math.cos(rad_ref)
        end_ref = (int(cx + dx_ref * panjang_vektor), int(cy + dy_ref * panjang_vektor))
        color_ref = (255, 230, 0)  # Cyan/Kuning cerah (BGR)
        cv2.arrowedLine(img, (cx, cy), end_ref, color_ref, 2, tipLength=0.3, line_type=cv2.LINE_AA)
        cv2.putText(img, "Ref: 0 (Bawah)", (end_ref[0] + 4, end_ref[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color_ref, 1, cv2.LINE_AA)

    # 2. Panah Polaritas Aktual Objek
    rad_act = math.radians(normalisasi_sudut_360(sudut_aktual))
    dx_act = math.sin(rad_act)
    dy_act = math.cos(rad_act)
    end_pt = (int(cx + dx_act * panjang_vektor), int(cy + dy_act * panjang_vektor))

    # Titik pusat pivot objek
    cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1, lineType=cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 2, (0, 0, 0), -1, lineType=cv2.LINE_AA)
    # Garis panah polaritas aktual
    cv2.arrowedLine(img, (cx, cy), end_pt, warna, 3, tipLength=0.35, line_type=cv2.LINE_AA)

    # Overlay Info
    teks_list = [
        f"{class_name} ({conf*100:.1f}%)",
        f"Status  : {status_qc}",
        f"Ref     : {sudut_ref:.1f} deg (Bawah)",
        f"Sudut/R : {sudut_aktual:.1f} deg | Kor CW: {koreksi_servo:.1f} deg",
        f"Aksi    : {aksi}"
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 1
    line_h = 18

    max_w = max(cv2.getTextSize(t, font, font_scale, thickness)[0][0] for t in teks_list) + 12
    box_h = len(teks_list) * line_h + 8

    bg_y1 = max(0, y1 - box_h)
    bg_y2 = bg_y1 + box_h
    bg_x1 = max(0, x1)
    bg_x2 = min(img.shape[1], bg_x1 + max_w)

    sub_img = img[bg_y1:bg_y2, bg_x1:bg_x2]
    if sub_img.shape[0] > 0 and sub_img.shape[1] > 0:
        dark_rect = np.zeros(sub_img.shape, dtype=np.uint8)
        dark_rect[:] = (15, 15, 15)
        res = cv2.addWeighted(sub_img, 0.2, dark_rect, 0.8, 0)
        img[bg_y1:bg_y2, bg_x1:bg_x2] = res

    for idx, t in enumerate(teks_list):
        t_y = bg_y1 + (idx + 1) * line_h - 3
        c_text = warna if idx == 0 else (255, 255, 255)
        cv2.putText(img, t, (bg_x1 + 6, t_y), font, font_scale, c_text, thickness, cv2.LINE_AA)

    return img
