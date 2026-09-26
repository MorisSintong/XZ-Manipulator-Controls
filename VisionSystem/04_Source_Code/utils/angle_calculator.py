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
    1. Segment the capacitor body using adaptive thresholding (tries both
       BINARY and BINARY_INV to handle varying backgrounds).
    2. Compute principal axis orientation via minAreaRect → continuous [0°, 180°).
    3. Determine polarity (lead pin vs body base) via brightness centroid
       shift along the major axis → extend to [0°, 360°).
    """
    if roi_bgr.size == 0 or roi_bgr.shape[0] < 5 or roi_bgr.shape[1] < 5:
        return 0.0

    h_roi, w_roi = roi_bgr.shape[:2]
    roi_area = h_roi * w_roi

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Try both threshold polarities and pick the one with better contour
    best_cnt = None
    best_rect = None
    best_aspect = 0.0

    for thresh_type in [cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
                        cv2.THRESH_BINARY + cv2.THRESH_OTSU]:
        _, thresh = cv2.threshold(blurred, 0, 255, thresh_type)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        # Reject contours that fill >90% of ROI (likely background detection)
        if area > 0.9 * roi_area:
            continue
        # Reject tiny contours (<5% of ROI)
        if area < 0.05 * roi_area:
            continue

        rect = cv2.minAreaRect(cnt)
        (_, _), (rw, rh), _ = rect
        if min(rw, rh) < 1:
            continue

        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
        # Prefer the contour with higher aspect ratio (more elongated = better
        # capacitor body detection)
        if aspect > best_aspect:
            best_aspect = aspect
            best_cnt = cnt
            best_rect = rect

    if best_rect is None:
        # Fallback: use PCA directly on the grayscale image moments
        # Compute image moments and use the orientation
        moments = cv2.moments(gray)
        if moments['mu20'] + moments['mu02'] > 0:
            angle_pca = 0.5 * math.atan2(2 * moments['mu11'],
                                          moments['mu20'] - moments['mu02'])
            angle_deg = math.degrees(angle_pca) % 180.0
            # Convert from math convention to system convention
            sudut_360 = (90.0 - angle_deg + 360.0) % 360.0
            return round(sudut_360, 2)
        return 0.0

    (cx, cy), (rw, rh), angle_rect = best_rect

    # ─── Langkah 1: Sumbu geometris 0°–180° ───
    # minAreaRect returns the angle of the WIDTH side relative to horizontal.
    # We want the angle of the LONG axis.
    if rw < rh:
        # Width is shorter → the long side is 'h', perpendicular to the angle
        angle_geom = (angle_rect + 90.0) % 180.0
        length = rh
    else:
        angle_geom = angle_rect % 180.0
        length = rw

    # Handle negative angles from minAreaRect
    if angle_geom < 0:
        angle_geom += 180.0

    # ─── Langkah 2: Deteksi posisi kaki → perluas ke 0°–360° ───
    # Sample brightness at two points INSIDE the body (not at the tips,
    # which bleed into background). Use 60% of half-length from center.
    rad = math.radians(angle_geom)
    sample_dist = (length / 2.0) * 0.6  # Stay well inside the body
    dx = math.cos(rad) * sample_dist
    dy = math.sin(rad) * sample_dist  # positive = downward in screen coords

    # Point 1: in the direction of angle_geom (60% from center)
    pt1_x = int(np.clip(cx + dx, 0, w_roi - 1))
    pt1_y = int(np.clip(cy + dy, 0, h_roi - 1))

    # Point 2: opposite direction (60% from center)
    pt2_x = int(np.clip(cx - dx, 0, w_roi - 1))
    pt2_y = int(np.clip(cy - dy, 0, h_roi - 1))

    # Use a tight patch to avoid background bleed
    patch_sz = max(2, int(length * 0.08))

    def get_brightness(x, y, img, ps):
        x1 = max(0, x - ps)
        x2 = min(w_roi, x + ps + 1)
        y1 = max(0, y - ps)
        y2 = min(h_roi, y + ps + 1)
        patch = img[y1:y2, x1:x2]
        return float(np.mean(patch)) if patch.size > 0 else 0.0

    b1 = get_brightness(pt1_x, pt1_y, gray, patch_sz)
    b2 = get_brightness(pt2_x, pt2_y, gray, patch_sz)

    # The lead pin side is BRIGHTER (metallic reflective pins).
    # angle_geom points toward pt1. If pt1 is the pin side, use angle_geom.
    # If pt2 is brighter, add 180° (pin is on the opposite side).
    if b2 > b1:
        math_angle = (angle_geom + 180.0) % 360.0
    else:
        math_angle = angle_geom

    # ─── Konversi ke konvensi sistem ───
    # OpenCV math angle: 0° = Right (horizontal), increases CW in screen coords
    # System convention: 0° = Down, 90° = Right, 180° = Up, 270° = Left
    # Mapping: system_angle = (90 - math_angle) mod 360
    # BUT screen Y is inverted, so the sign convention means:
    #   math 0° (right) → system 90°  ✓
    #   math 90° (down in screen) → system 0° ✓
    #   math 180° (left) → system 270° ✓
    #   math 270° (up in screen) → system 180° ✓
    sudut_360 = (90.0 - math_angle + 360.0) % 360.0

    return round(sudut_360, 2)


def evaluasi_qc_dan_servo(class_id: int, class_name: str, roi_bgr: np.ndarray = None, sudut_manual: float = None):
    """
    Menentukan status QC, deviasi sudut, koreksi putaran micro-servo (CW),
    serta format framing UART ke ESP32.
    """
    nama_lower = class_name.lower()
    sudut_ref = 0.0

    is_elco = ("benar" in nama_lower) or ("salah" in nama_lower)

    if not is_elco:
        # ─── Bukan Elco → Reject ───
        sudut_act = 0.0
        dev = 0.0
        kor_servo = 0.0
        arah_servo = "DIAM"
        status_qc = "BUKAN ELCO (REJECT)"
        aksi = "ABAIKAN / REJECT"
        uart_msg = f"K{class_id},S0.0,R0.0\n"
        return sudut_act, sudut_ref, dev, kor_servo, arah_servo, status_qc, aksi, uart_msg

    if sudut_manual is not None:
        sudut_deteksi = sudut_manual
    else:
        if roi_bgr is not None and roi_bgr.size > 0:
            sudut_deteksi = hitung_sudut_kontur(roi_bgr)
        else:
            sudut_deteksi = 0.0

    sudut_act, dev, kor_servo, arah_servo = hitung_koreksi_terpendek(sudut_deteksi, sudut_ref)

    if abs(dev) < 15.0:
        status_qc = "BENAR (SESUAI REFERENSI)"
        aksi = "LANJUT KE JIG (PASS)"
    else:
        status_qc = "SALAH (PERLU REORIENTASI)"
        aksi = f"PUTAR SERVO CW {kor_servo:.1f}°"

    uart_msg = f"K{class_id},S{sudut_act:.1f},R{kor_servo:.1f}\n"

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
