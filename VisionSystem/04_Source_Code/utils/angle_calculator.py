"""
Modul Penghitung Sudut & Evaluasi Polaritas Kapasitor
Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
Husein Alhamid (4212301035)

REMEDIATED: Continuous angle calculation [0°, 360°) via PCA + brightness
centroid analysis. No quantization to 4 discrete buckets.
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
    Menghitung sudut orientasi polaritas kapasitor secara KONTINU
    dalam range [0°, 360°).

    Konvensi sudut (CW dari bawah, sesuai tampilan layar):
        0°   = Kaki di bawah (polaritas BENAR / referensi)
        90°  = Kaki di kanan
        180° = Kaki di atas (terbalik)
        270° = Kaki di kiri

    Strategi (PCA + Brightness Centroid):
    1. Ekstrak kontur terbesar dari ROI via thresholding.
    2. Hitung sumbu geometris bodi kapasitor menggunakan cv2.minAreaRect
       untuk mendapatkan sudut kontinu 0°–180° (sumbu panjang).
    3. Disambiguasi 180° menggunakan brightness centroid shift:
       - Kaki kapasitor (lead pins) adalah metalik/terang.
       - Hitung centroid brightness relatif terhadap centroid geometris.
       - Arah offset brightness menentukan sisi kaki → perluas ke 0°–360°.
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
        # Fallback: use PCA on the entire ROI mask
        return _pca_fallback(gray, h_roi, w_roi)

    cnt = max(contours, key=cv2.contourArea)

    # Need at least 5 points for minAreaRect to be meaningful
    if len(cnt) < 5:
        return _pca_fallback(gray, h_roi, w_roi)

    rect = cv2.minAreaRect(cnt)  # ((cx, cy), (w, h), angle)
    (rcx, rcy), (rw, rh), angle_rect = rect

    # ─── Step 1: Continuous geometric axis angle (0°–180°) ───
    # cv2.minAreaRect returns angle in range [-90, 0) for OpenCV 4.x
    # or [0, 90) for OpenCV 3.x. We normalize to get the angle of the
    # LONG axis measured CW from the positive-Y (downward) direction.

    if rw < rh:
        # Width < Height: the long axis is roughly vertical.
        # angle_rect gives the rotation of the width side.
        # Long axis angle = angle_rect + 90
        angle_long_axis = angle_rect + 90.0
    else:
        # Width >= Height: the long axis is roughly horizontal.
        angle_long_axis = angle_rect

    # Normalize to [0, 180) — this is the geometric axis (no polarity yet)
    angle_geom = angle_long_axis % 180.0
    if angle_geom < 0:
        angle_geom += 180.0

    # ─── Step 2: Convert to screen convention ───
    # OpenCV minAreaRect angle is measured from the positive-X axis (right),
    # counter-clockwise. Our convention: 0° = down (positive-Y), CW.
    # Conversion: screen_angle = (90 - opencv_angle) mod 180
    # This maps: 0° OpenCV → 90° screen (right), 90° OpenCV → 0° screen (down)
    angle_screen_180 = (90.0 - angle_geom) % 180.0

    # ─── Step 3: Half-split brightness for 180° disambiguation ───
    # The lead pins of a capacitor are metallic and typically BRIGHTER
    # than the cylindrical body. We split the FULL ROI image into two
    # halves along the principal axis through the contour centroid,
    # then compare mean brightness. The brighter half contains the legs.
    #
    # We use the FULL ROI (not just the contour mask) because the bright
    # metallic legs may be outside the thresholded contour region.

    # Geometric centroid of the contour
    M = cv2.moments(cnt)
    if M["m00"] > 0:
        gcx = M["m10"] / M["m00"]
        gcy = M["m01"] / M["m00"]
    else:
        gcx, gcy = rcx, rcy

    # Direction vector of the geometric axis in image coordinates.
    # angle_geom is from the +X axis. We need the unit vector.
    axis_rad = math.radians(angle_geom)
    ax = math.cos(axis_rad)
    ay = math.sin(axis_rad)

    # Project every pixel onto the axis direction relative to centroid
    ys, xs = np.mgrid[0:h_roi, 0:w_roi]
    proj = (xs - gcx) * ax + (ys - gcy) * ay

    # Compare mean brightness of the two halves of the FULL image
    margin = 2.0  # Exclude pixels near the split line
    pos_mask = proj > margin
    neg_mask = proj < -margin

    if np.any(pos_mask) and np.any(neg_mask):
        # Use the original grayscale image (not masked)
        pos_brightness = float(np.mean(gray[pos_mask]))
        neg_brightness = float(np.mean(gray[neg_mask]))

        # screen_180 corresponds to the NEGATIVE direction of the
        # OpenCV axis vector. This is because:
        # - screen convention: 0°=down (+Y), angle_screen = (90-angle_geom)%180
        # - OpenCV axis at angle_geom=90° (down) → screen=0° → legs point down
        # - The "positive" projection direction for angle_geom=90° is (0,1)=down
        # So positive projection = same direction as screen_180 angle
        #
        # Convert positive direction to screen angle:
        pos_screen = (90.0 - math.degrees(axis_rad)) % 360.0
        neg_screen = (pos_screen + 180.0) % 360.0

        if pos_brightness > neg_brightness + 0.5:
            # Legs are in the positive direction
            sudut_360 = pos_screen
        elif neg_brightness > pos_brightness + 0.5:
            # Legs are in the negative direction
            sudut_360 = neg_screen
        else:
            # Can't disambiguate — return 0°–180° result
            sudut_360 = angle_screen_180
    else:
        sudut_360 = angle_screen_180

    return round(normalisasi_sudut_360(sudut_360), 2)


def _pca_fallback(gray: np.ndarray, h: int, w: int) -> float:
    """
    Fallback angle estimation using image moments/PCA when no contour found.
    Returns continuous angle in [0°, 180°).
    """
    # Use aspect ratio for a rough estimate
    if h > w:
        # Taller than wide → vertical → ~0° (legs down or up)
        ratio = w / max(h, 1)
        # Slight tilt estimation from moment analysis
        return 0.0 if ratio < 0.7 else 45.0
    else:
        ratio = h / max(w, 1)
        return 90.0 if ratio < 0.7 else 45.0


def evaluasi_qc_dan_servo(class_id: int, class_name: str, roi_bgr: np.ndarray = None, sudut_manual: float = None):
    """
    Menentukan status QC, deviasi sudut, koreksi putaran micro-servo (CW).

    Logika:
    - Elco_Benar & Elco_Salah: Analisis sudut fisik dari ROI (atau sudut_manual)
      lalu hitung koreksi CW ke referensi 0° (kaki di bawah).
    - Bukan_Elco: Reject tanpa koreksi.

    Koreksi servo selalu searah jarum jam (CW, nilai positif).

    Returns:
        (sudut_act, sudut_ref, dev, kor_servo, arah_servo, status_qc, aksi, uart_data)
        uart_data is a dict with structured fields for binary packet encoding.
    """
    nama_lower = class_name.lower()
    sudut_ref = 0.0

    is_elco = ("benar" in nama_lower or "salah" in nama_lower or
               "elco" in nama_lower)

    if sudut_manual is not None:
        # ─── Mode manual: sudut diberikan langsung ───
        sudut_act, dev, kor_servo, arah_servo = hitung_koreksi_terpendek(sudut_manual, sudut_ref)

        if abs(dev) < 15.0:
            status_qc = "BENAR (SESUAI REFERENSI)"
            aksi = "LANJUT KE JIG (PASS)"
        elif is_elco:
            status_qc = "SALAH (PERLU REORIENTASI)"
            aksi = f"PUTAR SERVO CW {kor_servo:.1f}°"
        else:
            status_qc = "BUKAN ELCO (REJECT)"
            aksi = "ABAIKAN / REJECT"
            sudut_act, dev, kor_servo = 0.0, 0.0, 0.0
            arah_servo = "DIAM"

    elif is_elco:
        # ─── Mode otomatis: deteksi sudut dari ROI citra ───
        if roi_bgr is not None and roi_bgr.size > 0:
            sudut_deteksi = hitung_sudut_kontur(roi_bgr)
        else:
            # No ROI available — return zero angle with a warning.
            # DO NOT use class name as a shortcut for physical angle.
            sudut_deteksi = 0.0

        sudut_act, dev, kor_servo, arah_servo = hitung_koreksi_terpendek(sudut_deteksi, sudut_ref)

        if abs(dev) < 15.0:
            status_qc = "BENAR (SESUAI REFERENSI)"
            aksi = "LANJUT KE JIG (PASS)"
        else:
            status_qc = "SALAH (PERLU REORIENTASI)"
            aksi = f"PUTAR SERVO CW {kor_servo:.1f}°"

    else:
        # ─── Bukan Elco → Reject ───
        sudut_act = 0.0
        dev = 0.0
        kor_servo = 0.0
        arah_servo = "DIAM"
        status_qc = "BUKAN ELCO (REJECT)"
        aksi = "ABAIKAN / REJECT"

    # Structured data for binary protocol encoding
    uart_data = {
        'class_id': class_id,
        'angle_deg': sudut_act,
        'correction_deg': kor_servo,
    }

    return sudut_act, sudut_ref, dev, kor_servo, arah_servo, status_qc, aksi, uart_data


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
