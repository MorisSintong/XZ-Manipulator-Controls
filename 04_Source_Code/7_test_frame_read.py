"""
===============================================================================
  7_test_frame_read.py — PENGUJIAN PEMBACAAN FRAME VIDEO (MISREAD TEST)
  Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
  Husein Alhamid (4212301035)
===============================================================================

  Deskripsi:
  Script ini menguji apakah SETIAP FRAME dari file video dapat dibaca dengan
  sukses oleh OpenCV. Tujuannya memastikan tidak ada frame yang terlewat,
  corrupt, atau gagal dibaca (misread) selama pemrosesan video.

  Pengujian meliputi:
  1. Pembacaan sekuensial tiap frame (ret, frame = cap.read())
  2. Validasi frame tidak None dan dimensi valid
  3. Validasi pixel data (bukan frame hitam/corrupt)
  4. Pencatatan frame gagal beserta posisi timestamp-nya
  5. Retry otomatis untuk frame yang gagal (seek + re-read)
  6. Laporan detail dan ringkasan per-video

  Cara menjalankan:
    python 7_test_frame_read.py
    python 7_test_frame_read.py --video "path/ke/video.mp4"
    python 7_test_frame_read.py --video "path/ke/video.mp4" --strict
    python 7_test_frame_read.py --all

===============================================================================
"""

import os
import sys
import io
import time
import csv
import argparse
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

import cv2
import numpy as np

ROOT_DIR = Path(__file__).parent.parent.resolve()
DEFAULT_OUTPUT_DIR = ROOT_DIR / "05_Hasil_Pengujian" / "frame_read_test"
VIDEO_EXTENSIONS = {'.mp4', '.avi', '.mov', '.mkv', '.wmv', '.flv', '.webm'}

# ─── Kode warna terminal ───
class TC:
    OK    = "\033[92m"   # Hijau
    FAIL  = "\033[91m"   # Merah
    WARN  = "\033[93m"   # Kuning
    INFO  = "\033[96m"   # Cyan
    BOLD  = "\033[1m"
    RESET = "\033[0m"


def format_waktu(detik: float) -> str:
    """Format detik ke MM:SS.mmm"""
    m = int(detik // 60)
    s = detik % 60
    return f"{m:02d}:{s:06.3f}"


def validasi_frame(frame: np.ndarray, strict: bool = False) -> tuple:
    """
    Validasi apakah frame yang dibaca valid.

    Returns:
        (is_valid: bool, reason: str)
    """
    if frame is None:
        return False, "Frame is None"

    if frame.size == 0:
        return False, "Frame kosong (size=0)"

    if len(frame.shape) < 2:
        return False, f"Dimensi frame invalid: {frame.shape}"

    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        return False, f"Resolusi invalid: {w}x{h}"

    if strict:
        # Cek apakah frame pure hitam (kemungkinan corrupt)
        if np.mean(frame) < 1.0:
            return False, "Frame hitam total (kemungkinan corrupt)"

        # Cek apakah frame pure putih
        if np.mean(frame) > 254.0:
            return False, "Frame putih total (kemungkinan corrupt)"

        # Cek apakah frame memiliki variasi pixel
        std_val = np.std(frame.astype(np.float32))
        if std_val < 0.5:
            return False, f"Frame tanpa variasi pixel (std={std_val:.3f}, kemungkinan corrupt)"

    return True, "OK"


def retry_read_frame(cap: cv2.VideoCapture, frame_idx: int, max_retries: int = 3) -> tuple:
    """
    Mencoba membaca ulang frame yang gagal dengan seek ke posisi frame tersebut.

    Returns:
        (success: bool, frame: np.ndarray or None, attempt: int)
    """
    for attempt in range(1, max_retries + 1):
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if ret and frame is not None:
            return True, frame, attempt
        time.sleep(0.01)  # Jeda kecil sebelum retry

    return False, None, max_retries


def test_single_video(video_path: Path, output_dir: Path, strict: bool = False,
                      max_retries: int = 3, verbose: bool = True) -> dict:
    """
    Menguji pembacaan setiap frame dari satu file video.

    Returns:
        dict berisi hasil pengujian lengkap
    """
    result = {
        "video": video_path.name,
        "video_path": str(video_path),
        "total_frames_expected": 0,
        "total_frames_read": 0,
        "total_frames_success": 0,
        "total_frames_failed": 0,
        "total_frames_recovered": 0,
        "failed_frames": [],
        "recovered_frames": [],
        "success_rate": 0.0,
        "status": "UNKNOWN",
        "duration_seconds": 0.0,
        "video_fps": 0.0,
        "video_resolution": "",
        "video_duration": "",
        "per_frame_log": [],
    }

    # ─── Buka video ───
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        result["status"] = "GAGAL BUKA VIDEO"
        print(f"  {TC.FAIL}[GAGAL]{TC.RESET} Tidak dapat membuka: {video_path.name}")
        return result

    # Ambil properti video
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    durasi_video = total_frames / video_fps if video_fps > 0 else 0

    result["total_frames_expected"] = total_frames
    result["video_fps"] = video_fps
    result["video_resolution"] = f"{width}x{height}"
    result["video_duration"] = format_waktu(durasi_video)

    print(f"\n{'=' * 78}")
    print(f"  {TC.BOLD}PENGUJIAN PEMBACAAN FRAME — {video_path.name}{TC.RESET}")
    print(f"{'=' * 78}")
    print(f"  Resolusi       : {width} x {height}")
    print(f"  FPS            : {video_fps:.1f}")
    print(f"  Total Frame    : {total_frames}")
    print(f"  Durasi Video   : {format_waktu(durasi_video)}")
    print(f"  Mode Strict    : {'Ya' if strict else 'Tidak'}")
    print(f"  Max Retry      : {max_retries}x per frame")
    print(f"{'=' * 78}")

    frame_idx = 0
    frames_success = 0
    frames_failed = 0
    frames_recovered = 0
    failed_list = []
    recovered_list = []
    per_frame_log = []

    t_start = time.time()

    # Progress bar config
    progress_interval = max(1, total_frames // 50)  # Update progress setiap ~2%

    print(f"\n  Progres pembacaan frame:")
    print(f"  {'Frame':<10} | {'Status':<12} | {'Timestamp':<12} | {'Waktu Baca (ms)':<16} | {'Keterangan'}")
    print(f"  {'-' * 78}")

    while True:
        t_frame_start = time.perf_counter()
        ret, frame = cap.read()
        t_frame_ms = (time.perf_counter() - t_frame_start) * 1000.0

        # Cek apakah video sudah habis
        if not ret and frame is None:
            # Verifikasi apakah ini benar-benar akhir video atau frame gagal
            if frame_idx < total_frames:
                # Frame gagal dibaca, coba retry
                retry_success, retry_frame, attempts = retry_read_frame(cap, frame_idx, max_retries)

                if retry_success:
                    # Frame berhasil dibaca setelah retry
                    is_valid, reason = validasi_frame(retry_frame, strict)
                    if is_valid:
                        frames_recovered += 1
                        frames_success += 1
                        recovered_list.append({
                            "frame": frame_idx + 1,
                            "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                            "retry_attempts": attempts,
                        })
                        log_entry = {
                            "frame": frame_idx + 1,
                            "status": "RECOVERED",
                            "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                            "read_time_ms": round(t_frame_ms, 3),
                            "detail": f"Berhasil setelah {attempts}x retry"
                        }
                        per_frame_log.append(log_entry)

                        if verbose or frame_idx % progress_interval == 0:
                            print(f"  {frame_idx + 1:<10} | {TC.WARN}RECOVERED{TC.RESET}   | "
                                  f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                                  f"{t_frame_ms:<16.3f} | Retry {attempts}x berhasil")

                        # Set posisi ke frame berikutnya
                        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)
                        frame_idx += 1
                        continue
                    else:
                        # Retry sukses baca tapi frame invalid
                        frames_failed += 1
                        failed_list.append({
                            "frame": frame_idx + 1,
                            "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                            "reason": f"Retry {attempts}x: frame dibaca tapi {reason}",
                        })
                        log_entry = {
                            "frame": frame_idx + 1,
                            "status": "FAILED",
                            "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                            "read_time_ms": round(t_frame_ms, 3),
                            "detail": f"Retry {attempts}x: {reason}"
                        }
                        per_frame_log.append(log_entry)

                        print(f"  {frame_idx + 1:<10} | {TC.FAIL}FAILED{TC.RESET}      | "
                              f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                              f"{t_frame_ms:<16.3f} | {reason}")

                        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)
                        frame_idx += 1
                        continue
                else:
                    # Retry juga gagal
                    frames_failed += 1
                    failed_list.append({
                        "frame": frame_idx + 1,
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "reason": f"Gagal baca setelah {max_retries}x retry",
                    })
                    log_entry = {
                        "frame": frame_idx + 1,
                        "status": "FAILED",
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "read_time_ms": round(t_frame_ms, 3),
                        "detail": f"Gagal total setelah {max_retries}x retry"
                    }
                    per_frame_log.append(log_entry)

                    print(f"  {frame_idx + 1:<10} | {TC.FAIL}FAILED{TC.RESET}      | "
                          f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                          f"{t_frame_ms:<16.3f} | Gagal {max_retries}x retry")

                    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)
                    frame_idx += 1
                    continue
            else:
                # Sudah melewati total_frames, video selesai
                break

        # Frame berhasil dibaca, validasi
        is_valid, reason = validasi_frame(frame, strict)

        if is_valid:
            frames_success += 1
            log_entry = {
                "frame": frame_idx + 1,
                "status": "OK",
                "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                "read_time_ms": round(t_frame_ms, 3),
                "detail": "OK"
            }
            per_frame_log.append(log_entry)

            # Print progress: setiap frame (verbose) atau setiap interval
            if verbose and frame_idx % progress_interval == 0:
                print(f"  {frame_idx + 1:<10} | {TC.OK}OK{TC.RESET}          | "
                      f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                      f"{t_frame_ms:<16.3f} | Dimensi: {frame.shape[1]}x{frame.shape[0]}")
        else:
            # Frame dibaca (ret=True) tapi konten invalid
            # Coba retry
            retry_success, retry_frame, attempts = retry_read_frame(cap, frame_idx, max_retries)

            if retry_success:
                is_valid2, reason2 = validasi_frame(retry_frame, strict)
                if is_valid2:
                    frames_recovered += 1
                    frames_success += 1
                    recovered_list.append({
                        "frame": frame_idx + 1,
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "retry_attempts": attempts,
                    })
                    log_entry = {
                        "frame": frame_idx + 1,
                        "status": "RECOVERED",
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "read_time_ms": round(t_frame_ms, 3),
                        "detail": f"Awalnya: {reason}, lalu berhasil retry {attempts}x"
                    }
                    per_frame_log.append(log_entry)

                    print(f"  {frame_idx + 1:<10} | {TC.WARN}RECOVERED{TC.RESET}   | "
                          f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                          f"{t_frame_ms:<16.3f} | {reason} -> Retry {attempts}x OK")

                    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)
                else:
                    frames_failed += 1
                    failed_list.append({
                        "frame": frame_idx + 1,
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "reason": reason,
                    })
                    log_entry = {
                        "frame": frame_idx + 1,
                        "status": "FAILED",
                        "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                        "read_time_ms": round(t_frame_ms, 3),
                        "detail": reason
                    }
                    per_frame_log.append(log_entry)

                    print(f"  {frame_idx + 1:<10} | {TC.FAIL}FAILED{TC.RESET}      | "
                          f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                          f"{t_frame_ms:<16.3f} | {reason}")

                    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)
            else:
                frames_failed += 1
                failed_list.append({
                    "frame": frame_idx + 1,
                    "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                    "reason": reason,
                })
                log_entry = {
                    "frame": frame_idx + 1,
                    "status": "FAILED",
                    "timestamp": format_waktu((frame_idx) / video_fps if video_fps > 0 else 0),
                    "read_time_ms": round(t_frame_ms, 3),
                    "detail": reason
                }
                per_frame_log.append(log_entry)

                print(f"  {frame_idx + 1:<10} | {TC.FAIL}FAILED{TC.RESET}      | "
                      f"{format_waktu((frame_idx) / video_fps if video_fps > 0 else 0):<12} | "
                      f"{t_frame_ms:<16.3f} | {reason}")

                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx + 1)

        frame_idx += 1

        # Safety: jangan melebihi total_frames
        if frame_idx >= total_frames:
            break

    cap.release()
    t_total = time.time() - t_start

    # ─── Hitung hasil ───
    result["total_frames_read"] = frame_idx
    result["total_frames_success"] = frames_success
    result["total_frames_failed"] = frames_failed
    result["total_frames_recovered"] = frames_recovered
    result["failed_frames"] = failed_list
    result["recovered_frames"] = recovered_list
    result["duration_seconds"] = round(t_total, 3)
    result["per_frame_log"] = per_frame_log

    if total_frames > 0:
        result["success_rate"] = round((frames_success / total_frames) * 100.0, 2)
    else:
        result["success_rate"] = 0.0

    if frames_failed == 0 and frames_success == total_frames:
        result["status"] = "PASSED (100%)"
    elif frames_failed == 0 and frames_success > 0:
        result["status"] = f"PASSED ({result['success_rate']:.1f}%)"
    else:
        result["status"] = f"FAILED ({frames_failed} frame gagal)"

    # ─── Print ringkasan per-video ───
    print(f"\n  {'─' * 78}")
    print(f"  {TC.BOLD}RINGKASAN — {video_path.name}{TC.RESET}")
    print(f"  {'─' * 78}")
    print(f"  Total Frame Expected  : {total_frames}")
    print(f"  Total Frame Dibaca    : {frame_idx}")
    print(f"  Frame Sukses          : {TC.OK}{frames_success}{TC.RESET}")
    print(f"  Frame Gagal           : {TC.FAIL if frames_failed > 0 else TC.OK}{frames_failed}{TC.RESET}")
    print(f"  Frame Recovered       : {TC.WARN}{frames_recovered}{TC.RESET}")
    print(f"  Success Rate          : {TC.OK if result['success_rate'] == 100.0 else TC.FAIL}"
          f"{result['success_rate']:.2f}%{TC.RESET}")
    print(f"  Durasi Pengujian      : {t_total:.3f} detik")
    print(f"  Kecepatan Baca        : {frame_idx / t_total:.1f} frame/detik" if t_total > 0 else "")

    if frames_failed > 0:
        print(f"\n  {TC.FAIL}DETAIL FRAME GAGAL:{TC.RESET}")
        for f_info in failed_list:
            print(f"    ✗ Frame #{f_info['frame']:>5} (t={f_info['timestamp']}) — {f_info['reason']}")

    if frames_recovered > 0:
        print(f"\n  {TC.WARN}DETAIL FRAME RECOVERED:{TC.RESET}")
        for r_info in recovered_list:
            print(f"    ↻ Frame #{r_info['frame']:>5} (t={r_info['timestamp']}) — Retry {r_info['retry_attempts']}x")

    status_icon = f"{TC.OK}✓ PASSED{TC.RESET}" if frames_failed == 0 else f"{TC.FAIL}✗ FAILED{TC.RESET}"
    print(f"\n  Status Akhir: {status_icon} — {result['status']}")
    print(f"  {'=' * 78}\n")

    return result


def simpan_laporan_csv(results: list, output_dir: Path):
    """Simpan laporan detail ke CSV."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # ─── CSV ringkasan semua video ───
    summary_csv = output_dir / "ringkasan_test_frame_read.csv"
    with open(summary_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            "No", "File_Video", "Resolusi", "FPS", "Durasi_Video",
            "Total_Frame", "Frame_Dibaca", "Frame_Sukses", "Frame_Gagal",
            "Frame_Recovered", "Success_Rate_%", "Status",
            "Durasi_Test_detik"
        ])
        for i, r in enumerate(results, 1):
            writer.writerow([
                i, r["video"], r["video_resolution"], r["video_fps"],
                r["video_duration"], r["total_frames_expected"],
                r["total_frames_read"], r["total_frames_success"],
                r["total_frames_failed"], r["total_frames_recovered"],
                r["success_rate"], r["status"], r["duration_seconds"]
            ])
    print(f"[✓] Ringkasan tersimpan: {summary_csv}")

    # ─── CSV detail per-frame per-video ───
    for r in results:
        if r["per_frame_log"]:
            stem = Path(r["video"]).stem
            detail_csv = output_dir / f"detail_frame_{stem}.csv"
            with open(detail_csv, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Frame", "Status", "Timestamp", "Waktu_Baca_ms", "Keterangan"])
                for entry in r["per_frame_log"]:
                    writer.writerow([
                        entry["frame"], entry["status"],
                        entry["timestamp"], entry["read_time_ms"],
                        entry["detail"]
                    ])
            print(f"[✓] Detail per-frame tersimpan: {detail_csv}")


def simpan_laporan_txt(results: list, output_dir: Path):
    """Simpan laporan ringkasan dalam format teks."""
    output_dir.mkdir(parents=True, exist_ok=True)
    report_file = output_dir / "laporan_test_frame_read.txt"

    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=" * 78 + "\n")
        f.write("  LAPORAN PENGUJIAN PEMBACAAN FRAME VIDEO (MISREAD TEST)\n")
        f.write("  Tugas Akhir: Quality Control Kapasitor — Husein Alhamid\n")
        f.write(f"  Tanggal: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 78 + "\n\n")

        all_passed = True
        total_videos = len(results)
        total_frames_all = 0
        total_success_all = 0
        total_failed_all = 0

        for i, r in enumerate(results, 1):
            total_frames_all += r["total_frames_expected"]
            total_success_all += r["total_frames_success"]
            total_failed_all += r["total_frames_failed"]
            if r["total_frames_failed"] > 0:
                all_passed = False

            f.write(f"─── Video #{i}: {r['video']} ───\n")
            f.write(f"  Path             : {r['video_path']}\n")
            f.write(f"  Resolusi         : {r['video_resolution']}\n")
            f.write(f"  FPS              : {r['video_fps']}\n")
            f.write(f"  Durasi Video     : {r['video_duration']}\n")
            f.write(f"  Total Frame      : {r['total_frames_expected']}\n")
            f.write(f"  Frame Dibaca     : {r['total_frames_read']}\n")
            f.write(f"  Frame Sukses     : {r['total_frames_success']}\n")
            f.write(f"  Frame Gagal      : {r['total_frames_failed']}\n")
            f.write(f"  Frame Recovered  : {r['total_frames_recovered']}\n")
            f.write(f"  Success Rate     : {r['success_rate']:.2f}%\n")
            f.write(f"  Status           : {r['status']}\n")
            f.write(f"  Durasi Test      : {r['duration_seconds']:.3f} detik\n")

            if r["failed_frames"]:
                f.write(f"\n  Frame Gagal:\n")
                for ff in r["failed_frames"]:
                    f.write(f"    ✗ Frame #{ff['frame']} (t={ff['timestamp']}) — {ff['reason']}\n")

            if r["recovered_frames"]:
                f.write(f"\n  Frame Recovered:\n")
                for rf in r["recovered_frames"]:
                    f.write(f"    ↻ Frame #{rf['frame']} (t={rf['timestamp']}) — Retry {rf['retry_attempts']}x\n")

            f.write("\n")

        f.write("=" * 78 + "\n")
        f.write("  RINGKASAN TOTAL\n")
        f.write("=" * 78 + "\n")
        f.write(f"  Total Video Diuji     : {total_videos}\n")
        f.write(f"  Total Frame Keseluruhan: {total_frames_all}\n")
        f.write(f"  Total Sukses          : {total_success_all}\n")
        f.write(f"  Total Gagal           : {total_failed_all}\n")
        rate = (total_success_all / total_frames_all * 100.0) if total_frames_all > 0 else 0.0
        f.write(f"  Success Rate Global   : {rate:.2f}%\n")
        f.write(f"  Kesimpulan            : {'SEMUA VIDEO PASSED ✓' if all_passed else 'ADA VIDEO YANG FAILED ✗'}\n")
        f.write("=" * 78 + "\n")

    print(f"[✓] Laporan lengkap tersimpan: {report_file}")


def cari_semua_video(root: Path) -> list:
    """Cari semua file video di root directory."""
    videos = []
    for ext in VIDEO_EXTENSIONS:
        videos.extend(root.glob(f"*{ext}"))
        videos.extend(root.glob(f"*{ext.upper()}"))
    return sorted(set(videos))


def main():
    parser = argparse.ArgumentParser(
        description="Pengujian Pembacaan Frame Video (Misread Test)"
    )
    parser.add_argument('--video', type=str, default=None,
                        help='Path ke file video spesifik untuk diuji')
    parser.add_argument('--all', action='store_true',
                        help='Uji semua file video di folder root proyek')
    parser.add_argument('--strict', action='store_true',
                        help='Mode strict: cek juga frame hitam/putih/tanpa variasi')
    parser.add_argument('--max-retries', type=int, default=3,
                        help='Maksimal retry per frame yang gagal (default: 3)')
    parser.add_argument('--output', type=str, default=str(DEFAULT_OUTPUT_DIR),
                        help='Folder output laporan pengujian')
    parser.add_argument('--verbose', action='store_true', default=True,
                        help='Tampilkan progress tiap frame')
    parser.add_argument('--quiet', action='store_true',
                        help='Hanya tampilkan frame yang gagal/recovered')

    args = parser.parse_args()

    if args.quiet:
        args.verbose = False

    output_dir = Path(args.output)

    # ─── Header ───
    print("\n" + "█" * 78)
    print("█" + " " * 76 + "█")
    print("█   PENGUJIAN PEMBACAAN FRAME VIDEO (MISREAD TEST)                         █")
    print("█   Tugas Akhir: QC Kapasitor — Husein Alhamid (4212301035)                █")
    print("█" + " " * 76 + "█")
    print("█" * 78)

    # ─── Kumpulkan daftar video ───
    video_list = []

    if args.video:
        vp = Path(args.video)
        if not vp.exists():
            print(f"\n{TC.FAIL}[ERROR]{TC.RESET} File video tidak ditemukan: {vp}")
            sys.exit(1)
        if vp.suffix.lower() not in VIDEO_EXTENSIONS:
            print(f"\n{TC.FAIL}[ERROR]{TC.RESET} Format file tidak didukung: {vp.suffix}")
            sys.exit(1)
        video_list.append(vp)
    elif args.all:
        video_list = cari_semua_video(ROOT_DIR)
    else:
        # Default: cari video di root
        video_list = cari_semua_video(ROOT_DIR)

    if not video_list:
        print(f"\n{TC.WARN}[!]{TC.RESET} Tidak ditemukan file video di: {ROOT_DIR}")
        print(f"    Format yang didukung: {', '.join(sorted(VIDEO_EXTENSIONS))}")
        sys.exit(1)

    print(f"\n  Video yang akan diuji ({len(video_list)} file):")
    for i, vp in enumerate(video_list, 1):
        size_mb = vp.stat().st_size / (1024 * 1024)
        print(f"    {i}. {vp.name} ({size_mb:.1f} MB)")

    # ─── Jalankan pengujian ───
    all_results = []
    for vp in video_list:
        result = test_single_video(vp, output_dir, strict=args.strict,
                                   max_retries=args.max_retries,
                                   verbose=args.verbose)
        all_results.append(result)

    # ─── Simpan laporan ───
    simpan_laporan_csv(all_results, output_dir)
    simpan_laporan_txt(all_results, output_dir)

    # ─── Ringkasan akhir ───
    total_v = len(all_results)
    total_f = sum(r["total_frames_expected"] for r in all_results)
    total_s = sum(r["total_frames_success"] for r in all_results)
    total_fail = sum(r["total_frames_failed"] for r in all_results)
    total_rec = sum(r["total_frames_recovered"] for r in all_results)
    rate = (total_s / total_f * 100.0) if total_f > 0 else 0.0

    print("\n" + "█" * 78)
    print(f"  {TC.BOLD}RINGKASAN AKHIR PENGUJIAN FRAME READ{TC.RESET}")
    print("█" * 78)
    print(f"  Total Video       : {total_v}")
    print(f"  Total Frame       : {total_f}")
    print(f"  Frame Sukses      : {TC.OK}{total_s}{TC.RESET}")
    print(f"  Frame Gagal       : {TC.FAIL if total_fail > 0 else TC.OK}{total_fail}{TC.RESET}")
    print(f"  Frame Recovered   : {TC.WARN}{total_rec}{TC.RESET}")
    print(f"  Success Rate      : {TC.OK if rate == 100.0 else TC.FAIL}{rate:.2f}%{TC.RESET}")

    if total_fail == 0:
        print(f"\n  {TC.OK}{'═' * 60}")
        print(f"  ✓ SEMUA FRAME BERHASIL DIBACA — TIDAK ADA MISREAD!")
        print(f"  {'═' * 60}{TC.RESET}")
    else:
        print(f"\n  {TC.FAIL}{'═' * 60}")
        print(f"  ✗ TERDAPAT {total_fail} FRAME GAGAL DIBACA (MISREAD)")
        print(f"  {'═' * 60}{TC.RESET}")

    print(f"\n  Output laporan: {output_dir}")
    print("█" * 78 + "\n")

    # Return exit code
    sys.exit(0 if total_fail == 0 else 1)


if __name__ == "__main__":
    main()
