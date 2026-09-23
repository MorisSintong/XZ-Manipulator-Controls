"""
Modul Conveyor Tracker — Spatial Object Tracking & Debounce
Tugas Akhir: Quality Control Kapasitor Berdasarkan Polaritas
Husein Alhamid (4212301035)

Lightweight centroid-based tracker that:
  1. Matches new detections to existing tracked objects via IoU + centroid distance.
  2. Evaluates each object exactly ONCE when it crosses the inspection line.
  3. Queues pick commands into a FIFO, preventing UART flooding.
"""

import math
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple


@dataclass
class Detection:
    """A single detection from YOLO in the current frame."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]   # (x1, y1, x2, y2)
    angle_deg: float = 0.0
    correction_deg: float = 0.0
    status_qc: str = ""
    aksi: str = ""


@dataclass
class TrackedObject:
    """A tracked object across multiple frames."""
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    centroid: Tuple[float, float]
    angle_deg: float = 0.0
    correction_deg: float = 0.0
    status_qc: str = ""
    aksi: str = ""

    frames_seen: int = 1
    frames_disappeared: int = 0
    dispatched: bool = False          # True if UART pick command already sent
    crossed_inspection: bool = False  # True if object crossed inspection line

    # Best measurement (highest confidence reading)
    best_confidence: float = 0.0
    best_angle_deg: float = 0.0
    best_correction_deg: float = 0.0
    best_status_qc: str = ""
    best_aksi: str = ""
    best_class_id: int = 0
    best_class_name: str = ""


def _centroid(bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
    """Compute centroid of bounding box (x1, y1, x2, y2)."""
    return ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)


def _iou(box_a: Tuple[int, int, int, int], box_b: Tuple[int, int, int, int]) -> float:
    """Compute Intersection over Union between two bounding boxes."""
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)
    if inter == 0:
        return 0.0

    area_a = max(1, (box_a[2] - box_a[0]) * (box_a[3] - box_a[1]))
    area_b = max(1, (box_b[2] - box_b[0]) * (box_b[3] - box_b[1]))

    return inter / (area_a + area_b - inter)


def _euclidean(pt_a: Tuple[float, float], pt_b: Tuple[float, float]) -> float:
    """Euclidean distance between two points."""
    return math.sqrt((pt_a[0] - pt_b[0]) ** 2 + (pt_a[1] - pt_b[1]) ** 2)


class ConveyorTracker:
    """
    Lightweight centroid + IoU tracker for capacitors on a moving conveyor.

    Each detected capacitor is tracked across frames. When it crosses the
    configurable inspection line (Y threshold), its best measurement is
    evaluated and queued EXACTLY ONCE for UART dispatch.

    This prevents serial flooding: even if a capacitor is visible for 30+
    frames, only 1 pick command is ever generated.

    Parameters:
        max_disappeared: Frames before a track is removed (default 10).
        distance_threshold: Max centroid distance (px) for matching (default 80).
        iou_threshold: Min IoU for matching (default 0.2).
        inspection_line_y: Y pixel coordinate of the inspection line.
            Objects crossing this line trigger evaluation.
            Default 240 (center of 480p frame).
        min_frames_before_eval: Minimum frames an object must be tracked
            before it can be evaluated (default 3). Filters transient
            false positives.
    """

    def __init__(
        self,
        max_disappeared: int = 10,
        distance_threshold: float = 80.0,
        iou_threshold: float = 0.2,
        inspection_line_y: int = 240,
        min_frames_before_eval: int = 3,
    ):
        self.max_disappeared = max_disappeared
        self.distance_threshold = distance_threshold
        self.iou_threshold = iou_threshold
        self.inspection_line_y = inspection_line_y
        self.min_frames_before_eval = min_frames_before_eval

        self._next_id = 1
        self._tracks: Dict[int, TrackedObject] = {}
        self._pending_picks: List[TrackedObject] = []

    def update(self, detections: List[Detection]) -> List[TrackedObject]:
        """
        Update tracker with new frame detections.

        Args:
            detections: List of Detection objects from current frame.

        Returns:
            List of all currently active TrackedObject instances.
        """
        if not detections:
            # No detections: increment disappeared counter for all tracks
            disappeared_ids = []
            for tid, track in self._tracks.items():
                track.frames_disappeared += 1
                if track.frames_disappeared > self.max_disappeared:
                    disappeared_ids.append(tid)
            for tid in disappeared_ids:
                del self._tracks[tid]
            return list(self._tracks.values())

        # Compute centroids for new detections
        det_centroids = [_centroid(d.bbox) for d in detections]

        if not self._tracks:
            # No existing tracks: register all detections as new tracks
            for det, cent in zip(detections, det_centroids):
                self._register(det, cent)
            return list(self._tracks.values())

        # Match detections to existing tracks using combined IoU + distance
        track_ids = list(self._tracks.keys())
        tracks_list = [self._tracks[tid] for tid in track_ids]

        # Build cost matrix: rows = existing tracks, cols = new detections
        n_tracks = len(tracks_list)
        n_dets = len(detections)

        matched_tracks = set()
        matched_dets = set()

        # Greedy matching by best combined score
        match_candidates = []
        for t_idx, track in enumerate(tracks_list):
            for d_idx, det in enumerate(detections):
                iou_val = _iou(track.bbox, det.bbox)
                dist = _euclidean(track.centroid, det_centroids[d_idx])

                # Combined score: high IoU and low distance is better
                if iou_val >= self.iou_threshold or dist <= self.distance_threshold:
                    score = iou_val * 1000 - dist  # higher is better
                    match_candidates.append((score, t_idx, d_idx))

        # Sort by score descending and greedily assign
        match_candidates.sort(key=lambda x: x[0], reverse=True)
        for score, t_idx, d_idx in match_candidates:
            if t_idx in matched_tracks or d_idx in matched_dets:
                continue
            matched_tracks.add(t_idx)
            matched_dets.add(d_idx)
            self._update_track(track_ids[t_idx], detections[d_idx], det_centroids[d_idx])

        # Increment disappeared for unmatched tracks
        disappeared_ids = []
        for t_idx in range(n_tracks):
            if t_idx not in matched_tracks:
                tid = track_ids[t_idx]
                self._tracks[tid].frames_disappeared += 1
                if self._tracks[tid].frames_disappeared > self.max_disappeared:
                    disappeared_ids.append(tid)
        for tid in disappeared_ids:
            del self._tracks[tid]

        # Register new tracks for unmatched detections
        for d_idx in range(n_dets):
            if d_idx not in matched_dets:
                self._register(detections[d_idx], det_centroids[d_idx])

        # Check inspection line crossings
        self._check_inspection_line()

        return list(self._tracks.values())

    def get_pending_picks(self) -> List[TrackedObject]:
        """
        Return objects that have crossed the inspection line and are
        ready for UART dispatch. Each object is returned exactly once.
        """
        picks = list(self._pending_picks)
        self._pending_picks.clear()
        return picks

    def reset(self):
        """Clear all tracks and pending picks."""
        self._tracks.clear()
        self._pending_picks.clear()
        self._next_id = 1

    @property
    def active_tracks(self) -> Dict[int, TrackedObject]:
        """Read-only access to current tracks."""
        return dict(self._tracks)

    def _register(self, det: Detection, centroid: Tuple[float, float]):
        """Register a new tracked object."""
        tid = self._next_id
        self._next_id += 1

        track = TrackedObject(
            track_id=tid,
            class_id=det.class_id,
            class_name=det.class_name,
            confidence=det.confidence,
            bbox=det.bbox,
            centroid=centroid,
            angle_deg=det.angle_deg,
            correction_deg=det.correction_deg,
            status_qc=det.status_qc,
            aksi=det.aksi,
            frames_seen=1,
            frames_disappeared=0,
            dispatched=False,
            crossed_inspection=False,
            best_confidence=det.confidence,
            best_angle_deg=det.angle_deg,
            best_correction_deg=det.correction_deg,
            best_status_qc=det.status_qc,
            best_aksi=det.aksi,
            best_class_id=det.class_id,
            best_class_name=det.class_name,
        )
        self._tracks[tid] = track

    def _update_track(self, tid: int, det: Detection, centroid: Tuple[float, float]):
        """Update an existing track with a new detection."""
        track = self._tracks[tid]
        track.bbox = det.bbox
        track.centroid = centroid
        track.class_id = det.class_id
        track.class_name = det.class_name
        track.confidence = det.confidence
        track.angle_deg = det.angle_deg
        track.correction_deg = det.correction_deg
        track.status_qc = det.status_qc
        track.aksi = det.aksi
        track.frames_seen += 1
        track.frames_disappeared = 0

        # Keep the best measurement (highest confidence)
        if det.confidence > track.best_confidence:
            track.best_confidence = det.confidence
            track.best_angle_deg = det.angle_deg
            track.best_correction_deg = det.correction_deg
            track.best_status_qc = det.status_qc
            track.best_aksi = det.aksi
            track.best_class_id = det.class_id
            track.best_class_name = det.class_name

    def _check_inspection_line(self):
        """
        Check if any tracked object has crossed the inspection line.
        If so, and it meets the minimum frames threshold, queue it for dispatch.
        """
        for track in self._tracks.values():
            if track.dispatched or track.crossed_inspection:
                continue

            # Check if centroid Y has crossed the inspection line
            cy = track.centroid[1]
            if cy >= self.inspection_line_y and track.frames_seen >= self.min_frames_before_eval:
                track.crossed_inspection = True
                track.dispatched = True
                self._pending_picks.append(track)
