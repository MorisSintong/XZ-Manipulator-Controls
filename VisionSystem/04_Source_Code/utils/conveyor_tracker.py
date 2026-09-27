import math
from typing import List, Dict, Any

class TrackInfo:
    def __init__(self, centroid: tuple, detection: Dict[str, Any]):
        self.centroid = centroid
        self.detection = detection
        self.disappeared = 0
        self.triggered = False

class ConveyorTracker:
    def __init__(self, max_disappeared: int = 15, max_distance: float = 80.0, inspection_line_y: float = 0.5):
        """
        Args:
            max_disappeared: Frames before dropping a lost track
            max_distance: Maximum centroid distance (pixels) to match existing track
            inspection_line_y: Fraction of frame height (0.0=top, 1.0=bottom) for the inspection trigger line
        """
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance
        self.inspection_line_y = inspection_line_y
        
        self.next_track_id = 1
        self.tracks: Dict[int, TrackInfo] = {}

    def _get_centroid(self, bbox) -> tuple:
        x1, y1, x2, y2 = bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    def _calculate_distance(self, c1: tuple, c2: tuple) -> float:
        return math.hypot(c1[0] - c2[0], c1[1] - c2[1])

    def update(self, detections: List[Dict[str, Any]], frame_height: int) -> List[Dict[str, Any]]:
        """
        Process new frame's detections and return list of objects that just crossed 
        the inspection line for the FIRST time.
        
        Args:
            detections: list of dicts with keys: 'bbox' (x1,y1,x2,y2), 'class_id', 'class_name', 'conf', 'roi', 'angle', 'correction'
            frame_height: height of the camera frame in pixels
        
        Returns:
            list of dicts for objects that should trigger UART send (crossed inspection line for first time)
            Each dict has: 'track_id', 'bbox', 'class_id', 'class_name', 'conf', 'roi', 'angle', 'correction', 'centroid'
        """
        trigger_list = []
        inspection_y_abs = self.inspection_line_y * frame_height

        if len(self.tracks) == 0:
            for det in detections:
                centroid = self._get_centroid(det['bbox'])
                self.tracks[self.next_track_id] = TrackInfo(centroid, det)
                self.next_track_id += 1
        else:
            if len(detections) > 0:
                detection_centroids = [self._get_centroid(det['bbox']) for det in detections]
                
                track_ids = list(self.tracks.keys())
                track_centroids = [self.tracks[tid].centroid for tid in track_ids]

                # Compute distance matrix (tracks vs detections)
                distances = []
                for i, t_cent in enumerate(track_centroids):
                    row = []
                    for j, d_cent in enumerate(detection_centroids):
                        row.append(self._calculate_distance(t_cent, d_cent))
                    distances.append(row)

                # Greedy matching
                used_tracks = set()
                used_detections = set()

                # Flatten and sort distances
                flat_distances = []
                for i in range(len(track_ids)):
                    for j in range(len(detection_centroids)):
                        flat_distances.append((distances[i][j], i, j))
                
                flat_distances.sort(key=lambda x: x[0])

                for dist, i, j in flat_distances:
                    if dist > self.max_distance:
                        continue
                    if i in used_tracks or j in used_detections:
                        continue
                    
                    # Match found
                    t_id = track_ids[i]
                    self.tracks[t_id].centroid = detection_centroids[j]
                    self.tracks[t_id].detection = detections[j]
                    self.tracks[t_id].disappeared = 0
                    
                    used_tracks.add(i)
                    used_detections.add(j)

                # Add new tracks for unmatched detections
                for j in range(len(detections)):
                    if j not in used_detections:
                        centroid = detection_centroids[j]
                        self.tracks[self.next_track_id] = TrackInfo(centroid, detections[j])
                        self.next_track_id += 1
                
                # Increment disappeared for unmatched tracks
                for i in range(len(track_ids)):
                    if i not in used_tracks:
                        t_id = track_ids[i]
                        self.tracks[t_id].disappeared += 1
            else:
                for t_id in list(self.tracks.keys()):
                    self.tracks[t_id].disappeared += 1

        # Remove disappeared tracks and check for triggers
        for t_id in list(self.tracks.keys()):
            track = self.tracks[t_id]
            if track.disappeared > self.max_disappeared:
                del self.tracks[t_id]
            else:
                centroid_y = track.centroid[1]
                if not track.triggered and centroid_y >= inspection_y_abs:
                    track.triggered = True
                    result_dict = track.detection.copy()
                    result_dict['track_id'] = t_id
                    result_dict['centroid'] = track.centroid
                    trigger_list.append(result_dict)

        return trigger_list

    def reset(self):
        """Clear all tracks."""
        self.next_track_id = 1
        self.tracks.clear()
