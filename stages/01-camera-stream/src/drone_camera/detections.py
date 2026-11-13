"""Detection/tracking data handed to the GUI now and to later 3D stages.

Every timestamp is a LOCAL RECEPTION time (see ``capture.Frame``), never the
camera exposure time. Track IDs are valid only inside one camera: camera-01 ID 1
and camera-02 ID 1 are NOT the same physical drone. Cross-camera matching is a
later stage.
"""

from dataclasses import asdict, dataclass, field

from .geometry import bounding_circle

# New: first processed frame of a track. Tracked: seen in the previous processed
# frame. Reacquired: the same ID reappeared after unprocessed/undetected frames.
STATE_NEW = "new"
STATE_TRACKED = "tracked"
STATE_REACQUIRED = "reacquired"


@dataclass(frozen=True)
class RawTrack:
    """One tracked detection as produced by the inference engine (pixels)."""
    track_id: int | None
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float


@dataclass(frozen=True)
class DroneObservation:
    camera_id: str
    frame_sequence: int
    received_at: float          # Wall-clock reception time of the analysed frame.
    received_monotonic: float
    track_id: int | None        # Per-camera ID; None if the tracker gave no ID.
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2 (tracker-filtered estimate).
    center: tuple[float, float]
    base_radius: float
    safe_radius: float
    safety_factor: float
    confidence: float           # Detector confidence of the matched detection.
    state: str                  # new / tracked / reacquired

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FrameDetections:
    camera_id: str
    frame_sequence: int
    received_at: float
    received_monotonic: float
    completed_monotonic: float  # When inference finished (monotonic clock).
    inference_ms: float         # Model call duration only.
    image_size: tuple[int, int]  # width, height of the analysed frame.
    observations: tuple[DroneObservation, ...] = field(default_factory=tuple)

    @property
    def latency_ms(self) -> float:
        """Frame reception to result, including queue wait: not exposure latency."""
        return (self.completed_monotonic - self.received_monotonic) * 1000

    def to_dict(self) -> dict:
        data = asdict(self)
        data["latency_ms"] = self.latency_ms
        return data


class TrackBook:
    """Per-camera ID lifecycle so a reappearing ID is reported as such."""

    def __init__(self):
        self._last_seen: dict[int, int] = {}  # track id -> processed-frame index
        self._index = 0

    def begin_frame(self) -> int:
        self._index += 1
        return self._index

    def classify(self, track_id: int | None, frame_index: int) -> str:
        if track_id is None:
            return STATE_NEW
        previous = self._last_seen.get(track_id)
        self._last_seen[track_id] = frame_index
        if previous is None:
            return STATE_NEW
        return STATE_TRACKED if previous == frame_index - 1 else STATE_REACQUIRED


def build_observations(camera_id: str, frame_sequence: int, received_at: float,
                       received_monotonic: float, tracks, safety_factor: float,
                       book: TrackBook) -> tuple[DroneObservation, ...]:
    index = book.begin_frame()
    result = []
    for track in tracks:
        if not (track.x2 > track.x1 and track.y2 > track.y1):
            continue  # Degenerate box: ignore instead of inventing a circle.
        circle = bounding_circle(track.x1, track.y1, track.x2, track.y2, safety_factor)
        result.append(DroneObservation(
            camera_id, frame_sequence, received_at, received_monotonic, track.track_id,
            (track.x1, track.y1, track.x2, track.y2), (circle.cx, circle.cy),
            circle.base_radius, circle.safe_radius, circle.safety_factor,
            float(track.confidence), book.classify(track.track_id, index)))
    return tuple(result)
