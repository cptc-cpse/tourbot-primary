import json
import time


class LidarPoint:
    def __init__(
        self,
        angle_deg: float,
        distance_mm: int,
        intensity: int,
        timestamp: float | None = None,
    ):
        self.angle_deg = angle_deg
        self.distance_mm = distance_mm
        self.intensity = intensity
        self.timestamp = time.time() if timestamp is None else timestamp

    def to_dict(self) -> dict[str, float | int]:
        return {
            "angle_deg": self.angle_deg,
            "distance_mm": self.distance_mm,
            "intensity": self.intensity,
            "timestamp": self.timestamp,
        }

    def to_json(self) -> str:
        """Serialize this point as a JSON object."""
        return json.dumps(self.to_dict())