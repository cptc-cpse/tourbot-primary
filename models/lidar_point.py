import json


class LidarPoint:
    def __init__(self, angle_deg, distance_mm, intensity):
        self.angle_deg = angle_deg
        self.distance_mm = distance_mm
        self.intensity = intensity

    def to_dict(self) -> dict[str, float | int]:
        return {
            "angle_deg": self.angle_deg,
            "distance_mm": self.distance_mm,
            "intensity": self.intensity
        }

    def to_json(self) -> str:
        """Serialize this point as a JSON object."""
        return json.dumps(self.to_dict())