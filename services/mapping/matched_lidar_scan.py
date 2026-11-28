from dataclasses import dataclass

from services.mapping.pose_sample import PoseSample


@dataclass(frozen=True)
class MatchedLidarScan:
    timestamp: float
    points: object
    position: PoseSample
