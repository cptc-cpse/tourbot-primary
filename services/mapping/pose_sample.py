from dataclasses import dataclass


@dataclass(frozen=True)
class PoseSample:
    timestamp: float
    x: float
    y: float
    heading_degrees: float
