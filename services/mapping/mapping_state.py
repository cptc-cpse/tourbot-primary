import logging
import math
from collections import deque
from threading import RLock
from typing import Any

from communication.message import Message
from configuration import get_config_value
from services.mapping.matched_lidar_scan import MatchedLidarScan
from services.mapping.pose_sample import PoseSample

POSE_HISTORY_LIMIT = 1000
PENDING_LIDAR_LIMIT = 32
MATCHED_LIDAR_LIMIT = 100

TOURBOT_BASE_WIDTH_M = 0.5
TOURBOT_BASE_LENGTH_M = 0.6

logger = logging.getLogger(__name__)


class MappingState:
    """Own and update the occupancy map, robot pose history, and matched scans."""

    def __init__(self, map_resolution_m: float | None = None) -> None:
        self.map_resolution_m = (
            get_config_value("map_resolution_m")
            if map_resolution_m is None
            else map_resolution_m
        )
        self._base_width_cells = round(TOURBOT_BASE_WIDTH_M / self.map_resolution_m)
        self._base_length_cells = round(TOURBOT_BASE_LENGTH_M / self.map_resolution_m)
        self._occupancy_map: list[list[int]] | None = None
        self._current_position: dict[str, int | float] | None = None
        self._position_history: deque[PoseSample] = deque(maxlen=POSE_HISTORY_LIMIT)
        self._pending_lidar_scans: deque[Message] = deque()
        self._matched_lidar_scans: deque[MatchedLidarScan] = deque()
        self._lock = RLock()

    def receive_lidar(self, message: Message) -> None:
        with self._lock:
            if len(self._pending_lidar_scans) >= PENDING_LIDAR_LIMIT:
                logger.error(
                    "Dropping LiDAR scan at %.6f: waiting for pose history exceeded %d scans",
                    message.timestamp,
                    PENDING_LIDAR_LIMIT,
                )
                return

            self._pending_lidar_scans.append(message)
            self._match_pending_lidar_scans()

    def receive_oak(self, message: Message) -> None:
        payload = message.payload
        try:
            oak_map = payload["map"]
            map_origin = payload["map_origin"]
            map_width = payload["map_width"]
            map_height = payload["map_height"]
            position = payload["position"]
            orientation = payload["orientation"]
        except (KeyError, TypeError):
            logger.warning("Received OAK message without required mapping data")
            return

        if (
            position["x"] == 0
            and position["y"] == 0
            and position["z"] == 0
            and orientation["x"] == 0.5
            and orientation["y"] == 0.5
            and orientation["z"] == 0.5
            and orientation["w"] == 0.5
        ):
            logger.warning("Default positional values detected; OAK odometry likely stalled")
            return

        grid_x = (position["x"] - map_origin["x"]) / self.map_resolution_m
        grid_y = (position["y"] - map_origin["y"]) / self.map_resolution_m
        heading_degrees = self.quaternion_to_map_heading_deg(orientation)
        x_index = int(grid_x)
        y_index = int(grid_y)
        footprint_cells = self.tourbot_footprint_cells(
            grid_x,
            grid_y,
            heading_degrees,
            map_width,
            map_height,
        )

        with self._lock:
            self._current_position = {
                "x": x_index,
                "y": y_index,
                "orientation": heading_degrees,
            }
            self._record_pose(
                PoseSample(
                    timestamp=message.timestamp,
                    x=grid_x,
                    y=grid_y,
                    heading_degrees=heading_degrees,
                )
            )

            for y in range(map_height):
                for x in range(map_width):
                    if (x, y) in footprint_cells:
                        oak_map[y][x] = 1
                    elif oak_map[y][x] >= 10:
                        oak_map[y][x] = 2

            self._occupancy_map = oak_map

    def interpolate_position_at(self, timestamp: float) -> PoseSample | None:
        with self._lock:
            if not self._position_history:
                return None

            samples = tuple(self._position_history)
            if timestamp < samples[0].timestamp or timestamp > samples[-1].timestamp:
                return None

            for before, after in zip(samples, samples[1:]):
                if before.timestamp <= timestamp <= after.timestamp:
                    if timestamp == before.timestamp:
                        return before
                    if timestamp == after.timestamp:
                        return after

                    fraction = (timestamp - before.timestamp) / (
                        after.timestamp - before.timestamp
                    )
                    heading_delta = (
                        (after.heading_degrees - before.heading_degrees + 180.0)
                        % 360.0
                    ) - 180.0
                    return PoseSample(
                        timestamp=timestamp,
                        x=before.x + (after.x - before.x) * fraction,
                        y=before.y + (after.y - before.y) * fraction,
                        heading_degrees=(
                            before.heading_degrees + heading_delta * fraction
                        )
                        % 360.0,
                    )

            if timestamp == samples[-1].timestamp:
                return samples[-1]
            return None

    def take_matched_lidar_scan(self) -> MatchedLidarScan | None:
        with self._lock:
            if not self._matched_lidar_scans:
                return None
            return self._matched_lidar_scans.popleft()

    def get_map_snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            if self._occupancy_map is None:
                return None
            return {
                "map": [row.copy() for row in self._occupancy_map],
                "resolution_m": self.map_resolution_m,
                "position": (
                    self._current_position.copy()
                    if self._current_position is not None
                    else None
                ),
            }

    def _record_pose(self, sample: PoseSample) -> None:
        if (
            self._position_history
            and sample.timestamp < self._position_history[-1].timestamp
        ):
            logger.warning(
                "Ignoring out-of-order pose at %.6f; latest pose is %.6f",
                sample.timestamp,
                self._position_history[-1].timestamp,
            )
            return
        if (
            self._position_history
            and sample.timestamp == self._position_history[-1].timestamp
        ):
            self._position_history[-1] = sample
        else:
            self._position_history.append(sample)
        self._match_pending_lidar_scans()

    def _match_pending_lidar_scans(self) -> None:
        while self._pending_lidar_scans and self._position_history:
            scan = self._pending_lidar_scans[0]
            if scan.timestamp < self._position_history[0].timestamp:
                self._pending_lidar_scans.popleft()
                logger.warning(
                    "Dropping LiDAR scan at %.6f: it predates retained pose history",
                    scan.timestamp,
                )
                continue

            position = self.interpolate_position_at(scan.timestamp)
            if position is None:
                if scan.timestamp > self._position_history[-1].timestamp:
                    return
                self._pending_lidar_scans.popleft()
                logger.warning(
                    "Dropping LiDAR scan at %.6f: no bracketing poses are available",
                    scan.timestamp,
                )
                continue

            if len(self._matched_lidar_scans) >= MATCHED_LIDAR_LIMIT:
                logger.error(
                    "Matched LiDAR queue is full; dropping scan at %.6f",
                    self._matched_lidar_scans[0].timestamp,
                )
                self._matched_lidar_scans.popleft()
            self._matched_lidar_scans.append(
                MatchedLidarScan(
                    timestamp=scan.timestamp,
                    points=scan.payload,
                    position=position,
                )
            )
            self._pending_lidar_scans.popleft()

    def tourbot_footprint_cells(
        self,
        center_x: float,
        center_y: float,
        heading_degrees: float,
        map_width: int,
        map_height: int,
    ) -> set[tuple[int, int]]:
        angle = math.radians(heading_degrees)
        forward_x = math.sin(angle)
        forward_y = -math.cos(angle)
        side_x = math.cos(angle)
        side_y = math.sin(angle)
        half_length = self._base_length_cells / 2.0
        half_width = self._base_width_cells / 2.0

        cells: set[tuple[int, int]] = set()
        for y in range(map_height):
            offset_y = y + 0.5 - center_y
            for x in range(map_width):
                offset_x = x + 0.5 - center_x
                forward_offset = offset_x * forward_x + offset_y * forward_y
                side_offset = offset_x * side_x + offset_y * side_y
                if (
                    abs(forward_offset) <= half_length
                    and abs(side_offset) <= half_width
                ):
                    cells.add((x, y))
        return cells

    @staticmethod
    def quaternion_to_map_heading_deg(orientation: dict[str, float]) -> float:
        """Convert an FLU-frame quaternion to clockwise map heading from -Y."""
        x, y, z, w = (
            orientation["x"],
            orientation["y"],
            orientation["z"],
            orientation["w"],
        )
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm == 0.0:
            raise ValueError("Orientation quaternion must have a finite, non-zero norm")

        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        heading_degrees = (90 - math.degrees(yaw)) % 360.0
        if math.isclose(heading_degrees, 360.0, rel_tol=0.0, abs_tol=1e-12):
            return 0.0
        return heading_degrees
