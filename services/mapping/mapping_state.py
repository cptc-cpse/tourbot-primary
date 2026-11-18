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
OAK_STALL_READINGS = 3

# Constants for the Tourbot's approximate physical dimensions in meters
TOURBOT_BASE_WIDTH_M = 0.5
TOURBOT_BASE_LENGTH_M = 0.6
TOURBOT_BODY_LENGTH_M = 0.3 # Should be the distance between OAK cameras

logger = logging.getLogger(__name__)


class MappingState:
    """Own and update the occupancy map, robot pose history, and matched scans."""

    def __init__(
        self,
        map_resolution_m: float | None = None,
        testing_static_lidar: bool | None = None,
    ) -> None:
        self.map_resolution_m = (
            get_config_value("map_resolution_m")
            if map_resolution_m is None
            else map_resolution_m
        )
        self.testing_static_lidar = (
            get_config_value("testing_static_lidar")
            if testing_static_lidar is None
            else testing_static_lidar
        )
        self._base_width_cells = round(TOURBOT_BASE_WIDTH_M / self.map_resolution_m)
        self._base_length_cells = round(TOURBOT_BASE_LENGTH_M / self.map_resolution_m)
        self._map_cells: dict[tuple[int, int], int] = {}
        self._cell_timestamps: dict[tuple[int, int], float] = {}
        self._map_bounds: tuple[int, int, int, int] | None = None
        self._current_pose: PoseSample | None = None
        self._static_lidar_pose: PoseSample | None = None
        self._position_history: deque[PoseSample] = deque(maxlen=POSE_HISTORY_LIMIT)
        self._pending_lidar_scans: deque[Message] = deque()
        self._matched_lidar_scans: deque[MatchedLidarScan] = deque()
        self._consecutive_oak_stall_readings = 0
        self._oak_stall_reported = False
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
            with self._lock:
                self._consecutive_oak_stall_readings += 1
                stall_readings = self._consecutive_oak_stall_readings
                should_raise = (
                    stall_readings >= OAK_STALL_READINGS
                    and not self._oak_stall_reported
                )
                if should_raise:
                    self._oak_stall_reported = True

            if should_raise:
                error = (
                    "OAK odometry returned default pose values for "
                    f"{stall_readings} consecutive readings"
                )
                logger.error(error)
                raise RuntimeError(error)

            if not self._oak_stall_reported:
                logger.warning(
                    "Default OAK pose detected (%d/%d consecutive readings)",
                    stall_readings,
                    OAK_STALL_READINGS,
                )
            return

        with self._lock:
            self._consecutive_oak_stall_readings = 0
            self._oak_stall_reported = False

        grid_x = position["x"] / self.map_resolution_m
        grid_y = position["y"] / self.map_resolution_m
        heading_degrees = self.quaternion_to_map_heading_deg(orientation)
        try:
            origin_x = round(map_origin["x"] / self.map_resolution_m)
            origin_y = round(map_origin["y"] / self.map_resolution_m)
            if (
                not isinstance(map_width, int)
                or not isinstance(map_height, int)
                or map_width <= 0
                or map_height <= 0
                or len(oak_map) != map_height
                or any(len(row) != map_width for row in oak_map)
            ):
                raise ValueError("OAK map dimensions do not match its grid")
        except (KeyError, TypeError, ValueError) as error:
            logger.warning("Ignoring invalid OAK map dimensions or origin: %s", error)
            return

        with self._lock:
            self._include_bounds(
                origin_x,
                origin_y,
                origin_x + map_width - 1,
                origin_y + map_height - 1,
            )
            for y, row in enumerate(oak_map):
                for x, value in enumerate(row):
                    cell_value = self._classify_oak_cell(value)
                    if cell_value is None:
                        continue
                    self._update_cell(
                        (origin_x + x, origin_y + y),
                        cell_value,
                        message.timestamp,
                    )

            self._current_pose = PoseSample(
                timestamp=message.timestamp,
                x=grid_x,
                y=grid_y,
                heading_degrees=heading_degrees,
            )
            if self.testing_static_lidar and self._static_lidar_pose is None:
                self._static_lidar_pose = self._current_pose
                logger.info(
                    "Locked static LiDAR pose at x=%.3f, y=%.3f, heading=%.1f°",
                    self._static_lidar_pose.x,
                    self._static_lidar_pose.y,
                    self._static_lidar_pose.heading_degrees,
                )
            boundary = self.tourbot_boundary_vertices(
                grid_x,
                grid_y,
                heading_degrees,
            )
            self._include_bounds(
                math.floor(min(point["x"] for point in boundary)),
                math.floor(min(point["y"] for point in boundary)),
                math.ceil(max(point["x"] for point in boundary)) - 1,
                math.ceil(max(point["y"] for point in boundary)) - 1,
            )
            self._record_pose(
                self._current_pose
            )

    @staticmethod
    def _classify_oak_cell(value: int) -> int | None:
        # RTAB-Map's 8-bit map image uses 89 for unknown, darker for occupied,
        # and brighter for free cells.
        if value == 89 or value == -1:
            return None
        return 2 if value < 89 else 0

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
            if self._map_bounds is None:
                return None

            min_x, min_y, max_x, max_y = self._map_bounds
            width = max_x - min_x + 1
            height = max_y - min_y + 1
            grid = [[-1 for _ in range(width)] for _ in range(height)]

            for (world_x, world_y), value in self._map_cells.items():
                grid_y = world_y - min_y
                grid_x = world_x - min_x
                grid[grid_y][grid_x] = value

            position: dict[str, int | float] | None = None
            robot_boundary: list[dict[str, float]] | None = None
            if self._current_pose is not None:
                position = {
                    "x": math.floor(self._current_pose.x) - min_x,
                    "y": math.floor(self._current_pose.y) - min_y,
                    "orientation": self._current_pose.heading_degrees,
                }
                robot_cells = self.tourbot_footprint_cells(
                    self._current_pose.x - min_x,
                    self._current_pose.y - min_y,
                    self._current_pose.heading_degrees,
                    width,
                    height,
                )
                for x, y in robot_cells:
                    grid[y][x] = 1
                robot_boundary = [
                    {"x": point["x"] - min_x, "y": point["y"] - min_y}
                    for point in self.tourbot_boundary_vertices(
                        self._current_pose.x,
                        self._current_pose.y,
                        self._current_pose.heading_degrees,
                    )
                ]

            return {
                "map": grid,
                "resolution_m": self.map_resolution_m,
                "map_origin": {
                    "x": min_x * self.map_resolution_m,
                    "y": min_y * self.map_resolution_m,
                },
                "position": position,
                "robot_boundary": robot_boundary,
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
            if self.testing_static_lidar and self._static_lidar_pose is not None:
                position = PoseSample(
                    timestamp=scan.timestamp,
                    x=self._static_lidar_pose.x,
                    y=self._static_lidar_pose.y,
                    heading_degrees=self._static_lidar_pose.heading_degrees,
                )
            else:
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

                if self.testing_static_lidar:
                    self._static_lidar_pose = position

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
            self._integrate_lidar_scan(self._matched_lidar_scans[-1])
            self._pending_lidar_scans.popleft()

    def _integrate_lidar_scan(self, scan: MatchedLidarScan) -> None:
        if not isinstance(scan.points, list):
            logger.warning(
                "Ignoring LiDAR scan at %.6f: expected a list of points",
                scan.timestamp,
            )
            return

        start = (math.floor(scan.position.x), math.floor(scan.position.y))
        invalid_points = 0
        for point in scan.points:
            if not isinstance(point, dict):
                invalid_points += 1
                continue
            angle_degrees = point.get("angle_deg")
            distance_mm = point.get("distance_mm")
            if (
                isinstance(angle_degrees, bool)
                or not isinstance(angle_degrees, (int, float))
                or isinstance(distance_mm, bool)
                or not isinstance(distance_mm, (int, float))
                or not math.isfinite(angle_degrees)
                or not math.isfinite(distance_mm)
                or distance_mm <= 0
            ):
                invalid_points += 1
                continue

            # LiDAR zero degrees is assumed to point along the robot's forward axis.
            angle = math.radians(
                (scan.position.heading_degrees + angle_degrees) % 360.0
            )
            distance_cells = distance_mm / 1000.0 / self.map_resolution_m
            endpoint = (
                math.floor(scan.position.x + distance_cells * math.sin(angle)),
                math.floor(scan.position.y - distance_cells * math.cos(angle)),
            )
            ray = self._bresenham_cells(start, endpoint)
            for cell in ray[:-1]:
                self._update_cell(cell, 0, scan.timestamp)
                self._include_cell(cell)
            self._update_cell(endpoint, 2, scan.timestamp)
            self._include_cell(endpoint)

        if invalid_points:
            logger.warning(
                "Ignored %d invalid points in LiDAR scan at %.6f",
                invalid_points,
                scan.timestamp,
            )

    def _update_cell(
        self,
        cell: tuple[int, int],
        value: int,
        timestamp: float,
    ) -> None:
        previous_timestamp = self._cell_timestamps.get(cell)
        if previous_timestamp is not None:
            if timestamp < previous_timestamp:
                return
            if (
                timestamp == previous_timestamp
                and self._map_cells.get(cell) == 2
                and value == 0
            ):
                return

        self._map_cells[cell] = value
        self._cell_timestamps[cell] = timestamp

    def _include_cell(self, cell: tuple[int, int]) -> None:
        self._include_bounds(cell[0], cell[1], cell[0], cell[1])

    def _include_bounds(
        self,
        min_x: int,
        min_y: int,
        max_x: int,
        max_y: int,
    ) -> None:
        if self._map_bounds is None:
            self._map_bounds = (min_x, min_y, max_x, max_y)
            return

        old_min_x, old_min_y, old_max_x, old_max_y = self._map_bounds
        self._map_bounds = (
            min(old_min_x, min_x),
            min(old_min_y, min_y),
            max(old_max_x, max_x),
            max(old_max_y, max_y),
        )

    @staticmethod
    def _bresenham_cells(
        start: tuple[int, int],
        end: tuple[int, int],
    ) -> list[tuple[int, int]]:
        x, y = start
        end_x, end_y = end
        delta_x = abs(end_x - x)
        step_x = 1 if x < end_x else -1
        delta_y = -abs(end_y - y)
        step_y = 1 if y < end_y else -1
        error = delta_x + delta_y
        cells = []

        while True:
            cells.append((x, y))
            if x == end_x and y == end_y:
                return cells

            doubled_error = 2 * error
            if doubled_error >= delta_y:
                error += delta_y
                x += step_x
            if doubled_error <= delta_x:
                error += delta_x
                y += step_y

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

    def tourbot_boundary_vertices(
        self,
        center_x: float,
        center_y: float,
        heading_degrees: float,
    ) -> list[dict[str, float]]:
        angle = math.radians(heading_degrees)
        forward_x = math.sin(angle)
        forward_y = -math.cos(angle)
        side_x = math.cos(angle)
        side_y = math.sin(angle)
        half_length = self._base_length_cells / 2.0
        half_width = self._base_width_cells / 2.0

        return [
            {
                "x": center_x + forward_x * half_length - side_x * half_width,
                "y": center_y + forward_y * half_length - side_y * half_width,
            },
            {
                "x": center_x + forward_x * half_length + side_x * half_width,
                "y": center_y + forward_y * half_length + side_y * half_width,
            },
            {
                "x": center_x - forward_x * half_length + side_x * half_width,
                "y": center_y - forward_y * half_length + side_y * half_width,
            },
            {
                "x": center_x - forward_x * half_length - side_x * half_width,
                "y": center_y - forward_y * half_length - side_y * half_width,
            },
        ]

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
