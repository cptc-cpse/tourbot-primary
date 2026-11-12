import os
import serial
import time
from collections.abc import Iterable
from typing import NoReturn

from communication.mqtt_client import Client
import communication.mqtt_topics as topics
from models.message import Message
from models.lidar_point import LidarPoint

PORT = os.environ.get(
    "LIDAR_PORT",
    "COM3" if os.name == "nt" else "/dev/ttyUSB0",
)
BAUD = 230400
PACKET_SIZE = 47  # 2-byte header + 45 bytes from the tutorial
SLEEP_INTERVAL = 1  # Time to wait between reads, in seconds
TOPIC = topics.LIDAR_TOPIC

def crc8(data: bytes) -> int:
    # Polynomial and initialization match the tutorial's CRC table.
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x4D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def decode_packet(packet: bytes) -> list[LidarPoint]:
    start_angle = int.from_bytes(packet[4:6], "little")
    end_angle = int.from_bytes(packet[42:44], "little")
    step = ((end_angle - start_angle) % 36000) // 11

    points = []
    for i in range(12):
        offset = 6 + 3 * i
        distance_mm = int.from_bytes(packet[offset:offset + 2], "little")
        intensity = packet[offset + 2]
        angle_degrees = ((start_angle + step * i) % 36000) / 100
        points.append(LidarPoint(angle_degrees, distance_mm, intensity))

    return points


class ScanAccumulator:
    """Collect lidar points and emit a scan at each full rotation."""

    def __init__(self) -> None:
        self._points: list[LidarPoint] = []
        self._previous_angle: float | None = None
        self._started = False

    def add(self, points: Iterable[LidarPoint]) -> list[list[LidarPoint]]:
        completed_scans = []
        for point in points:
            if (
                self._previous_angle is not None
                and point.angle_deg < self._previous_angle
            ):
                if self._started:
                    completed_scans.append(self._points)
                self._started = True
                self._points = []

            self._points.append(point)
            self._previous_angle = point.angle_deg

        return completed_scans


def stream_data() -> NoReturn:
    publisher = Client(qos=0, retain=False, process_id="lidar_data_collection")
    buffer = bytearray()
    scan_accumulator = ScanAccumulator()

    with serial.Serial(PORT, BAUD, timeout=0.1) as lidar:
        print(f"Reading from {PORT}; press Ctrl+C to stop.")
        while True:
            buffer.extend(lidar.read(256))

            while True:
                header = buffer.find(b"\x54\x2C")
                if header < 0:
                    # Keep a trailing 0x54 in case the header is split across reads.
                    buffer[:] = b"\x54" if buffer[-1:] == b"\x54" else b""
                    break

                del buffer[:header]
                if len(buffer) < PACKET_SIZE:
                    break

                packet = bytes(buffer[:PACKET_SIZE])
                if crc8(packet[:-1]) != packet[-1]:
                    # Shift by one byte and search again to regain alignment.
                    del buffer[0]
                    continue

                del buffer[:PACKET_SIZE]
                points = decode_packet(packet)
                for scan in scan_accumulator.add(points):
                    message = Message(scan)
                    publisher.publish(TOPIC, message.to_json())
                    time.sleep(SLEEP_INTERVAL)