import logging
import time

import depthai as dai

from communication.mqtt_client import Client
from configuration import get_config_value
from communication.message import Message

logger = logging.getLogger(__name__)

OAK_TOPIC: str = get_config_value("oak_topic")
PUBLISH_INTERVAL: float = get_config_value("oak_publish_interval_s")

class MqttDataNode(dai.node.ThreadedHostNode):
    """Publish RTAB-Map pose, occupancy grid, and point clouds over MQTT."""

    def __init__(self) -> None:
        super().__init__()
        self.inputPose = dai.Node.Input(self)
        self.inputGrid = dai.Node.Input(self)

    def run(self) -> None:
        with Client(qos=0, retain=False, process_id="oak_data_collection") as client:
            last_publish_time = 0.0
            while self.mainLoop():
                pose = self.inputPose.get()
                if pose is None:
                    continue
                if not isinstance(pose, dai.TransformData):
                    error = f"Expected TransformData, received {type(pose).__name__}"
                    logger.error(error)
                    raise TypeError(error)

                current_time = time.monotonic()
                if current_time - last_publish_time < PUBLISH_INTERVAL:
                    continue
                last_publish_time = current_time

                grid = self.inputGrid.tryGet()
                if isinstance(grid, dai.MapData):
                    self._publish_pose_and_grid(client, pose, grid)
                elif grid is not None:
                    error = f"Expected MapData, received {type(grid).__name__}"
                    logger.error(error)
                    raise TypeError(error)

    @staticmethod
    def _publish_pose_and_grid(client: Client, pose: dai.TransformData, grid: dai.MapData) -> None:
        translation = pose.getTranslation()
        quaternion = pose.getQuaternion()

        frame = grid.map.getCvFrame()
        payload = {
            "position": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
            },
            "orientation": {
                "x": quaternion.qx,
                "y": quaternion.qy,
                "z": quaternion.qz,
                "w": quaternion.qw,
            },
            "map_origin": {"x": grid.minX, "y": grid.minY},
            "map_width": frame.shape[1],
            "map_height": frame.shape[0],
            "map": frame.tolist(),
        }
        client.publish(OAK_TOPIC, Message(payload).to_json())