import time
from typing import Any

import depthai as dai

from communication.mqtt_client import Client
from configuration import get_config_value
from models.message import Message

CAMERA_FPS: int = get_config_value("oak_camera_fps")
IMAGE_WIDTH: int = get_config_value("oak_image_width")
IMAGE_HEIGHT: int = get_config_value("oak_image_height")
IMU_FREQUENCY: int = get_config_value("oak_imu_frequency") 
SLAM_PARAMS: dict[str, str] = {
    "RGBD/CreateOccupancyGrid": "true",
    "Grid/3D": "true",
    "Rtabmap/SaveWMState": "true",
    "Grid/CellSize": str(get_config_value("map_resolution_m")),
}

OAK_TOPIC: str = get_config_value("oak_topic")
PUBLISH_INTERVAL: float = get_config_value("oak_publish_interval_s")


def stream_data() -> None:
    """Run OAK stereo SLAM and publish its outputs over MQTT."""
    with dai.Pipeline() as pipeline:
        left = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B,
            sensorFps=CAMERA_FPS,
        )
        right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C,
            sensorFps=CAMERA_FPS,
        )
        imu = pipeline.create(dai.node.IMU)
        odometry = pipeline.create(dai.node.RTABMapVIO)
        slam = pipeline.create(dai.node.RTABMapSLAM)
        stereo = pipeline.create(dai.node.StereoDepth)
        publisher = pipeline.create(MqttDataNode)

        slam.setParams(SLAM_PARAMS)
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            IMU_FREQUENCY,
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        stereo.setExtendedDisparity(False)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)
        stereo.initialConfig.setLeftRightCheckThreshold(10)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

        left.requestOutput((IMAGE_WIDTH, IMAGE_HEIGHT)).link(stereo.left)
        right.requestOutput((IMAGE_WIDTH, IMAGE_HEIGHT)).link(stereo.right)
        stereo.rectifiedLeft.link(odometry.rect)
        stereo.depth.link(odometry.depth)
        imu.out.link(odometry.imu)
        stereo.depth.link(slam.depth)
        stereo.rectifiedLeft.link(slam.rect)
        odometry.transform.link(slam.odom)

        slam.transform.link(publisher.inputPose)
        slam.occupancyGridMap.link(publisher.inputGrid)

        pipeline.start()
        while pipeline.isRunning():
            time.sleep(1)


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
                    raise TypeError(f"Expected TransformData, received {type(pose).__name__}")

                current_time = time.monotonic()
                if current_time - last_publish_time < PUBLISH_INTERVAL:
                    continue
                last_publish_time = current_time

                grid = self.inputGrid.tryGet()
                if isinstance(grid, dai.MapData):
                    self._publish_pose_and_grid(client, pose, grid)
                elif grid is not None:
                    raise TypeError(f"Expected MapData, received {type(grid).__name__}")

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