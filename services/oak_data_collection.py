import time
from typing import Any

import depthai as dai

from communication.mqtt_client import Client
import communication.mqtt_topics as topics
from models.message import Message

CAMERA_FPS = 60
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 400
IMU_FREQUENCY = 200
SLAM_PARAMS = {
    "RGBD/CreateOccupancyGrid": "true",
    "Grid/3D": "true",
    "Rtabmap/SaveWMState": "true",
}

POSE_TOPIC = topics.POSE_TOPIC
OCCUPANCY_GRID_TOPIC = topics.OCCUPANCY_GRID_TOPIC
OBSTACLE_POINT_CLOUD_TOPIC = topics.OBSTACLE_POINT_CLOUD_TOPIC
GROUND_POINT_CLOUD_TOPIC = topics.GROUND_POINT_CLOUD_TOPIC


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
        slam.obstaclePCL.link(publisher.inputObstaclePCL)
        slam.groundPCL.link(publisher.inputGroundPCL)

        pipeline.start()
        while pipeline.isRunning():
            time.sleep(1)


class MqttDataNode(dai.node.ThreadedHostNode):
    """Publish RTAB-Map pose, occupancy grid, and point clouds over MQTT."""

    def __init__(self) -> None:
        super().__init__()
        self.inputPose = dai.Node.Input(self)
        self.inputGrid = dai.Node.Input(self)
        self.inputObstaclePCL = dai.Node.Input(self)
        self.inputGroundPCL = dai.Node.Input(self)

    def run(self) -> None:
        with Client(qos=0, retain=False, process_id="oak_data_collection") as client:
            while self.mainLoop():
                pose = self.inputPose.get()
                if pose is None:
                    continue
                if not isinstance(pose, dai.TransformData):
                    raise TypeError(f"Expected TransformData, received {type(pose).__name__}")
                self._publish_pose(client, pose)

                grid = self.inputGrid.tryGet()
                if isinstance(grid, dai.MapData):
                    self._publish_grid(client, grid)
                elif grid is not None:
                    raise TypeError(f"Expected MapData, received {type(grid).__name__}")

                for input_cloud, topic in (
                    (self.inputObstaclePCL, OBSTACLE_POINT_CLOUD_TOPIC),
                    (self.inputGroundPCL, GROUND_POINT_CLOUD_TOPIC),
                ):
                    cloud = input_cloud.tryGet()
                    if isinstance(cloud, dai.PointCloudData):
                        self._publish_point_cloud(client, cloud, topic)
                    elif cloud is not None:
                        raise TypeError(
                            f"Expected PointCloudData, received {type(cloud).__name__}"
                        )

    @staticmethod
    def _publish_pose(client: Client, pose: dai.TransformData) -> None:
        translation = pose.getTranslation()
        quaternion = pose.getQuaternion()
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
        }
        client.publish(POSE_TOPIC, Message(payload).to_json())

    @staticmethod
    def _publish_grid(client: Client, grid: dai.MapData | None) -> None:
        if grid is None:
            return

        frame = grid.map.getCvFrame()
        payload = {
            "origin": {"x": grid.minX, "y": grid.minY},
            "width": frame.shape[1],
            "height": frame.shape[0],
            "data": frame.tolist(),
        }
        client.publish(OCCUPANCY_GRID_TOPIC, Message(payload).to_json())

    @staticmethod
    def _publish_point_cloud(
        client: Client,
        cloud: dai.PointCloudData | None,
        topic: str,
    ) -> None:
        if cloud is None:
            return

        points, colors = cloud.getPointsRGB()
        payload: dict[str, Any] = {
            "points": points.tolist(),
            "colors": colors.tolist(),
        }
        client.publish(topic, Message(payload).to_json())