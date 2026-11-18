import time
from typing import Any

import depthai as dai

from configuration import get_config_value
from services.oak.mqtt_data_node import MqttDataNode

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