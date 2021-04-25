import launch
import os


def generate_launch_description():
    """
    launch file to launch camera info publishers and sync left and right camera.
    """
    hardware_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_hardware/lib/copilot_daa_hardware"
    calibration_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_calibration/lib/copilot_daa_calibration"
    left_image_topic, right_image_topic = "/left/camera_info", "/right/camera_info"
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            name="left-camera-info-publisher",
            cmd=["./publish_camera_info",
                 "--topic-to-publish", left_image_topic],
            cwd=[calibration_launch_file_dir]
        ),
        launch.actions.ExecuteProcess(
            name="right-camera-info-publisher",
            cmd=["./publish_camera_info",
                 "--topic-to-publish", right_image_topic],
            cwd=[calibration_launch_file_dir]
        ),
        launch.actions.ExecuteProcess(
            name="synced-stereo-camera",
            cmd=["./uvc_driver_node", "--device-id", os.environ["SYNC_STEREO_CAM_ID"],
                 "--topic-to-publish", "/synced_stereo/image_raw",
                 "--output-filename", "synced-stereo-camera",
                 "--frame-res", "1280", "480"],
            cwd=[hardware_launch_file_dir]
        )
    ])
