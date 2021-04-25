import launch
import os


def generate_launch_description():
    """
    launch file to launch left and right camera publishers, and stereo image saver node (sync)
    """
    hardware_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_hardware/lib/copilot_daa_hardware"
    calibration_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_calibration/lib/copilot_daa_calibration"
    image_topic, saver_image_service = "/synced_stereo/image_raw", "/stereo/image_saver"
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            name="synced-stereo-camera",
            cmd=["./uvc_driver_node", "--device-id", os.environ["SYNC_STEREO_CAM_ID"], "--topic-to-publish", image_topic,
                 "--output-filename", "synced-stereo-camera",
                 "--frame-res", "1280", "480"],
            cwd=[hardware_launch_file_dir]
        ),
        launch.actions.ExecuteProcess(
            name="synced_stereo_image_saver",
            cmd=["./stereo_image_saver_node",
                 "--synced", "True",
                 "--save-img-service-name", saver_image_service,
                 "--save-dir", "/home/garuda/dev_ws/src/copilot_daa_calibration/data/calib_data",
                 "--img-topic-name", image_topic],
            cwd=[calibration_launch_file_dir]
        )
    ])
