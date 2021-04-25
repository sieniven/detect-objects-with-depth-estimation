import launch


def generate_launch_description():
    """
    launch file to launch left and right camera publishers, and stereo image saver node (unsync)
    """
    hardware_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_hardware/lib/copilot_daa_hardware"
    calibration_launch_file_dir = \
        "/home/garuda/dev_ws/install/copilot_daa_calibration/lib/copilot_daa_calibration"
    left_image_topic, right_image_topic, saver_image_service = \
        "/stereo/left/image_raw", "/stereo/right/image_raw", "/stereo/image_saver"
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            name="right-camera",
            cmd=["./uvc_driver_node", "--device-id", "4", "--topic-to-publish", right_image_topic,
                 "--output-filename", "right-camera"],
            cwd=[hardware_launch_file_dir]
        ),
        launch.actions.ExecuteProcess(
            name="left-camera",
            cmd=["./uvc_driver_node", "--device-id", "2", "--topic-to-publish", left_image_topic,
                 "--output-filename", "left-camera"],
            cwd=[hardware_launch_file_dir]
        ),
        launch.actions.ExecuteProcess(
            name="stereo_image_saver",
            cmd=["./stereo_image_saver_node",
                 "--save-img-service-name", saver_image_service,
                 "--save-dir", "/home/garuda/dev_ws/src/copilot_daa_calibration/data/calib_data",
                 "--left-img-topic-name", left_image_topic,
                 "--right-img-topic-name", right_image_topic,
                 ],
            cwd=[calibration_launch_file_dir]
        )
    ])
