import os
import numpy as np
import cv2
import glob
import yaml
import datetime


class StereoCalibrator(object):
    """
    A class to represent the stereo calibrator.
    """
    def __init__(self):
        """Initialise the class."""
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((9*6, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the calib_data.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.

        self.cal_path = "/home/garuda/dev_ws/src/copilot_daa_calibration/data/calib_data"
        self.read_images(self.cal_path)

    def read_images(self, cal_path):
        # type: (str) -> None
        """A function to read images and calibrate each camera individually.
        Args:
            cal_path: the proportional controller constant
        """
        images_right = glob.glob(os.path.join(cal_path, "right", "*.jpg"))
        images_left = glob.glob(os.path.join(cal_path, "left", "*.jpg"))
        images_left.sort()
        images_right.sort()
        distorted_images = {"left": [], "right": []}
        for i, fname in enumerate(images_right):
            img_l = cv2.imread(images_left[i])
            img_r = cv2.imread(images_right[i])

            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (9, 6), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (9, 6), None)

            # If found, add object points, image points (after refining them)
            self.objpoints.append(self.objp)

            if ret_l:
                cv2.cornerSubPix(gray_l, corners_l, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_l.append(corners_l)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_l, (9, 6),
                                                  corners_l, ret_l)
                cv2.imshow(images_left[i], img_l)
                cv2.waitKey(500)
                distorted_images["left"].append(img_l)

            if ret_r:
                cv2.cornerSubPix(gray_r, corners_r, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_r.append(corners_r)

                # Draw and display the corners
                ret_r = cv2.drawChessboardCorners(img_r, (9, 6),
                                                  corners_r, ret_r)
                cv2.imshow(images_right[i], img_r)
                cv2.waitKey(500)
                distorted_images["right"].append(img_r)
            img_shape = gray_l.shape[::-1]

        rms_error, self.__intrinsic_mtx_left, self.__dist_coef_left, self.__rotation_mtx_left, \
            self.__translation_mtx_left = cv2.calibrateCamera(
                self.objpoints, self.imgpoints_l, img_shape, None, None)
        self.__new_camera_mtx_left, roi = cv2.getOptimalNewCameraMatrix(
            self.__intrinsic_mtx_left, self.__dist_coef_left, img_shape, 1, img_shape)
        print("Left Camera Calibration RMS error: %s" % rms_error)
        rms_error, self.__intrinsic_mtx_right, self.__dist_coef_right, self.__rotation_mtx_right, \
            self.__translation_mtx_right = cv2.calibrateCamera(
                self.objpoints, self.imgpoints_r, img_shape, None, None)
        print("Right Camera Calibration RMS error: %s" % rms_error)
        self.__new_camera_mtx_right, roi = cv2.getOptimalNewCameraMatrix(
            self.__intrinsic_mtx_right, self.__dist_coef_right, img_shape, 1, img_shape)
        save_undistorted_images(distorted_image_filenames={"left": images_left, "right": images_right},
                                distorted_images=distorted_images,
                                intrinsic_mtx={"left": self.__intrinsic_mtx_left,
                                               "right": self.__intrinsic_mtx_right},
                                dist_coef={"left": self.__dist_coef_left, "right": self.__dist_coef_right},
                                new_camera_mtx={"left": self.__new_camera_mtx_left,
                                                "right": self.__new_camera_mtx_right})
        self.camera_model = self.stereo_calibrate(img_shape)

    def stereo_calibrate(self, image_shape):
        # type: (tuple) -> dict
        """ A function to perform stereo calibration and rectification.
        Args:
            image_shape: the proportional controller constant
        Returns:
            camera_model: calibrated stereo camera parameters
        """
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        rms_error, intrinsic_mtx_left, dist_coef_left, intrinsic_mtx_right, dist_coef_right, \
            rotation_mtx, translation_mtx, essential_mtx, fundamental_mtx = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_l,
            self.imgpoints_r, self.__new_camera_mtx_right, self.__dist_coef_left,
            self.__intrinsic_mtx_right, self.__dist_coef_right, image_shape,
                criteria=stereocalib_criteria, flags=flags)

        rect_mtx_left, rect_mtx_right, projection_mtx_left, projection_mtx_right, \
            disparity_to_depth_map_mtx, roi_left, roi_right = \
            cv2.stereoRectify(intrinsic_mtx_left, dist_coef_left, intrinsic_mtx_right,
                              dist_coef_right, image_shape, rotation_mtx, translation_mtx,
                              flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.9)
        print("Stereo Calibration RMS error: %s" % rms_error)

        # for i in range(len(self.r1)):
        #     print("--- pose[", i+1, "] ---")
        #     self.ext1, _ = cv2.Rodrigues(self.r1[i])
        #     self.ext2, _ = cv2.Rodrigues(self.r2[i])
        #     print('Ext1', self.ext1)
        #     print('Ext2', self.ext2)
        camera_model = {
            "image": {
                "width": image_shape[0],
                "height": image_shape[1]
            },
            "intrinsic": {
                "left": intrinsic_mtx_left.tolist(),
                "right": intrinsic_mtx_right.tolist()
            },
            "distortion": {
                "left": dist_coef_left.tolist(),
                "right": dist_coef_right.tolist()
            },
            "rectification": {
                "left": rect_mtx_left.tolist(),
                "right": rect_mtx_right.tolist()
            },
            "projection_mtx": {
                "left": projection_mtx_left.tolist(),
                "right": projection_mtx_right.tolist()
            },
            "disparity_to_depth_map_mtx": disparity_to_depth_map_mtx.tolist(),
            "new_camera_mtx": {
                "left": self.__new_camera_mtx_left.tolist(),
                "right": self.__new_camera_mtx_right.tolist()
            }
        }
        dir_to_save = "/home/garuda/dev_ws/src/copilot_daa_calibration/data/intrinsic"
        timestamp = (datetime.datetime.now()).strftime("%m/%d/%Y/%H:%M:%S")
        output_filename = os.path.join(dir_to_save, (timestamp + ".yaml").replace("/", "-"))
        with open(output_filename, 'w') as file:
            yaml.dump(camera_model, file, default_flow_style=None)
        cv2.destroyAllWindows()
        print("output filename: %s" % output_filename)
        return camera_model


def save_undistorted_images(distorted_image_filenames, distorted_images,
                            intrinsic_mtx, dist_coef, new_camera_mtx):
    # type: (dict, dict, dict, dict, dict) -> None
    """ A function to save undistorted images
    Args:
        distorted_image_filenames: the proportional controller constant
        distorted_images: distorted images captured from the stereo pair
        intrinsic_mtx: intrinsic matrix of each of the stereo pair camera
        dist_coef: distortion coefficient of each of the stereo pair camera
        new_camera_mtx: the new camera matrix of each camera from the stereo pair
    """
    dir_to_save = "/home/garuda/dev_ws/src/copilot_daa_calibration/data/undistorted"
    if not os.path.exists(os.path.join(dir_to_save, "left")):
        os.makedirs(os.path.join(dir_to_save, "left"))
    else:
        files = glob.glob(os.path.join(dir_to_save, "left", "*"))
        for file in files:
            os.remove(file)
    if not os.path.exists(os.path.join(dir_to_save, "right")):
        os.makedirs(os.path.join(dir_to_save, "right"))
    else:
        files = glob.glob(os.path.join(dir_to_save, "right", "*"))
        for file in files:
            os.remove(file)
    for index, distorted_image_filename in enumerate(distorted_image_filenames["left"]):
        undistorted_image = cv2.undistort(distorted_images["left"][index], intrinsic_mtx["left"],
                                          dist_coef["left"],
                                          np.hstack((new_camera_mtx["left"], np.zeros((3, 1)))))
        cv2.imwrite(os.path.join(dir_to_save, "left", os.path.basename(distorted_image_filename)),
                    undistorted_image)
    for index, distorted_image_filename in enumerate(distorted_image_filenames["right"]):
        undistorted_image = cv2.undistort(distorted_images["right"][index], intrinsic_mtx["right"],
                                          dist_coef["right"],
                                          np.hstack((new_camera_mtx["right"], np.zeros((3, 1)))))
        cv2.imwrite(os.path.join(dir_to_save, "right", os.path.basename(distorted_image_filename)),
                    undistorted_image)


if __name__ == '__main__':
    StereoCalibrator()
