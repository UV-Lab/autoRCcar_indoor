import os
import argparse
import yaml
import numpy as np
import pandas
import json
from scipy.spatial.transform import Rotation as R


def pose_quat_to_se3(pos, quat):
    """
    Convert position and quaternion to SE3 transformation matrix.

    Parameters:
    - pos: Tuple of position (x, y, z)
    - quat: Tuple of quaternion (x, y, z, w)

    Returns:
    - SE3 transformation matrix as a numpy array
    """
    # Convert quaternion to rotation matrix
    rotation_matrix = R.from_quat(quat).as_matrix()

    # Create SE3 transformation matrix
    se3_matrix = np.eye(4)  # Initialize 4x4 identity matrix
    se3_matrix[:3, :3] = rotation_matrix  # Set rotation part
    se3_matrix[:3, 3] = pos  # Set translation part

    return se3_matrix


def find_closest_time(ref_time, times, start_index, tol_ms=10):
    tol_ns = tol_ms * 1000000

    diff_prev = abs(ref_time - times[start_index])
    is_found = False
    for i in range(start_index + 1, len(times)):
        diff = abs(ref_time - times[i])
        # If time difference start to increase, compare previous diff with tolerance
        if diff > diff_prev:
            # Returns true If previous diff is smaller than tolerance
            # Updates index as previous one in both cases for next search
            start_index = i - 1
            if diff_prev < tol_ns:
                is_found = True
            break

        diff_prev = diff

    return is_found, start_index


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Open pose csv file and match images and poses, then make transformation.json file."
    )

    parser.add_argument(
        "--base_path", required=True, help="Path to the base dataset folder."
    )
    parser.add_argument(
        "--calib_yaml",
        required=False,
        help="Path to the calibration file in YAML format. Default is base_path/calibration.yaml.",
    )
    parser.add_argument(
        "--image_folder",
        required=False,
        help="Path to the image(timestamp.png) folder. Default is base_path/images/",
    )
    parser.add_argument(
        "--pose_csv",
        required=False,
        help="Path to the camera pose csv(time, pos xyz, quat xyzw) file. Default is base_path/poses.csv",
    )

    args = parser.parse_args()
    base_path = args.base_path

    calib_yaml = args.calib_yaml or os.path.join(base_path, "calibration.yaml")
    if not os.path.isfile(calib_yaml):
        raise FileNotFoundError(f"Calibration file '{calib_yaml}' does not exist.")

    # output_dir = args.output_dir or os.path.join(
    #     os.path.dirname(os.path.abspath(bag_file))
    # )

    images_path = args.image_folder or os.path.join(base_path, "images")
    if not os.path.isdir(images_path):
        raise FileNotFoundError(f"Image folder '{images_path}' does not exist.")
    image_times = [int(f[:-4]) for f in os.listdir(images_path) if f.endswith(".png")]
    image_times.sort()

    poses_csv = args.pose_csv or os.path.join(base_path, "poses.csv")
    if not os.path.isfile(poses_csv):
        raise FileNotFoundError(f"Pose csv file '{poses_csv}' does not exist.")

    transformations = []

    print("Find closest pose for each image...")
    csv_reads = pandas.read_csv(poses_csv)
    csv_index = 0
    num_missed = 0
    for image_time in image_times:

        is_found, csv_index = find_closest_time(
            image_time, csv_reads["timestamp"], csv_index
        )

        if not is_found:
            num_missed = num_missed + 1
            continue

        timestamp = int(csv_reads.iloc[csv_index]["timestamp"])
        pos = csv_reads.loc[csv_index, ["pos_x", "pos_y", "pos_z"]].to_numpy()
        quat = csv_reads.loc[
            csv_index, ["quat_x", "quat_y", "quat_z", "quat_w"]
        ].to_numpy()

        se3 = pose_quat_to_se3(pos, quat)

        transformations.append(
            {
                "timestamp": timestamp,  # Include timestamp if needed for other uses
                "transform_matrix": se3.tolist(),  # Convert numpy array to list for JSON serialization
            }
        )

    # if not os.path.exists(output_dir):
    #     raise FileNotFoundError(f"Output directory '{output_dir}' does not exist.")

    # output_file_path = os.path.join(os.path.abspath(output_dir), (args.topic + ".csv"))

    print(
        "#images not synced with poses / #total images:",
        num_missed,
        "/",
        len(image_times),
    )

    # # Read the YAML file
    # with open(calib_yaml, "r") as yaml_file:
    #     yaml_data = yaml.safe_load(yaml_file)

    # camera_matrix = yaml_data["camera_matrix"]
    # distortion_coeffs = yaml_data["distortion_coefficients"]

    # json_data = {
    #     "camera_angle_x": 2
    #     * np.arctan(camera_matrix["data"][2] / camera_matrix["data"][0]),
    #     "camera_angle_y": 2
    #     * np.arctan(camera_matrix["data"][5] / camera_matrix["data"][4]),
    #     "fl_x": camera_matrix["data"][0],
    #     "fl_y": camera_matrix["data"][4],
    #     "k1": distortion_coeffs["data"][0],
    #     "k2": distortion_coeffs["data"][1],
    #     "p1": distortion_coeffs["data"][2],
    #     "p2": distortion_coeffs["data"][3],
    #     "cx": camera_matrix["data"][2],
    #     "cy": camera_matrix["data"][5],
    #     "w": 1920,  # Replace with actual width
    #     "h": 1080,  # Replace with actual height
    #     "aabb_scale": 4,
    # }

    # print("Progress {:2.2%}".format(i / len(topics)), end="\r")
