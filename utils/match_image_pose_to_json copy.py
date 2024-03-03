import os
import argparse
import yaml
import numpy as np
import pandas
import cv2
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


def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()


def calc_sharpness(imagePath):
    image = cv2.imread(imagePath)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    return fm


def to_rotmat(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = np.dot(a, b)
    # handle exception for the opposite direction input
    if c < -1 + 1e-10:
        return to_rotmat(a + np.random.uniform(-1e-2, 1e-2, 3), b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2 + 1e-10))


def closest_point_2_lines(
    oa, da, ob, db
):  # returns point closest to both rays of form o+t*d, and a weight factor that goes to 0 if the lines are parallel
    da = da / np.linalg.norm(da)
    db = db / np.linalg.norm(db)
    c = np.cross(da, db)
    denom = np.linalg.norm(c) ** 2
    t = ob - oa
    ta = np.linalg.det([t, db, c]) / (denom + 1e-10)
    tb = np.linalg.det([t, da, c]) / (denom + 1e-10)
    if ta > 0:
        ta = 0
    if tb > 0:
        tb = 0
    return (oa + ta * da + ob + tb * db) * 0.5, denom


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Open pose csv file and match images and poses, then make transformation.json file."
    )
    parser.add_argument(
        "--base_path",
        required=True,
        help="Path to the base dataset folder. Base folder should have images folder.",
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
    sampling_rate = 5
    for i, image_time in enumerate(image_times):
        if i % sampling_rate != 0:
            continue

        if i > 500:
            break

        is_found, csv_index = find_closest_time(
            image_time, csv_reads["timestamp"], csv_index
        )

        if not is_found:
            num_missed = num_missed + 1
            continue

        pos = csv_reads.loc[csv_index, ["pos_x", "pos_y", "pos_z"]].to_numpy()
        quat = csv_reads.loc[
            csv_index, ["quat_x", "quat_y", "quat_z", "quat_w"]
        ].to_numpy()

        se3 = pose_quat_to_se3(pos, quat)

        transformations.append(
            {
                "timestamp": image_time,  # Include timestamp if needed for other uses
                "transform_matrix": se3,
            }
        )

    # if not os.path.exists(output_dir):
    #     raise FileNotFoundError(f"Output directory '{output_dir}' does not exist.")

    # output_file_path = os.path.join(os.path.abspath(output_dir), (args.topic + ".csv"))

    print(
        "#images not synced with poses / #total images:",
        num_missed,
        "/",
        len(image_times) / sampling_rate,
    )

    with open(calib_yaml, "r") as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)

    camera_matrix = yaml_data["camera_matrix"]
    distortion_coeffs = yaml_data["distortion_coefficients"]

    json_data = {
        "camera_angle_x": 2
        * np.arctan(camera_matrix["data"][2] / camera_matrix["data"][0])
        * 180
        / np.pi,
        "camera_angle_y": 2
        * np.arctan(camera_matrix["data"][5] / camera_matrix["data"][4])
        * 180
        / np.pi,
        "fl_x": camera_matrix["data"][0],
        "fl_y": camera_matrix["data"][4],
        "k1": distortion_coeffs["data"][0],
        "k2": distortion_coeffs["data"][1],
        "p1": distortion_coeffs["data"][2],
        "p2": distortion_coeffs["data"][3],
        "cx": camera_matrix["data"][2],
        "cy": camera_matrix["data"][5],
        "w": 1920,  # Replace with actual width
        "h": 1080,  # Replace with actual height
        "aabb_scale": 32,
    }

    up = np.zeros(3)
    for transformation in transformations:
        se3 = transformation["transform_matrix"]
        se3[0:3, 2] *= -1  # flip the y and z axis
        se3[0:3, 1] *= -1
        se3 = se3[[1, 0, 2, 3], :]
        se3[2, :] *= -1  # flip whole world upside down
        up += se3[0:3, 1]
        transformation["transform_matrix"] = se3

    up = up / np.linalg.norm(up)
    print("up vector was", up)
    rot = to_rotmat(up, [0, 0, 1])  # rotate up vector to [0,0,1]
    rot = np.pad(rot, [0, 1])
    rot[-1, -1] = 1

    for transformation in transformations:
        transformation["transform_matrix"] = np.matmul(
            rot, transformation["transform_matrix"]
        )  # rotate up to be the z axis

    totw = 0.0
    totp = np.array([0.0, 0.0, 0.0])
    for f in transformations:
        mf = f["transform_matrix"][0:3, :]
        for g in transformations:
            mg = g["transform_matrix"][0:3, :]
            p, w = closest_point_2_lines(mf[:, 3], mf[:, 2], mg[:, 3], mg[:, 2])
            if w > 0.00001:
                totp += p * w
                totw += w
    if totw > 0.0:
        totp /= totw
    print(totp)  # the cameras are looking at totp

    for f in transformations:
        f["transform_matrix"][0:3, 3] -= totp

    avglen = 0.0
    for f in transformations:
        avglen += np.linalg.norm(f["transform_matrix"][0:3, 3])
    avglen /= len(transformations)
    print("avg camera distance from origin", avglen)
    for f in transformations:
        f["transform_matrix"][0:3, 3] *= 4.0 / avglen  # scale to "nerf sized"
        f["transform_matrix"] = f["transform_matrix"].tolist()

    print("Adding frame data...")
    json_data["frames"] = []
    for i, transformation in enumerate(transformations):

        file_path = os.path.join("images", str(transformation["timestamp"]) + ".png")
        frame = {
            # "file_path": "images/" + str(transformation["timestamp"]) + ".png",
            "file_path": file_path,
            "sharpness": calc_sharpness(os.path.join(base_path, file_path)),
            "transform_matrix": transformation["transform_matrix"],
        }
        json_data["frames"].append(frame)
        print("Progress {} / {}".format(i, len(transformations)), end="\r")

    print("\n")
    with open(os.path.join(base_path, "transforms.json"), "w") as outfile:
        json.dump(json_data, outfile, indent=2)

    print("Done!")
