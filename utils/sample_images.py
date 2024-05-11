import os
import argparse
import shutil
from scipy.spatial.transform import Rotation as R


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Open pose csv file and match images and poses, then make transformation.json file."
    )
    parser.add_argument(
        "--path",
        "-p",
        required=True,
        help="Path to images folder.",
    )

    # parser.add_argument(
    #     "--sampling_rate",
    #     required=True,
    #     help="Sampling rate",
    # )

    # parser.add_argument(
    #     "--output_path",
    #     required=False,
    #     help="Path to images folder.",
    # )

    args = parser.parse_args()
    path = args.path

    if not os.path.isdir(path):
        raise FileNotFoundError(f"Path does not exist: '{path}'")

    # output_dir = args.output_dir or os.path.join(
    #     os.path.dirname(os.path.abspath(bag_file))
    # )

    output_dir = os.path.join(path, "sampled_images")
    os.makedirs(output_dir)

    file_names = [f for f in os.listdir(path) if f.endswith(".png")]
    file_names.sort()

    sampling_rate = 5
    for i, file_name in enumerate(file_names):
        if i % sampling_rate != 0:
            continue

        file_path = os.path.join(path, file_name)
        output_path = os.path.join(output_dir, file_name)

        shutil.copyfile(file_path, output_path)
