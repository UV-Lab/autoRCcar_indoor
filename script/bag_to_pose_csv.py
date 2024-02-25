import argparse
import os
import csv
from sqlite_ros2bag_parser import BagFileParser


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script to parse pose from ros2 bag file."
    )
    parser.add_argument(
        "--input_bag_path",
        "-i",
        required=True,
        help="Path to a bag file.",
    )
    parser.add_argument(
        "--output_dir",
        "-o",
        nargs="?",
        default="",
        help="Directory Path to write [topic_name].csv (default: bag_file_dir/).",
    )
    parser.add_argument(
        "--topic",
        "-t",
        nargs="?",
        default="nav_topic",
        help="Name of pose topic (default: nav_topic).",
    )

    args = parser.parse_args()
    bag_file = args.input_bag_path
    if not os.path.isfile(bag_file):
        raise FileNotFoundError(f"Bag file '{bag_file}' does not exist.")

    output_dir = None
    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
        if not os.path.exists(output_dir):
            raise FileNotFoundError(f"Output directory '{output_dir}' does not exist.")
    else:
        output_dir = os.path.dirname(os.path.abspath(bag_file))

    output_file_path = os.path.join(os.path.abspath(output_dir), (args.topic + ".csv"))

    print("Opening bag file, might take a while...")
    parser = BagFileParser(bag_file)
    topics = parser.get_messages("/" + args.topic)

    labels = [
        "timestamp",
        "pos_x",
        "pos_y",
        "pos_z",
        "quat_x",
        "quat_y",
        "quat_z",
        "quat_w",
    ]
    with open(output_file_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(labels)
        for i, (timestamp, topic) in enumerate(topics):
            writer.writerow(
                [
                    timestamp,
                    topic.position.x,
                    topic.position.y,
                    topic.position.z,
                    topic.quaternion.x,
                    topic.quaternion.y,
                    topic.quaternion.z,
                    topic.quaternion.w,
                ]
            )
            print("Progress {:2.2%}".format(i / len(topics)), end="\r")
