import argparse
import cv2
import os
from cv_bridge import CvBridge
from sqlite_ros2bag_parser import BagFileParser


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script to parse image from ros2 bag file."
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
        help="Directory path to create images folder (default: bag_file_dir/images).",
    )
    parser.add_argument(
        "--topic",
        "-t",
        nargs="?",
        default="image_topic",
        help="Name of iamge topic (default: image_topic).",
    )
    parser.add_argument(
        "--show_image",
        "-s",
        action="store_true",
        help="Show image if flag is present.",
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

    output_dir = os.path.join(output_dir, "images")
    os.makedirs(output_dir, exist_ok=True)

    print("Opening bag file, might take a while...")
    parser = BagFileParser(bag_file)
    img_topics = parser.get_messages("/" + args.topic)
    cv_bridge = CvBridge()

    for i, (timestamp, img_topic) in enumerate(img_topics):
        img_topic.step = img_topic.width * 3
        img = cv_bridge.imgmsg_to_cv2(img_topic, "bgr8")
        cv2.imwrite(os.path.join(output_dir, str(timestamp) + ".png"), img)
        print("Progress {:2.2%}".format(i / len(img_topics)), end="\r")
        if args.show_image:
            cv2.imshow("Image", img)
            cv2.waitKey(1)
