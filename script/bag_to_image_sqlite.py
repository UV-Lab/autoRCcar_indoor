import argparse
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import cv2
import os
import sys
from cv_bridge import CvBridge


# Reference: https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/?answer=377686#post-id-377686
class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()  # same as num topics
        # Deserialise all and timestamp them
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script to parse image topic from ros2 bag."
    )
    parser.add_argument("bag_file", help="Path to a bag file.")
    parser.add_argument(
        "output_path",
        nargs="?",
        default="",
        help="Path to the output folder (default: directory of the bag file).",
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default="image_topic",
        help="Name of the image topic (default: image_topic).",
    )

    args = parser.parse_args()
    bag_file = args.bag_file
    if not os.path.isfile(bag_file):
        raise FileNotFoundError(f"Bag file '{bag_file}' does not exist.")

    # script_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
    output_path = args.output_path or os.path.join(
        os.path.dirname(os.path.abspath(bag_file)), args.topic
    )
    if args.output_path:
        output_path = args.output_path
        if not os.path.exists(output_path):
            raise FileNotFoundError(f"Output directory '{output_path}' does not exist.")
        else:
            output_path = os.path.join(os.path.abspath(output_path), args.topic)
    os.makedirs(output_path, exist_ok=True)

    print("Opening bag file, might take a while...")
    parser = BagFileParser(bag_file)
    img_topics = parser.get_messages("/" + args.topic)
    cv_bridge = CvBridge()

    for i, [timestamp, img_topic] in enumerate(img_topics):
        img_topic.step = img_topic.width * 3
        img = cv_bridge.imgmsg_to_cv2(img_topic, "bgr8")
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        cv2.imwrite(os.path.join(output_path, str(timestamp) + ".png"), img)
        print("Progress {:2.2%}".format(i / len(img_topics)), end="\r")
