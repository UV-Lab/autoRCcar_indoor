import argparse
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import os
import csv


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
        "output_dir",
        nargs="?",
        default="",
        help="Path to the output folder (default: directory of the bag file).",
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default="nav_topic",
        help="Name of tpic (default: nav_topic).",
    )

    args = parser.parse_args()
    bag_file = args.bag_file
    if not os.path.isfile(bag_file):
        raise FileNotFoundError(f"Bag file '{bag_file}' does not exist.")

    output_dir = args.output_dir or os.path.join(
        os.path.dirname(os.path.abspath(bag_file))
    )

    if not os.path.exists(output_dir):
        raise FileNotFoundError(f"Output directory '{output_dir}' does not exist.")

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
        for i, [timestamp, topic] in enumerate(topics):
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
