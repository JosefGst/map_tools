#! /usr/bin/env python3

import argparse
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import time
import sys
import math

import tf.transformations


class SavePoses(object):
    def __init__(self):
        self._pose = PoseWithCovarianceStamped()
        self._pose_sub = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.pose_callback
        )

    def pose_callback(self, msg):
        self._pose = msg

    def write_to_file(self, pose_name, output_file, euler=False):
        time.sleep(1)
        with open(output_file, "a") as file:
            file.write(
                str(pose_name) + " = [" + format(self._pose.pose.pose.position.x, ".2f")
            )
            file.write(", " + format(self._pose.pose.pose.position.y, ".2f"))
            if euler:  # write the euler angles
                _, _, yaw = tf.transformations.euler_from_quaternion([0, 0,
                    self._pose.pose.pose.orientation.z,
                    self._pose.pose.pose.orientation.w]
                )
                file.write(", " + format(math.degrees(yaw), ".2f"))

            else:  # write the quaternion
                file.write(", " + format(self._pose.pose.pose.orientation.z, ".2f"))
                file.write(", " + format(self._pose.pose.pose.orientation.w, ".2f"))

            file.write("]\n")

        rospy.loginfo("Written Pose to poses.txt")


def init_cli(args):
    parser = argparse.ArgumentParser(
        prog="pose",
        description="Record the current amcl_pose and orientation in quaternion of the robot and save it to a file. \nExample usage: rosrun map_tools pose.py \nExample usage: rosrun map_tools pose.py -o my_pose.txt -e ", formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "name",
        nargs="?",
        default="pose",
        help='Give the pose a name. Default is "pose".',
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="poses.txt",
        help='Outputs to the specified file path eg."my_pose.txt".',
    )
    parser.add_argument(
        "-e",
        "--euler",
        action="store_true",
        help="Output the orientation in Euler angles.",
    )
    parser.add_argument("-v", "--version", action="version", version="%(prog)s 0.1.0")

    return parser.parse_args(args)


def main():
    rospy.init_node("pose_recorder", log_level=rospy.INFO)
    args = init_cli(sys.argv[1:])
    save_pose_object = SavePoses()
    save_pose_object.write_to_file(args.name, args.output, args.euler)


if __name__ == "__main__":
    main()
