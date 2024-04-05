#! /usr/bin/env python3

from pose_cli import init_cli
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

        rospy.loginfo("Written " + pose_name + " to poses.txt")



def main():
    rospy.init_node("pose_recorder", log_level=rospy.INFO)
    args = init_cli(sys.argv[1:])
    save_pose_object = SavePoses()
    save_pose_object.write_to_file(args.name, args.output, args.euler)


if __name__ == "__main__":
    main()
