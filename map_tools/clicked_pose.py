#! /usr/bin/env python3

from clicked_pose_cli import init_cli
import rospy
from pose import SavePoses
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys

class SaveClickedPoses(SavePoses):
    def __init__(self):
        self.args = init_cli(sys.argv[1:])
        self.pose_counter = 0
        self._pose_sub = rospy.Subscriber(
            "clicked_pose", PoseWithCovarianceStamped, self.pose_callback
        )

    def pose_callback(self, msg):
        self._pose = msg
        self.write_to_file("pose" + str(self.pose_counter), self.args.output, self.args.euler)
        self.pose_counter += 1


def main():
    rospy.init_node("clicked_pose_recorder", log_level=rospy.INFO)
    save_pose_object = SaveClickedPoses()
    rospy.spin()


if __name__ == "__main__":
    main()
