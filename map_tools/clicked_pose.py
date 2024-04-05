#! /usr/bin/env python3

import rospy
from pose import SavePoses
from geometry_msgs.msg import PoseWithCovarianceStamped

class SaveClickedPoses(SavePoses):
    def __init__(self):
        self.pose_counter = 0
        self._pose_sub = rospy.Subscriber(
            "clicked_pose", PoseWithCovarianceStamped, self.pose_callback
        )

    def pose_callback(self, msg):
        self._pose = msg
        self.write_to_file("pose" + str(self.pose_counter), "poses.txt")
        self.pose_counter += 1



def main():
    rospy.init_node("clicked_pose_recorder", log_level=rospy.INFO)
    save_pose_object = SaveClickedPoses()
    rospy.spin()


if __name__ == "__main__":
    main()
