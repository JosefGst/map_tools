#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys

class SavePoses(object):
    def __init__(self, pose_name):
        
        self._pose = PoseWithCovarianceStamped()
        self.poses_dict = {"pose1":self._pose}
        # self._pose_sub = rospy.Subscriber('diff_drive/odom', Odometry , self.sub_callback)
        self._pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.write_to_file(pose_name)

    def pose_callback(self, msg):
        self._pose = msg
    
    def write_to_file(self,pose_name):
        
        time.sleep(1)
        self.poses_dict["pose1"] = self._pose
        rospy.loginfo("Written pose1")
            
        
        with open('poses.txt', 'a') as file:
            file.write(str(pose_name) + ' = [' + str(self._pose.pose.pose.position.x))
            file.write(', ' + str(self._pose.pose.pose.position.y))
            file.write(', ' + str(self._pose.pose.pose.orientation.z))
            file.write(', ' + str(self._pose.pose.pose.orientation.w))
            file.write(']\n')
                    
        rospy.loginfo("Written all Poses to poses.txt file")
        


if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    if len(sys.argv) < 2:
        print("usage: sendgoals poses_to_file.py <pose_name>")
    else:
        save_spots_object = SavePoses(sys.argv[1])
    #rospy.spin() # mantain the service open.