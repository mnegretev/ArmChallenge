#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def main():
    print("CAPTURING ONE POINT CLOUD...")
    rospy.init_node("cloud_capturer")
    cloud = rospy.wait_for_message("/camera/depth/points", PointCloud2)
    
    print(cloud.header.frame_id)

if __name__ == "__main__":
    main()
