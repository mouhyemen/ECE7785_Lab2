#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge 
import cv2
import numpy as np


def callback(data):
	global pub, bridge
	cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
	cv2.imshow('image',cv_image)
	cv2.waitKey(5)
	cv2.destroyAllWindows()


if __name__ == '__main__':
	rospy.init_node('img_tracker', anonymous=True)
	img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,callback)
	
	global pub, bridge
	bridge = CvBridge()
	pub = rospy.Publisher('/ball_location', Point, queue_size=1)
	rospy.spin()