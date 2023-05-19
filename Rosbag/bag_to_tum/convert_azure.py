import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

rgb = '/home/sydemo/bag2tum/results/rgb/'  #rgb path
depth = '/home/sydemo/bag2tum/results/depth/'   #depth path
bridge = CvBridge()

file_handle1 = open('/home/sydemo/bag2tum/results/depth.txt', 'w')
file_handle2 = open('/home/sydemo/bag2tum/results/rgb.txt', 'w')

with rosbag.Bag('/home/sydemo/image.bag', 'r') as bag:
    for topic,msg,t in bag.read_messages():
        if topic == "/depth_to_rgb/image_raw":  #depth topic
            cv_image = bridge.imgmsg_to_cv2(msg)
            timestr = "%.6f" %  msg.header.stamp.to_sec()   #depth time stamp
            image_name = timestr+ ".png"
            path = "depth/" + image_name
            file_handle1.write(timestr + " " + path + '\n')
            cv2.imwrite(depth + image_name, cv_image)
        if topic == "/rgb/image_raw":   #rgb topic
            cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
            timestr = "%.6f" %  msg.header.stamp.to_sec()   #rgb time stamp
            image_name = timestr+ ".jpg"
            path = "rgb/" + image_name
            file_handle2.write(timestr + " " + path + '\n')
            cv2.imwrite(rgb + image_name, cv_image)
file_handle1.close()
file_handle2.close()
