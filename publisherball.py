#!/usr/bin/env python

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_msg", Image, queue_size=100)
        self.bridge = CvBridge()

    def pub(self, cv_image):
       try:
         self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
       except CvBridgeError as e:
         print(e)
        
def main():
    ic = image_converter()
    rospy.init_node('image_pub', anonymous=True)
    video_capture = cv2.VideoCapture('../topic03_perception/video/tennis-ball-video.mp4')

    while(True):
        ret, rgb_image = video_capture.read()
        if ret == True:
            # cv2.imshow("RGB Image",rgb_image)
            ic.pub(rgb_image)
        else:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()