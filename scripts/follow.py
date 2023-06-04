#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self,):
    self.image_pub = rospy.Publisher("masked_image",Image,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw",Image,self.callback)

    self.vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
    #=======================================================================
    # Read Color
    #=======================================================================
    path = '/home/ros/ros_ws/competition_ws/src/competition_pkg/scripts/tracking_color.txt'
    f = open(path, 'r')
    tracking_color = f.readlines()[0].strip()
    f.close()
    rospy.loginfo(tracking_color)
    

    #=======================================================================
    # mask Red
    #=======================================================================
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # convert HSV
    h, w, ch = hsv.shape # get Image size
    hsv1 = hsv
    
    color_ranges = {
    'red': [(0, 64, 0), (30, 255, 255), (150, 64, 0), (179, 255, 255)],
    'green': [(40, 64, 0), (80, 255, 255)],
    'blue': [(90, 64, 0), (130, 255, 255)]
}
    if tracking_color == 'red':
        color = color_ranges['red']
    elif tracking_color == 'blue':
        color = color_ranges['blue']
    elif tracking_color == 'green':
        color = color_ranges['green']

    # Red HSV value range 1
    if tracking_color == 'red':
        hsv_min = np.array(color[0])
        hsv_max = np.array(color[1])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # Red HSV value range 2
        hsv_min = np.array(color[2])
        hsv_max = np.array(color[3])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # Mask red areas (255: red, 0: non-red)
        mask = mask1 + mask2
    else:
        hsv_min = np.array(color[0])
        hsv_max = np.array(color[1])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = mask1

    # masking
    masked_hsv = cv2.bitwise_and(hsv1, hsv1, mask=mask)

    # show image
    # cv2.imshow("Image window", cv2.cvtColor( masked, cv2.COLOR_HSV2BGR))
    # cv2.waitKey(3)

    #=======================================================================
    # Calculate red area
    #=======================================================================
    ones = np.ones((h, w))
    masked = cv2.bitwise_and( ones, ones, mask=mask)
    ones_l = sum(sum( masked[0:h, 0:int(w/3)]))           # Calculate left-hand side area
    ones_c = sum(sum( masked[0:h, int(w/3):int(2*w/3)]))  # Calculate center area
    ones_r = sum(sum( masked[0:h, int(2*w/3):w]))         # Calculate right-hand side area

    cmd_vel = Twist() # 速度のメッセージ型を用意
    if (ones_l > ones_c) and (ones_l > ones_r):   # Left is red
      rospy.loginfo("Left side")
      cmd_vel.linear.x  = 0.00  # set liner velocity
      cmd_vel.angular.z = 0.20  # set angular velocity
    elif (ones_c > ones_l) and (ones_c > ones_r):  # Center is red
      rospy.loginfo("Center")
      cmd_vel.linear.x  = 0.05
      cmd_vel.angular.z = 0.00
    elif (ones_r > ones_l) and (ones_r > ones_c): # Right is red
      rospy.loginfo("Right side")
      cmd_vel.linear.x  = 0.00
      cmd_vel.angular.z = -0.20
    else:
      cmd_vel.linear.x  = 0.00
      cmd_vel.angular.z = 0.00
    self.vel_pub.publish( cmd_vel)

    try:
      # OpenCV -> ROS
      self.image_pub.publish(self.bridge.cv2_to_imgmsg( cv2.cvtColor( masked_hsv, cv2.COLOR_HSV2BGR), "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('follow', anonymous=False)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

