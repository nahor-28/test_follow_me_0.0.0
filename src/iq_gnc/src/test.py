import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def imgCallback(data):
  cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
  cv2.imshow("Raw Image", cv_img)
  cv2.waitKey(3)

def main():
  print("Hey Universe!")
  rospy.init_node('my_test_node')
  img_sub =rospy.Subscriber("/camera/image_raw", Image, imgCallback)
  rospy.spin()

if __name__ == "__main__":
  main()

