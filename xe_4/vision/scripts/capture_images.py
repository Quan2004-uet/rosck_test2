#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Đường dẫn lưu ảnh (chỉnh theo đúng cấu trúc của bạn)
SAVE_DIR = os.path.expanduser("~/test_ws/src/xe_4/vision/dataset/images/train")

# Tạo thư mục nếu chưa có
os.makedirs(SAVE_DIR, exist_ok=True)

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xe_4/camera1/image_raw", Image, self.callback)
        self.count = 0
        rospy.loginfo("Image saver initialized.")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = os.path.join(SAVE_DIR, f"image_{self.count:04}.jpg")
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved: {filename}")
            self.count += 1
        except Exception as e:
            rospy.logerr(f"Error converting/saving image: {e}")

if __name__ == '__main__':
    rospy.init_node('image_saver_node')
    saver = ImageSaver()
    rospy.spin()
