#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from PIL import Image as PILImage
import numpy as np
import cv2
import matplotlib.pyplot as plt

from ultralytics import YOLO

class ImageSegmentationNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to the compressed image topic from the ROS bag
        self.image_sub = rospy.Subscriber('/front/zed_node/rgb/image_rect_color/compressed', CompressedImage, self.image_callback)

        # # Publisher for the segmented image (this would be the input for ORB-SLAM3)
        # self.image_pub = rospy.Publisher('/camera/image_segmented', Image, queue_size=1)
        
        # Publisher for the segmented image (this would be the input for ORB-SLAM3. Named as the same topic required by ORB-SLAM3 for ease)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

    def process_segmentation(self, image):
         
        try:
            # model used to process the image
            my_new_model = YOLO('/home/offiong/yolo_model/best.pt')

            # Load image as NumPy array and resize to 640x640
            img2 = np.array(image.resize((640, 640), resample=PILImage.LANCZOS))
            
            # Default to the input image in case of any failure in segmentation
            segmented_image = img2
            
            results = my_new_model.predict(img2, conf=0.2, classes=[2,4], verbose=False) # Trunks is 4, Poles os 2. if need be use [2,4] to predict both

            # Validate predictions
            if results[0].masks is not None:
                masks = np.array(results[0].masks.data.cpu())
                combined_mask = np.any(masks, axis=0).astype(np.uint8)  # Combine instance masks into one mask
                segmented_image = img2 * combined_mask[:, :, np.newaxis]  # Multiply image by combined mask

            # Break after processing the first image (if you only want to process one)
            # break

        except Exception as e:
            rospy.logerr("Segmentation model error: {0}".format(e))

        return segmented_image
    
    def image_callback(self, msg):
        try:
            # Decompress the image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert the OpenCV image (BGR) to RGB for PIL
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert the NumPy array to a PIL image
            img = PILImage.fromarray(cv_image_rgb)
            
            # Perform segmentation (replace with your model's processing function)
            segmented_image = self.process_segmentation(img)

            # Convert the segmented image back to ROS Image message
            segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")

            # Publish the segmented image
            self.image_pub.publish(segmented_msg)

            # #  Maintain the loop rate                
            rate = rospy.Rate(10)  # Set a publishing rate (e.g., 10 Hz)
            # rate.sleep()

        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('image_segmentation_node_2', anonymous=True)
    node = ImageSegmentationNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")