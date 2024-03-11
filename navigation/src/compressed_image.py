#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def publish_camera_image():
    # Initialize ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Initialize image publisher using image_transport
    image_pub = rospy.Publisher('camera_image/compressed', CompressedImage, queue_size=10)
    
    # Initialize OpenCV camera capture
    cap = cv2.VideoCapture(2)  # Change the parameter if your camera is not the default
    
    # Initialize CvBridge
    bridge = CvBridge()

    rate = rospy.Rate(150)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # Capture frame from the camera
        ret, frame = cap.read()

        if ret:
            # Convert OpenCV image to ROS Compressed Image message
            ros_image = bridge.cv2_to_compressed_imgmsg(frame)
            
            # Publish the ROS Compressed Image message
            image_pub.publish(ros_image)

        rate.sleep()

    # Release the camera capture when the node is shut down
    cap.release()

if __name__ == '__main__':
    try:
        publish_camera_image()
    except rospy.ROSInterruptException:
        pass
