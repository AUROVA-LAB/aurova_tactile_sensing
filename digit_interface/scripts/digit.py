#!/home/aurova/anaconda3/envs/digit_ros_torch/bin/python3

# first of all, you need to source the virtualenv. Then source the devel setup.bash file.

# ROS IMPORTS
import rospy
from sensor_msgs.msg import Image
import time
import cv2


# DIGIT LIBRARY
from digit_interface import Digit
from digit_interface import DigitHandler


'''
    setup_digit function: reads the sensor id from the launch file and initializes the sensor.
    Arguments:
        - None
    Returns: object class of the sensor library.
'''
def setup_digit(sensor_id):

    # get sensor id from launch file
    id_d = rospy.get_param(sensor_id)

    # initialize digit sensor with the usb serial
    d = Digit(id_d)
    d.connect()


    return d, id_d


'''
    main function: initialize the ros node and the publisher. Runs an infinite loop
        where reads images from the sensor and publish them in the topic.
    Arguments:
        - None
    Returns: None
'''
def main():

    # initialize digit sensor
    d45, id45 = setup_digit("/sensor_id1")
    d55, id55 = setup_digit("/sensor_id2")

    # initialize ros node
    rospy.init_node(f"digit_ros_node", anonymous=True)
    # create publisher with the topic and the type of the msg
    pub45 = rospy.Publisher(f"digit45/camera/image_color", Image, queue_size=10)
    pub55 = rospy.Publisher(f"digit55/camera/image_color", Image, queue_size=10)

    rate = rospy.Rate(60)

    # infinite loop
    while True:

        # comment if you want
        rospy.loginfo("Publishing Digit images!")

        # get image from the sensor
        digit45_img = d45.get_frame()
        digit55_img = d55.get_frame()

        cv2.imshow("digit45", digit45_img)
        cv2.imshow("digit55", digit55_img)
        cv2.waitKey(1)

        # create the ros msg for the images and fill with all the info
        
        image_msg45 = Image()
        image_msg45.header.stamp = rospy.Time.from_sec(time.time())
        image_msg45.header.frame_id = "digit_camera"
        image_msg45.height = (digit45_img.shape)[0]
        image_msg45.width = (digit45_img.shape)[1]
        image_msg45.step = digit45_img.strides[0]
        image_msg45.data = digit45_img.flatten().tolist()
        image_msg45.encoding = "bgr8"
        
        
        image_msg55 = Image()
        image_msg55.header.stamp = rospy.Time.from_sec(time.time())
        image_msg55.header.frame_id = "digit_camera"
        image_msg55.height = (digit55_img.shape)[0]
        image_msg55.width = (digit55_img.shape)[1]
        image_msg55.step = digit55_img.strides[0]
        image_msg55.data = digit55_img.flatten().tolist()
        image_msg55.encoding = "bgr8"
        
        # transform to ros type and publish img
        pub45.publish(image_msg45)
        pub55.publish(image_msg55)
        # 100 hz rate of publication
        rate.sleep()



if __name__ == '__main__':
    main()
