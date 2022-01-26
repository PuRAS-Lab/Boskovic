#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# globals
image_pub = None

# globals for parametrisation
publish_topic = "output_image"

# pogledajte ovaj primer:
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

def roi(image, vertices):
    #blank mask:
    mask = np.zeros_like(image)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(image, mask)
    return masked

def callback(data, args):
    tresh1 = args[0]
    tresh2 = args[1]
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    #cv_image postaje grayscale slika koja koristi cv_image kao src za transformaciju
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY )
    #Blurujemo sliku sa 3x3
    ksize = (3, 3)
    cv_image = cv2.blur(cv_image,ksize )
    
    #Nakon blur-ovanja slike primjenjujemo Canny-jev algoritam
    edges = cv2.Canny(cv_image, tresh1, tresh2)
    
    #Kreiranje ROI
    vertices = np.array([[0,600],[0,500],[300,300],[450,300],[700,500],[700,600]], np.int32)
    roi_img = roi(edges, [vertices])
    
    # NOTE: dve linije ispod, ne zaboraviti promeniti ime varijable (roi_img, ...)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(roi_img, "8UC1"))
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", roi_img)
    # mora da se stavi neki broj, inače ne osvežava image
    cv2.waitKey(1)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    tresh1 = rospy.get_param("canny_tresh_1", "100")
    tresh2 = rospy.get_param("canny_tresh_2", "200")
    
    # u RVIZ pogledajte koji je Source u Image prozoru
    rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, callback, (tresh1, tresh2))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    image_pub = rospy.Publisher(publish_topic, Image, queue_size=10)
    listener()

