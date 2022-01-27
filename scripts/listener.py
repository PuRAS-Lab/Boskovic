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
    cv_image = cv2.GaussianBlur(cv_image,ksize, 0)
    
    #Nakon blur-ovanja slike primjenjujemo Canny-jev algoritam
    edges = cv2.Canny(cv_image, tresh1, tresh2)
    
    #Kreiranje ROI
    vertices = np.array([[args[2],args[3]],[args[4],args[5]],[args[6],args[7]],[args[8],args[9]],[args[10],args[11]],[args[12],args[13]]], np.int32)
    roi_img = roi(edges, [vertices])
    
    rho = 2  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 100  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 40  # minimum number of pixels making up a line
    max_line_gap = 10  # maximum gap in pixels between connectable line segments
    line_image = np.zeros([cv_image.shape[0], cv_image.shape[1], 1], dtype=np.uint8)   # creating a blank to draw lines on
    line_image.fill(255)
    
    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(roi_img, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_image,(x1,y1),(x2,y2),(0,0,255),3)
    
    lines_edges = cv2.addWeighted(cv_image, 0.8, line_image, 1, 0)

    # NOTE: dve linije ispod, ne zaboraviti promeniti ime varijable (roi_img, ...)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(lines_edges, "8UC1"))
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv2.bitwise_not(lines_edges))
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
    # fetch a group (dictionary) of parameters - ROI points
    if rospy.has_param('roi_points'):
        roi_points = rospy.get_param('roi_points')
        x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6 = roi_points['x1'], roi_points['y1'], roi_points['x2'], roi_points['y2'], \
                                                         roi_points['x3'], roi_points['y3'], roi_points['x4'], roi_points['y4'], \
                                                         roi_points['x5'], roi_points['y5'], roi_points['x6'], roi_points['y6']
    else:
        x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6 = 0, 600, 0, 520, 325, 325, 500, 325, 700, 520, 700, 600
    #rospy.loginfo("coordinates are %s, %s, %s, %s, %s, %s,%s, %s, %s, %s, %s, %s", x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6)
    
    # u RVIZ pogledajte koji je Source u Image prozoru
    rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, callback, (tresh1, tresh2, x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    image_pub = rospy.Publisher(publish_topic, Image, queue_size=10)
    listener()

