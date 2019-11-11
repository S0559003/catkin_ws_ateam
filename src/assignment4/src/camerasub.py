#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('assignment4')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self):

    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw",Image,self.callback)


  def callback(self,data):

    #convert the ros image to a cv image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # the intrinsic_parameters from the CameraInfo topic
    intrinsic_parameters = np.float32([[383.7944641113281, 0.0, 322.3056945800781],[0.0, 383.7944641113281, 241.67051696777344],[0.0, 0.0, 1.0]])

    # the distortion coefficients from the CameraInfo topic
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    #convert the image to a grayscale image
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # hide bigger white places on the field by occulude them with black rectangles
    rows, cols = cv_image.shape
    cv_image = cv2.rectangle(cv_image, (0,0), (cols-1,106), (0,0,0), -1)
    cv_image = cv2.rectangle(cv_image, (0,rows-130), (cols-1,rows-1), (0,0,0), -1)
    cv_image = cv2.rectangle(cv_image, ((cols/2)-120,rows-230), ((cols/2) + 150,rows-1), (0,0,0), -1)

    #convert the image to binary
    binary_img =  cv2.threshold(cv_image, 240, 255, cv2.THRESH_BINARY)[1]

    # find contours in the binary image
    im2, contours, hierarchy = cv2.findContours(binary_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #index for the points to increment in the loop
    counter = 0
    #dict to store the 6 points
    points_on_ground = np.float32([[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]])
    points_on_car =  np.array([[0.5,0.2,0],[0.5,-0.2,0],[0.8,0.2,0],[0.8,-0.2,0],[1.1,0.2,0],[1.1,-0.2,0]])

    for c in contours:
       # calculate moments for each contour
       M = cv2.moments(c)

       # calculate x,y coordinate of center
       cX = int(M["m10"] / M["m00"])
       cY = int(M["m01"] / M["m00"])
       cv2.circle(binary_img, (cX, cY), 1, (0, 0, 0), -1)
       points_on_ground[counter] = [cX, cY]
       counter += 1

    #compute extrinsic parameters with solvePnP
    #ret, rvec, tvec = cv2.solvePnP(object_points, marker_points, camera_matrix, dist_coeff)
    ret, rvec, tvec = cv2.solvePnP(points_on_car, points_on_ground, intrinsic_parameters, dist_coeffs)

    rotation_matrix = np.arange(3,3)

    rotation_matrix, wasanderes = cv2.Rodrigues(rvec)

    transformation_matrix = [[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],tvec[0][0]],
                   [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],tvec[1][0]],
                   [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2],tvec[2][0]],
                   [0.0, 0.0, 0.0, 1.0]]

    #print("transformation matrix: ",transformation_matrix)
    #print("\n\n\n\n\n\n")

    inverse_transformation = np.linalg.inv(transformation_matrix)


    print("\n******************************** Output solvePnP **********************************\n")
    print("rvec: \n",rvec)
    #print("inverse transformation: ",inverse_transformation)
    print("\n\n")

    print("tvec: \n",tvec)
    #print("inverse transformation: ",inverse_transformation)
    print("\n\n")

    print("\n******************************* Transofmation Matrix an Inverse ***********************************\n")
    print("Transformation Matrix: \n",transformation_matrix)
    #print("inverse transformation: ",inverse_transformation)
    print("\n\n")

    print("Inverse Transformation Matrix: \n",inverse_transformation)
    #print("inverse transformation: ",inverse_transformation)
    print("\n\n")

    print("\n************************ Check Inverse Matrix with a Point P ***************************************\n")
    print("Point P for check inverse: ",[points_on_ground[0][0],points_on_ground[0][1],0,1])
    pointtransform = np.dot(transformation_matrix,[points_on_ground[0][0],points_on_ground[0][1],1,1])
    print("\n\n")

    print("Dot Product (P * Transformation Matrix) = D :",pointtransform)
    print("\n\n")
    inversetransformp = np.dot(inverse_transformation, pointtransform)

    print("Dot Product (D * Inverse Transformation Matrix) = P ",inversetransformp)

    '''
    transformation_matrix:

    [0.016633132510232462, -0.99773441110706473,    -0.065187297809735922,  0.03167831,
    -0.52853957600128143,   0.046568970831419976,   -0.84763037201134284,   0.25024891,
     0.84874569542548017,   0.048552815042389619,   -0.52656753474571261,   0.09192414,
     0.0,                    0.0,                    0.0,                   1.0        ]
    '''

    cv2.imshow("Image window", binary_img)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
