from __future__ import print_function
import cv2
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
#from Ros1_driving.detectColorsRosCam import empty
from sensor_msgs.msg import Image
from math import *
from geometry_msgs.msg import Twist
import math




class follow_lane:

    laneFollowCounter = 0

    def intersection(self,rho1,theta1,rho2,theta2):
        """Finds the intersection of two lines given in Hesse normal form.

        Returns closest integer pixel locations.
        See https://stackoverflow.com/a/383527/5087436
        """
        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([[rho1], [rho2]])
        x0, y0 = np.linalg.solve(A, b)
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        return [[x0, y0]]


    def followLane(self,imgFrame) :

        frameHeight = imgFrame.shape[0]
        frameWidth = imgFrame.shape[1]


        dst = cv2.Canny(imgFrame, 50, 200, None, 3)
        
        # Copy edges to the images that will display the results in BGR
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        
        lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 20, 100)
        
        leftLinePrinted = False
        rightLinePrinted = False

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if (theta >0.01 and theta < 10.0) :
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    #Right lane
                    if (theta > 2.3 and theta < 2.6 and rightLinePrinted == False ) :
                        print("line: " + str(i)+ ", theta: " + str(theta))
                        cv2.line(cdstP, pt1, pt2, (192,192,192), 3, cv2.LINE_AA)
                        rightLinePrinted = True
                        rho1 = rho
                        theta1 = theta 

                    #left lane
                    if (theta > 0.2 and theta < 0.8  and leftLinePrinted == False) :
                        print("line: " + str(i)+ ", theta: " + str(theta))
                        cv2.line(cdstP, pt1, pt2, (255,255,0), 3, cv2.LINE_AA)
                        leftLinePrinted = True
                        rho2 = rho
                        theta2 = theta 

                    # if (i == 8) :
                    #     print("line: " + str(i)+ ", theta: " + str(theta))
                    #     cv2.line(cdstP, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)


        

        if (leftLinePrinted == True and rightLinePrinted == True) :
            InterceptionPoint = self.intersection(rho1,theta1,rho2,theta2)   


            InterceptionPointX = InterceptionPoint[0][0]

            print (InterceptionPointX)


            if (InterceptionPointX < 300) :
                move.linear.x = 0.1
                #move.linear.x = 0.0
                move.angular.z = 0.1
                pub.publish(move)
                print ("L")
                self.laneFollowCounter += 1
            if (InterceptionPointX > 340) :
                move.linear.x = 0.1
                #move.linear.x = 0.0
                move.angular.z = -0.1
                pub.publish(move)
                self.laneFollowCounter += 1
                print ("R")
                self.noLanesFound = 0
            if (InterceptionPointX > 300 and InterceptionPointX < 340) :
                move.linear.x = 0.1
                #move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                print ("F")
                self.laneFollowCounter += 1
                self.noLanesFound = 0
        elif (leftLinePrinted == True and rightLinePrinted == False) :
                move.linear.x = 0.0
                move.angular.z = 0.1
                pub.publish(move)
        
        if (self.laneFollowCounter > 20) :
            self.laneFollowCounter = 0
            exit("ending lane assist")
        #cv2.imshow("Source", imgFrame)
        #cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        return imgFrame


rospy.init_node('LaneFollower', anonymous=True)

move = Twist() 
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

followLane = follow_lane()

cap = cv2.VideoCapture("http://192.168.1.212:8080/?action=stream")

frameCounter = 0
while(True):
    frameCounter +=1
    ret, frame = cap.read()
    if (frameCounter == 10 ):
        followLane.followLane(frame)
        frameCounter=0
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()