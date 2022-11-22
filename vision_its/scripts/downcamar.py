#!/usr/bin/env python3
import rospy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2.aruco as aruco
from vision_its.msg import Artag
from vision_its.msg import Artags
import math

bridge = CvBridge()
artag_pub = None
allowed_error = 0.0

def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

def ArucoDetect(img, aruco_bbox):
    (h, w) = img.shape[:2]
    if np.all(aruco_bbox[1] is not None):  # If there are markers found by detector
        artags = Artags()
        for i in range(0, len(aruco_bbox[1])):  # Iterate in markers
            # CAMERA_MATRIX=np.array([[287.15056916836687, 0.0, 423.78333060345307], [0.0, 287.8995718273279, 398.78145503436554], [0.0, 0.0, 1.0]])
            # DIST=np.array([[0.007889529092625133, -0.006923112471089945, 0.008924586121118452, 0.00390728088062372, 0.0015919785219895355]])
            CAMERA_MATRIX=np.array([[39160.75085796965, 0.0, 319.7063806125065], [0.0, 33359.3167021978, 180.29977661202506], [0.0, 0.0, 1.0]])

            DIST=np.array([[-38.947506388946984, -0.0031341493541042938, 0.12117021956373404, -0.02014223538019587, -7.087827942888905e-08]])


            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(aruco_bbox[0][i], 0.02, CAMERA_MATRIX, DIST)
            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            aruco.drawDetectedMarkers(img, aruco_bbox[0])  # Draw A square around the markers
            cv2.drawFrameAxes(img, CAMERA_MATRIX, DIST, rvec, tvec, 0.01)  # Draw Axis

            rotM = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec[0], rotM, jacobian = 0)

            # Old Centering Method
            # x_sum = aruco_bbox[0][i][0][0][0]+ aruco_bbox[0][i][0][1][0]+ aruco_bbox[0][i][0][2][0]+ aruco_bbox[0][i][0][3][0]
            # y_sum = aruco_bbox[0][i][0][0][1]+ aruco_bbox[0][i][0][1][1]+ aruco_bbox[0][i][0][2][1]+ aruco_bbox[0][i][0][3][1]
            # x_centerPixel = x_sum*.25
            # y_centerPixel = y_sum*.25
            # centered = isNear(w, h, x_centerPixel, y_centerPixel, 10)

            ypr = cv2.RQDecomp3x3(rotM)
            yaw = ypr[0][0]
            pitch = ypr[0][1]
            roll = ypr[0][2]
            x = tvec[0][0][0]
            y = tvec[0][0][1]
            z = tvec[0][0][2]

            centered = isNear(x, y, 0, 0, 0.001)
            perpendicular = isPerpendicular(yaw, pitch, allowed_error)

            artag = Artag()
            print(artag)

            artag.id = int(aruco_bbox[1][i])
            artag.yaw = yaw
            artag.pitch = pitch
            artag.roll = roll
            # artag.x = x
            # artag.y = y
            # artag.z = z
            # artag.isCentered = centered
            # artag.isPerpendicular = perpendicular
            artags.artags.append(artag)


        artag_pub.publish(artags)

def isNear(w, h, cX, cY, r):
    d = math.sqrt((cX - w/2) ** 2 + (cY - h/2) ** 2)
    return d <= r

def isPerpendicular(yaw, pitch, error):
    return (pitch <= error and pitch >= -error) and (yaw >= 180 - error or yaw <= -180 + error)

# For Debugging
def ArucoDebug(img):
    (h, w) = img.shape[:2]
    cv2.circle(img, (int(w/2), int(h/2)), 10, (0, 255, 0), 2)


def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    arucofound = findArucoMarkers(frame)
    
    ArucoDetect(frame, arucofound)
    ArucoDebug(frame)

    cv2.imshow("Artag", frame)

    key = cv2.waitKey(10)

    if key==27: # 27 = Esc
        rospy.signal_shutdown("Done testing!")

def receive():
    # rospy.Subscriber("/camera/fisheye1/rect/image", Image, callback)
    rospy.Subscriber("/drone/down_camera/image_raw", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("test" , anonymous=True , disable_signals=True)
    artag_pub = rospy.Publisher('/artag', Artags, queue_size=10)
    try:
        receive()
    except rospy.ROSInterruptException: pass
