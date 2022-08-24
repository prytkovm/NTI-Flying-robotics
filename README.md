# Code parts from NTI olympiad (rospy + clever)

## Colors recognition

```python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import time
from clever import srv
from std_srvs.srv import Trigger


rospy.init_node('flight') # Register in ROS as node "flight"
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()
color_yellow = (0, 255, 255)

# Limits for colors in HSV color space
hsv_green_min = np.array((43, 100, 100), np.uint8)
hsv_green_max = np.array((77, 255, 255), np.uint8)
hsv_yellow_min = np.array((15, 100, 100), np.uint8)
hsv_yellow_max = np.array((35, 255, 255), np.uint8)
color_low1 = (0, 100, 100)
color_high1 = (15, 255, 255)
color_low2 = (165, 100, 100)
color_high2 = (180, 255, 255)
colors_pub = rospy.Publisher('~colors_detection', Image, queue_size=1)


def image_callback(data):
    ''' Image-processing callback function '''
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_thresh1 = cv2.inRange(img_hsv, color_low1, color_high1)
    img_thresh2 = cv2.inRange(img_hsv, color_low2, color_high2)
    img_thresh = cv2.bitwise_or(img_thresh1, img_thresh2)
    # Operations to reduce noises on image
    kernel = np.ones((5, 5), dtype=np.uint8)
    img_thresh_eroded = cv2.erode(img_thresh, kernel)
    img_thresh_dilated = cv2.dilate(img_thresh_eroded, kernel)
    moments = cv2.moments(img_thresh_dilated)
    # m00 - sum of pixel values
    # m01 - static moment relative to the y axis
    # m10 - static moment relative to the x axis
    if moments["m00"] != 0.0:
        cnt_x = int(moments["m10"] / moments["m00"])
        cnt_y = int(moments["m01"] / moments["m00"])
        img = cv2.circle(img, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
        cv2.putText(img, "High temperature", (cnt_x + 10, cnt_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print('High temperature at x={} y={} | Express test dropped'.format(cnt_x, cnt_y))
        colors_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    thresh2 = cv2.inRange(img_hsv, hsv_yellow_min, hsv_yellow_max)
    moments = cv2.moments(thresh2, 1)
    if moments["m00"] != 0.0:
        cnt_x = int(moments["m10"] / moments["m00"])
        cnt_y = int(moments["m01"] / moments["m00"])
        img = cv2.circle(img, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
        cv2.putText(img, "Need to check", (cnt_x + 10, cnt_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print('Need to check at x={} y={} | Express test dropped'.format(cnt_x, cnt_y))
        colors_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    thresh3 = cv2.inRange(img_hsv, hsv_green_min, hsv_green_max)
    moments = cv2.moments(thresh3, 1)
    if moments["m00"] != 0.0:
        cnt_x = int(moments["m10"] / moments["m00"])
        cnt_y = int(moments["m01"] / moments["m00"])
        img = cv2.circle(img, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
        cv2.putText(img, "Healthy", (cnt_x + 10, cnt_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print('Healthy at x={} y={}'.format(cnt_x, cnt_y))
        colors_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)


navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
time.sleep(11)
navigate(x=0.295, y=0.295, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(4)
navigate(x=0.885, y=0.295, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(4)
navigate(x=0.295, y=0.885, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=0.885, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.295, y=1.475, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=1.475, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.295, y=2.065, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=2.065, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.59, y=2.655, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
land()
```

## QR-codes recognition 

```python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import time
from clever import srv
from std_srvs.srv import Trigger
#так же регистрируемся в ROS и сообщаем, что мы нода
rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()
color_yellow = (0, 255, 255)
#Указываем куда будем публиковать изображения, в нашем случае /flight/imageQR
image_pub = rospy.Publisher('~imageQR', Image, queue_size=1)


def image_callback(data):
    ''' Image-processing callback function'''
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.encode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        cv_image = cv2.circle(cv_image, (xc, yc), 10, (0, 255, 0), 3)
        cv2.putText(cv_image, b_data, (xc + 10, yc - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1) # Subscribe to camera topic

navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
time.sleep(11)
navigate(x=0.295, y=0.295, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(4)
navigate(x=0.885, y=0.295, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(4)
navigate(x=0.295, y=0.885, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=0.885, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.295, y=1.475, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=1.475, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.295, y=2.065, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.885, y=2.065, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0.59, y=2.655, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='aruco_map')
time.sleep(5)
land()

```
