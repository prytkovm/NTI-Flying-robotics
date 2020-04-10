# NTI_Power_Codecomments
Итоговый отчет
```python
#аналогично импортируем все, что нам понадобится
# -*- coding: utf-8 -*-
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

#так же объявляем функцию для обработки изображения
def image_callback(data):
	#так же получаем изображение с камеры
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    #декодируем
    barcodes = pyzbar.decode(cv_image)
   # в цикле будем производить необходимые нам операции с распознанными QR кодами
    for barcode in barcodes:
    	#переводим ранее декодированную информацию в utf-8
        b_data = barcode.data.encode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        #так же нарисуем окружность в центре распознанного QR кода
        cv_image = cv2.circle(cv_image, (xc, yc), 10, (0, 255, 0), 3)
       # и поместим надпись с декодированной информацией
        cv2.putText(cv_image, b_data, (xc + 10, yc - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))
        # теперь опубликуем изображение с проведенными над ним манипуляциями в необходимый нам топик
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

# ну и подписка на топик камеры так же необходима
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

# летим по координатам
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
