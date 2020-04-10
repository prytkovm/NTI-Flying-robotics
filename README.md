С распознаванием цветов как в Python, так и в целом мы столкнулись впервые, поэтому было решено сначала поизучать теорию и поиграться
с распознаванием при помощи вебки.
Тесты с вебкамерой помогли подбрать цветовые диапозоны и отладить сам алгоритм распознавания цвета, после чего были предприняты попытки написания кода под Клевер:
```python
# -*- coding: utf-8 -*-
#импортируем все, что нам необходимо
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import time
from clever import srv
from std_srvs.srv import Trigger
#регистрируемся в ROS и говорим, что мы нода с указанным именем( в нашем случае  "flight")
rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
#инициализируем библиотеку cv_bridge и создаем объект с нужным нам названием (в нашем случае bridge).
#Через него будем производить конвертацию сообщений ROS в изображения openCv и наоборот
bridge = CvBridge()
#цвет шрифта
color_yellow = (0, 255, 255)
# нижняя и верхняя границы зеленого цвета
hsv_green_min = np.array((43, 100, 100), np.uint8)
hsv_green_max = np.array((77, 255, 255), np.uint8)
# нижняя и верхняя границы желтого цвета
hsv_yellow_min = np.array((15, 100, 100), np.uint8)
hsv_yellow_max = np.array((35, 255, 255), np.uint8)
#с красным цветом немного иначе, т.к в цветовом пространстве hsv ось H закольцована,
#а значит красными будут как величины близкие к 0 , так и близкие к 180,
#поэтому распознавать будем в два прохода и результат комбинировать результат
color_low1 = (0, 100, 100)
color_high1 = (15, 255, 255)
color_low2 = (165, 100, 100)
color_high2 = (180, 255, 255)
#объект colors_pub, отвечающий за публикацию изображения в /flight/colors_detection
colors_pub = rospy.Publisher('~colors_detection', Image, queue_size=1)

#Функция, которая будет обрабатывать изображение
def image_callback(data):
	#получаем изображение с камеры
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    #перевод полученного изображения в цветовое пространство hsv
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  # выделение части изображения, которая входит в цветовой диапозон красного
    img_thresh1 = cv2.inRange(img_hsv, color_low1, color_high1)
    img_thresh2 = cv2.inRange(img_hsv, color_low2, color_high2)
#объединение двух изображений сверху. Если точка белая хотя бы на одном из изображений,
#то она будет белой и на конечном
    img_thresh = cv2.bitwise_or(img_thresh1, img_thresh2)
#избавление от шумов
#создаем ядро - квадратный структурный элемент, который в нашем случае имеет размер 5х5 пикселей
    kernel = np.ones((5, 5), dtype=np.uint8)
#при помощи операции эрозии (erode) мы уменьшаем количество шумов, но вместе с этим уменьшаются и найденные области
    img_thresh_eroded = cv2.erode(img_thresh, kernel)
#чтобы восстановить найденные области пользуемся операцией dilate (бинаризованное изображение и структурный элемент в качестве параметров)
    img_thresh_dilated = cv2.dilate(img_thresh_eroded, kernel)
#вычисляем центр белой области на финальном изображении
    moments = cv2.moments(img_thresh_dilated)
# m00 - сумма значений пикселей
# m01 - статический момент относительно оси y
# m10 - статический момент относительно оси x
# проверям попало ли что-то в наш диапозон, чтобы не напороться на деление на нуль
    if moments["m00"] != 0.0:
        cnt_x = int(moments["m10"] / moments["m00"])
        cnt_y = int(moments["m01"] / moments["m00"])
# рисуем в найденном центре окружность с радиусом 10, толщиной линии 3 и зеленым цветом этой линии
        img = cv2.circle(img, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
 # помещаем рядом с окружностью текст, сообщающий нам о том, какая у пациента температура 
        cv2.putText(img, "High temperature", (cnt_x + 10, cnt_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
 # Вывод в терминал информации о температуре и о том, сброшен ли экспресс-тест
        print('High temperature at x={} y={} | Express test dropped'.format(cnt_x, cnt_y))
 #публикуем обработанное изображение в нужный нам топик
        colors_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
# выделение части изображения, которая входит в цветовой диапозон желтого
    thresh2 = cv2.inRange(img_hsv, hsv_yellow_min, hsv_yellow_max)
   #операции, аналогичные тем, что были проделаны с красным, только на этот раз с желтым
    moments = cv2.moments(thresh2, 1)
    if moments["m00"] != 0.0:
        cnt_x = int(moments["m10"] / moments["m00"])
        cnt_y = int(moments["m01"] / moments["m00"])
        img = cv2.circle(img, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
        cv2.putText(img, "Need to check", (cnt_x + 10, cnt_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        print('Need to check at x={} y={} | Express test dropped'.format(cnt_x, cnt_y))
        colors_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
 #операции, аналогичные тем, что были проделаны с красным, на этот раз с зеленым
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

#подписываемся на топик камеры
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)


# Полет по заданным координатам
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
Это было достаточно тяжело, особенно по началу, но позже пришли и понимание, и радость от того, что все работает корректно.
С QR-кодами было намного проще и их распознавание начало корректно работать уже во втором тестовом запуске(распознавание цветов 
начало нормально работать только в день финальных запусков):

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

```
