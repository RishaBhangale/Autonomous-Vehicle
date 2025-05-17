#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from queue import Queue
import cv2
import numpy as np
import math 
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

class ImageConverter:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=10)
        self.image = None
        self.image_sub = rospy.Subscriber(topic, Image, self.callback, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.image = cv_image
        if not self.image_queue.full():
            self.image_queue.put(cv_image)

    def get_image(self):
        try:
            return self.image_queue.get(block=False)
        except:
            return self.image

class igvc():
    def shutdown(self):
        print("lane_daf STOP")

    def __init__(self, image_topic):
        rospy.init_node('lane_daf', anonymous=True)
        print("lane_daf START")
        rospy.on_shutdown(self.shutdown)

        self.image_converter = ImageConverter(image_topic)
        self.avg = 0

        self.intialTrackBarVals = [0, 255, 140, 255, 255, 255, 0, 0, 6]
        self.intialTrackBarValss = [0, 0, 255, 255, 255, 255, 0, 0, 6]

        cv2.namedWindow("Trackbars")
        cv2.resizeWindow("Trackbars", 360, 360)
        for i, val in enumerate(["H", "S", "V", "H2", "S2", "V2", "top", "bott", "height"]):
            cv2.createTrackbar(val, "Trackbars", self.intialTrackBarVals[i], 255 if i < 6 else 10, lambda x: None)

        self.fig = plt.figure()
        self.plot = None

    def lds(self):
        img = self.image_converter.get_image()
        if img is None:
            return

        w, h = img.shape[1], img.shape[0]
        roi_width = int(w)
        img = img[:int(h), :roi_width]

        trackbar_vals = [cv2.getTrackbarPos(val, "Trackbars") for val in ["H", "S", "V", "H2", "S2", "V2"]]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HLS) 
        maskedWhite = cv2.inRange(hsv,
                                  np.array(self.intialTrackBarVals[:3]),
                                  np.array(self.intialTrackBarVals[3:6]))
        maskedBarrels = cv2.inRange(hsv,
                                    np.array(self.intialTrackBarValss[:3]),
                                    np.array(self.intialTrackBarValss[3:6]))
        maskedWhite = cv2.bitwise_or(maskedWhite, maskedBarrels)
        imgBlur = cv2.GaussianBlur(maskedWhite, (5, 5), 0)

        plt.clf()

        p = 0.75
        polygons = np.array([[(0, int(h * p)), (roi_width, int(h * p)), (roi_width, h), (0, h)]])
        mask = np.zeros_like(imgBlur)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(imgBlur, mask)
        masked_image = 255 - masked_image

        hits = np.sum(masked_image, axis=0)
        self.plot, = plt.plot(hits)

        peaks, peak_plateaus = find_peaks(hits, plateau_size=10)
        imgg = img.copy()
        size = peak_plateaus['plateau_sizes']
        if size.size != 0:
            t = size.argmax()
            left, right = peak_plateaus['left_edges'][t], peak_plateaus['right_edges'][t]
            imgg = cv2.line(imgg, (left, int(h * p)), (right, int(h * p)), (255, 0, 0), 5)
            midptx1, midpty1 = (left + right) // 2, int(h * p)
            midptx2, midpty2 = roi_width // 2, h
            imgg = cv2.line(imgg, (midptx1, midpty1), (midptx2, midpty2), (255, 0, 0), 5)
            pp = (math.degrees(math.atan2((midpty2 - midpty1), midptx2 - midptx1)) - 90)
            self.avg = (pp + self.avg) / 2
            imgg = cv2.putText(imgg, str(self.avg), (roi_width // 2, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        else:
            if hits[0] > 200000:
                print('left error')
            if hits[-1] > 200000:
                print('right error')
            imgg = cv2.putText(imgg, str(self.avg), (roi_width // 2, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.imshow('LDds', imgg)
        cv2.waitKey(1)

if __name__ == '__main__':
    pub = rospy.Publisher('/float_data', Float32, queue_size=10)
    obj = igvc('/zed2i/zed_node/left_raw/image_raw_color')  # Replace with your actual topic name
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        obj.lds()
        pub.publish(obj.avg)
        rate.sleep()
