#!/usr/bin/env python

from glob import glob
import os
import rospy
import re
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray, Rect
import time
import cv2

class BenchmarkTest():

    def __init__(self):
        self.img_dir = rospy.get_param('~img_dir', None)
        self.gt_file = rospy.get_param('~gt_file', None)
        self.iou_thresh = rospy.get_param('~iou_thresh', 0.5)
        self.bridge = CvBridge()

        self.img_names = sorted(glob(os.path.join(self.img_dir, '*')))
        # print(self.img_names)

        with open(self.gt_file, 'r') as f:
            self.gt_rects = [list(map(float, re.split('[,\t ]', x.strip()))) for x in f.readlines()]

        # print(self.gt_rects)
        self.image_pub = rospy.Publisher(
            '~test_img', Image, queue_size=1)
        self.init_rect_pub = rospy.Publisher(
            '~init_rect', RectArray, queue_size=1)

        self.result_rect_sub = rospy.Subscriber('~output_rect', RectArray, self.rectCb)
        self.pred_rects = []
        self.pred_rects.append(self.gt_rects[0])

        time.sleep(1)

        stamp = rospy.Time.now()
        img = cv2.imread(self.img_names[0])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)

        rect_msg = RectArray()
        rect_msg.header.stamp = stamp
        rect = self.gt_rects[0]
        rect_msg.rects.append(Rect(rect[0], rect[1], rect[2], rect[3]))
        self.init_rect_pub.publish(rect_msg)

        time.sleep(1)
        img = cv2.imread(self.img_names[1])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)



    def rectCb(self, msg):
        #print(msg.rects[0]) # TODO: do evaluation with ground truth
        rect = msg.rects[0]
        self.pred_rects.append([rect.x, rect.y, rect.width, rect.height])

        if len(self.pred_rects) == len(self.img_names):
            if len(self.pred_rects) == len(self.gt_rects):
                mean_iou = self.compute_mean_iou()
                print("mean iou of {} is {}".format(self.img_names[0].rsplit("/",1)[0], mean_iou))
                if mean_iou < self.iou_thresh:
                    raise ValeError("unexpected tracking result")
            rospy.signal_shutdown("stop")

        stamp = rospy.Time.now()
        img = cv2.imread(self.img_names[len(self.pred_rects)])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)


    def compute_mean_iou(self):
        mean_iou = []
        for pred, gt in zip(self.pred_rects, self.gt_rects):
            area_a = pred[2] * pred[3]
            area_b = gt[2] * gt[3]
            iou_x1 = max(pred[0], gt[0])
            iou_y1 = max(pred[1], gt[1])
            iou_x2 = min(pred[0] + pred[2], gt[0] + gt[2])
            iou_y2 = min(pred[1] + pred[3], gt[1] + gt[3])
            iou_w = iou_x2 - iou_x1
            iou_h = iou_y2 - iou_y1
            if iou_w < 0 or iou_h < 0:
                mean_iou.append(0.0)
            else:
                area_iou = iou_w * iou_h
                iou = area_iou / (area_a + area_b - area_iou)
                mean_iou.append(iou)
        return sum(mean_iou)/len(mean_iou)


def main():
    rospy.init_node('benchmark_test_node')
    test = BenchmarkTest()
    rospy.spin()


if __name__ == '__main__':
    main()
