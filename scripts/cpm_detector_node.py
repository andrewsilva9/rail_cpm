#!/usr/bin/env python
# This file is responsible for bridging ROS to the ObjectDetector class (built with PyCaffe)

from __future__ import division

import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rail_cpm.msg import Keypoints, Poses

import pose_estimation

# Debug Helpers
FAIL_COLOR = '\033[91m'
ENDC_COLOR = '\033[0m'


def eprint(error):
    sys.stderr.write(
        FAIL_COLOR
        + type(error).__name__
        + ": "
        + error.message
        + ENDC_COLOR
    )
# End Debug Helpers


class CPM(object):
    """
    This class takes in image data and finds / annotates objects within the image
    """

    def __init__(self):
        rospy.init_node('cpm_detector_node')
        self.person_keypoints = []
        self.keypoint_arrays = []
        self.image_datastream = None
        self.input_image = None
        self.bridge = CvBridge()
        self.detector = pose_estimation.PoseMachine()
        self.debug = rospy.get_param('~debug', default=False)
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/qhd/image_color_rect')

    def _parse_image(self, image_msg):
        """
        Take in an image and draw a bounding box within it
        Publishes bounding box data onwards
        :param image_msg: Image data
        :return: None
        """

        header = image_msg.header

        try:
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print e
            return
        # self.person_keypoints = self.detector.estimate_keypoints(image_cv)
        print 'ABOUT TO ESTIMATE KEYPOINTS'
        candidate, subset = self.detector.estimate_keypoints(image_cv)
        #### DEBUG ####
        if self.debug:
            out_image = self.detector.visualize_keypoints(image_cv, candidate, subset)
            try:
                image_msg = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            except CvBridgeError as e:
                print e

            image_msg.header = header
            self.image_pub.publish(image_msg)
        #### END DEBUG ####

        # Instantiate detections object
        obj_arr = Poses()
        obj_arr.header = header
        # obj_arr.objects = self.objects
        # For each object / keypoint set found in the image:

        # for bbox_obj in self.objects:
        #     if len(bbox_obj) < 1:
        #         continue
        #     bbox_obj = bbox_obj[0]
        #     rospy.loginfo("BBox Obj" + str(bbox_obj))
        #     msg = Object()
        #     msg.object_id = bbox_obj[4]
        #     msg.top_left_x = int(bbox_obj[0])
        #     msg.top_left_y = int(bbox_obj[1])
        #     msg.bot_right_x = int(bbox_obj[2])
        #     msg.bot_right_y = int(bbox_obj[3])
        #     obj_arr.objects.append(msg)
        # for bbox_obj in self.objects2:
        #     if len(bbox_obj) < 1:
        #         continue
        #     bbox_obj = bbox_obj[0]
        #     rospy.loginfo("BBox Obj" + str(bbox_obj))
        #     msg = Object()
        #     msg.object_id = bbox_obj[4]
        #     msg.top_left_x = int(bbox_obj[0])
        #     msg.top_left_y = int(bbox_obj[1])
        #     msg.bot_right_x = int(bbox_obj[2])
        #     msg.bot_right_y = int(bbox_obj[3])
        #     obj_arr.objects.append(msg)

        self.object_pub.publish(obj_arr)

    def run(self,
            pub_image_topic='/rail_cpm/debug/keypoint_image',
            pub_object_topic='/rail_cpm/keypoints'):
        rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
        if self.debug:
            self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2) # image publisher
        self.object_pub = rospy.Publisher(pub_object_topic, Poses, queue_size=2) # objects publisher
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = CPM()
        detector.run()
    except rospy.ROSInterruptException:
        pass
