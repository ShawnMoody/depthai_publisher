#!/usr/bin/env python3

import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Time

tfbr = None
pub_found = None

camera_name = "camera"
target_name = "Aruco"
LZ = 23

class ArucoDetector():
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters_create()

    frame_sub_topic = '/depthai_node/image/compressed'

    def send_tf_target(self):
        time_found = rospy.Time.now()
        # Create a transform arbitrarily in the camera frame
        self.t = TransformStamped()
        self.t.header.stamp = time_found
        self.t.header.frame_id = camera_name
        self.t.child_frame_id = target_name
        # In here we need a code to get the target location relative to the camera (Perhaps solve PnP)
        # Once we know where the target is, relative to the camera frame, we create and sent that transform (relative position target to camera)
        self.t.transform.translation.x = -0.4
        self.t.transform.translation.y = 0.2
        self.t.transform.translation.z = 1.5-0.15     # - altitude of the UAV camera Z.
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0
        # Send the transformation to TF
        # and "found" timestamp to localiser
        self.tfbr.sendTransform(self.t)
        self.pub_found.publish(time_found)

    def __init__(self):
        self.time_finished_processing = rospy.Time(0)

        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)

        self.br = CvBridge()

        if not rospy.is_shutdown():
            self.frame_sub = rospy.Subscriber(
                self.frame_sub_topic, CompressedImage, self.img_callback)

    def img_callback(self, msg_in):
        if msg_in.header.stamp > self.time_finished_processing:
            try:
                frame = self.br.compressed_imgmsg_to_cv2(msg_in)
            except CvBridgeError as e:
                rospy.logerr(e)

            aruco = self.find_aruco(frame)
            self.publish_to_ros(aruco)

            self.time_finished_processing = rospy.Time.now()

        # cv2.imshow('aruco', aruco)
        # cv2.waitKey(1)

    def find_aruco(self, frame):
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))
                #self.rospy.init_node('tf2_broadcaster_target')
                rospy.loginfo("sending possible landing zone found...")
                rospy.loginfo(marker_ID)

                if marker_ID == LZ : 
                    # Setup tf2 broadcaster and timestamp publisher
                    self.tfbr = tf2_ros.TransformBroadcaster()
                    self.pub_found = rospy.Publisher('/UAVTeam3/Aruco', Time, queue_size=10)
                 # Give the nodes a few seconds to configure
                    rospy.sleep(rospy.Duration(2))
                    # Send out our target messages
                    self.send_tf_target()
                    # Give the nodes a few seconds to transmit data
                    # then we can exit
                    rospy.sleep(rospy.Duration(2))
                    rospy.loginfo("Designated Landing Zone Found !")

                cv2.putText(frame, str(
                    marker_ID), (top_left[0], top_right[1] - 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)


        return frame

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        self.aruco_pub.publish(msg_out)


def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()
