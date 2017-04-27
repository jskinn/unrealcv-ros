#!/usr/bin/env python
from __future__ import print_function
import cv2
import cv_bridge
import rospy
import geometry_msgs.msg as geom_msgs
import unrealcv_ros.srv as services


def main():
    set_location_service = rospy.ServiceProxy('set_camera_location', services.SetCameraLocation)
    set_rotation_service = rospy.ServiceProxy('set_camera_rotation', services.SetCameraRotation)
    get_image_service = rospy.ServiceProxy('get_camera_view', services.GetCameraImage)

    set_location_service(0, geom_msgs.Point(x=100, y=-270, z=100))
    set_rotation_service(0, geom_msgs.Quaternion(w=0.70710678, x=0, y=0, z=-0.70710678))
    response = get_image_service(0, 'depth')

    opencv_bridge = cv_bridge.CvBridge()
    image = opencv_bridge.imgmsg_to_cv2(response.image)
    cv2.imshow('test', image)
    cv2.waitKey()


if __name__ == "__main__":
    main()
