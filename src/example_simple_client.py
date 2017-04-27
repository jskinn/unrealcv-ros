#!/usr/bin/env python
from __future__ import print_function
import cv2
import cv_bridge
import rospy
import geometry_msgs.msg as geom_msgs
import unrealcv_ros.srv as services


def create_pose(location, rotation):
    if len(location) >= 3:
        x, y, z = location[0:3]
    else:
        x = y = z = 0
    if len(rotation) >= 4:
        qw, qx, qy, qz = rotation[0:4]
    else:
        qw = 1
        qx = qy = qz = 0

    return geom_msgs.Pose(
        position=geom_msgs.Point(x=x, y=y, z=z),
        orientation=geom_msgs.Quaternion(w=qw, x=qx, y=qy, z=qz)
    )


def main():
    location = (100, -270, 100)
    rotation = (1, 0, 0, 0)

    get_camera_view_service = rospy.ServiceProxy('get_camera_view', services.GetImageForPose)

    pose = create_pose(location, rotation)
    response = get_camera_view_service(pose)

    opencv_bridge = cv_bridge.CvBridge()
    image = opencv_bridge.imgmsg_to_cv2(response.image)
    cv2.imshow('test', image)
    cv2.waitKey()


if __name__ == "__main__":
    main()
