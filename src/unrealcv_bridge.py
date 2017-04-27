#!/usr/bin/env python
from __future__ import print_function
import os
import threading
import rospy
import cv_bridge
import cv2 as opencv
import unrealcv
import unreal_coordinates as ue_coords
import unrealcv_ros.srv as services


# Following tutorial here: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
class UnrealCVBridge(object):

    def __init__(self, config):
        host = unrealcv.HOST
        port = unrealcv.PORT

        if 'endpoint' in config:
            host, port = config['endpoint']
        if 'port' in config:
            port = config['port']
        if 'hostname' in config:
            host = config['hostname']

        self.opencv_bridge = cv_bridge.CvBridge()

        self._client_lock = threading.Lock()
        self._client = unrealcv.Client(endpoint=(host, port))
        self._client.connect()
        if not self._client.isconnected():
            raise RuntimeError("Could not connect to unrealcv simulator, is it running?")

        # Service attributes
        self._get_camera_view_service = None

    def create_services(self):
        print("Starting services...")
        self._get_camera_view_service = rospy.Service('get_camera_view', services.GetCameraView,
                                                      self.handle_get_camera_view)

    # Helpers and locking
    def get_camera_image(self, camera_id, location, rotation):
        roll, pitch, yaw = rotation
        self._client_lock.acquire()
        self._client.request("vset /camera/{0}/location {1} {2} {3}".format(camera_id, location[0],
                                                                            location[1], location[2]))
        self._client.request("vset /camera/{0}/rotation {1} {2} {3}".format(camera_id, pitch, yaw, roll))
        image_filename = self._client.request("vget /camera/{0}/lit".format(camera_id))
        self._client_lock.release()
        return image_filename

    # Service Handlers
    def handle_get_camera_view(self, request):
        # Parse the request arguments
        location = request.pose.position
        location = (location.x, location.y, location.z)
        rotation = request.pose.orientation
        rotation = (rotation.w, rotation.x, rotation.y, rotation.z)

        location, rotation = ue_coords.transform_to_unreal(location, rotation)
        camera_id = 0   # TODO: Handle multiple camera IDs
        image_filename = self.get_camera_image(camera_id, location, rotation)
        image_mat = opencv.imread(image_filename)
        os.remove(image_filename)
        return self.opencv_bridge.cv2_to_imgmsg(image_mat, encoding='passthrough')


def main():
    rospy.init_node('unrealcv_ros')
    unrealcv_bridge = UnrealCVBridge(config={})  # TODO: Get config from somewhere
    unrealcv_bridge.create_services()

    print("Ready!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down...')


if __name__ == '__main__':
    main()
