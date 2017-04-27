#!/usr/bin/env python
from __future__ import print_function
import os
import threading
import numpy as np
import rospy
import std_msgs.msg
import geometry_msgs.msg
import cv_bridge
import cv2 as opencv
import unrealcv
import unreal_coordinates as ue_coords
import unrealcv_ros.srv as services


# Helper messages to create UnrealCV message URIs
def make_vget_camera_image(camera_id, view_mode, filename=None):
    if filename is None:
        return "vget /camera/{0}/{1}".format(camera_id, view_mode)
    else:
        return "vget /camera/{0}/{1} {2}".format(camera_id, view_mode, filename)


def make_vget_camera_location(camera_id):
    return "vget /camera/{0}/location".format(camera_id)


def make_vget_camera_rotation(camera_id):
    return "vget /camera/{0}/rotation".format(camera_id)


def make_vget_viewmode():
    return "vget /viewmode"


def make_vset_move_camera(camera_id, x, y, z):
    location, _ = ue_coords.transform_to_unreal((x, y, z), None)
    return "vset /camera/{0}/moveto {1} {2} {3}".format(camera_id, location[0], location[1], location[2])


def make_vset_camera_location(camera_id, x, y, z):
    location, _ = ue_coords.transform_to_unreal((x, y, z), None)
    return "vset /camera/{0}/location {1} {2} {3}".format(camera_id, location[0], location[1], location[2])


def make_vset_camera_rotation(camera_id, w, x, y, z):
    _, rotation = ue_coords.transform_to_unreal(None, (w, x, y, z))
    roll, pitch, yaw = rotation
    return "vset /camera/{0}/rotation {1} {2} {3}".format(camera_id, pitch, yaw, roll)


def make_vset_viewmode(viewmode):
    return "vset /viewmode {0}".format(viewmode)


def make_vget_object_color(object_name):
    return "vget /object/{0}/color".format(object_name)


def make_vset_object_color(object_name, r, g, b):
    return "vset /object/{0}/color {1} {2} {3}".format(object_name, r, g, b)


def make_vget_object_location(object_name):
    return "vget /object/{0}/location".format(object_name)


def make_vget_object_rotation(object_name):
    return "vget /object/{0}/rotation".format(object_name)


def make_vset_object_location(object_name, x, y, z):
    location, _ = ue_coords.transform_to_unreal((x, y, z), None)
    return "vset /object/{0}/location {1} {2} {3}".format(object_name, location[0], location[1], location[2])


def make_vset_object_rotation(object_name, w, x, y, z):
    _, rotation = ue_coords.transform_to_unreal(None, (w, x, y, z))
    roll, pitch, yaw = rotation
    return "vset /object/{0}/rotation {1} {2} {3}".format(object_name, pitch, yaw, roll)


class UnrealCVPassthrough(object):
    """
    A ROS node for the unrealcv API.
    The goal of this node is to exactly mirror the UnrealCV API, documented here:
    http://unrealcv.org/reference/commands.html
    This is based on the source code, at time of writing, the documentation above is incomplete.
    """

    # These are the valid view modes for the cameras.
    view_modes = ['lit', 'depth', 'normal', 'object_mask', 'wireframe']

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

        # Store the declare services
        self._services = []

    def create_services(self):
        print("Starting services...")
        # Camera control services
        self._services.append(rospy.Service('get_camera_view', services.GetCameraImage, self.handle_get_camera_image))
        self._services.append(rospy.Service('get_camera_view_with_filename', services.GetCameraImageWithFilename,
                                            self.handle_get_camera_image))
        self._services.append(rospy.Service('get_camera_location', services.GetCameraLocation,
                                            self.handle_get_camera_location))
        self._services.append(rospy.Service('get_camera_rotation', services.GetCameraRotation,
                                            self.handle_get_camera_rotation))
        self._services.append(rospy.Service('get_viewmode', services.GetViewmode, self.handle_get_viewmode))
        self._services.append(rospy.Service('move_camera', services.MoveCamera, self.handle_move_camera))
        self._services.append(rospy.Service('set_camera_location', services.SetCameraLocation,
                                            self.handle_set_camera_location))
        self._services.append(rospy.Service('set_camera_rotation', services.SetCameraRotation,
                                            self.handle_set_camera_rotation))
        self._services.append(rospy.Service('set_viewmode', services.SetViewmode, self.handle_set_viewmode))

        # object control services
        self._services.append(rospy.Service('get_object_color', services.GetObjectColor, self.handle_get_object_color))
        self._services.append(rospy.Service('set_object_color', services.SetObjectColor, self.handle_set_object_color))
        self._services.append(rospy.Service('get_object_location', services.GetObjectLocation,
                                            self.handle_get_object_location))
        self._services.append(rospy.Service('get_object_rotation', services.GetObjectRotation,
                                            self.handle_get_object_rotation))
        self._services.append(rospy.Service('set_object_location', services.SetObjectLocation,
                                            self.handle_set_object_location))
        self._services.append(rospy.Service('set_object_rotation', services.SetObjectRotation,
                                            self.handle_set_object_rotation))

    def shutdown_services(self, reason=''):
        for service in self._services:
            service.shutdown(reason)
        self._client.disconnect()

    # Helpers and locking
    def request_client(self, request):
        self._client_lock.acquire()
        result = self._client.request(request)
        self._client_lock.release()
        return result

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
    def handle_get_camera_image(self, request):
        # Parse the request arguments
        filename = None
        if hasattr(request, 'filename'):
            filename = request.filename
        view_mode = 'lit'
        if hasattr(request, 'view_mode') and request.view_mode in self.view_modes:
            view_mode = request.view_mode

        unrealcv_message = make_vget_camera_image(request.camera_id, view_mode, filename)
        image_filename = self.request_client(unrealcv_message)
        if os.path.isfile(image_filename):
            image_mat = opencv.imread(image_filename)
            os.remove(image_filename)
        else:
            image_mat = np.matrix([[]])
        return self.opencv_bridge.cv2_to_imgmsg(image_mat, encoding='passthrough')

    def handle_get_camera_location(self, request):
        message = make_vget_camera_location(request.camera_id)
        location = self.request_client(message)
        location, _ = ue_coords.transform_from_unreal(location, None)
        return geometry_msgs.msg.Point(x=location[0], y=location[1], z=location[2])

    def handle_get_camera_rotation(self, request):
        message = make_vget_camera_location(request.camera_id)
        rotation = self.request_client(message)
        _, rotation = ue_coords.transform_from_unreal(None, rotation)
        return geometry_msgs.msg.Quaternion(w=rotation[0], x=rotation[1], y=rotation[2], z=rotation[3])

    def handle_get_viewmode(self, request):
        return self.request_client(make_vget_viewmode())

    def handle_move_camera(self, request):
        message = make_vset_move_camera(request.camera_id, request.location.x, request.location.y, request.location.z)
        return self.request_client(message)

    def handle_set_camera_location(self, request):
        message = make_vset_camera_location(request.camera_id, request.location.x,
                                            request.location.y, request.location.z)
        return self.request_client(message)

    def handle_set_camera_rotation(self, request):
        message = make_vset_camera_rotation(request.camera_id, request.rotation.w, request.rotation.x,
                                            request.rotation.y, request.rotation.z)
        return self.request_client(message)

    def handle_set_viewmode(self, request):
        view_mode = 'lit'
        if hasattr(request, 'view_mode') and request.view_mode in self.view_modes:
            view_mode = request.view_mode
        return self.request_client(make_vset_viewmode(view_mode))

    def handle_get_object_color(self, request):
        color = self.request_client(make_vget_object_color(request.object_name))
        r, g, b, a = map(int, color.split())
        return std_msgs.msg.ColorRGBA(r=r, g=g, b=b, a=a)

    def handle_set_object_color(self, request):
        message = make_vset_object_color(request.object_name, request.color.r, request.color.g, request.color.b)
        return self.request_client(message)

    def handle_get_object_location(self, request):
        message = make_vget_object_location(request.object_name)
        location = self.request_client(message)
        location, _ = ue_coords.transform_from_unreal(location, None)
        return geometry_msgs.msg.Point(x=location[0], y=location[1], z=location[2])

    def handle_get_object_rotation(self, request):
        message = make_vget_object_rotation(request.object_name)
        rotation = self.request_client(message)
        _, rotation = ue_coords.transform_from_unreal(None, rotation)
        return geometry_msgs.msg.Quaternion(w=rotation[0], x=rotation[1], y=rotation[2], z=rotation[3])

    def handle_set_object_location(self, request):
        message = make_vset_object_location(request.object_name, request.location.x,
                                            request.location.y, request.location.z)
        return self.request_client(message)

    def handle_set_object_rotation(self, request):
        message = make_vset_object_rotation(request.object_name, request.rotation.w, request.rotation.x,
                                            request.rotation.y, request.rotation.z)
        return self.request_client(message)


def main():
    rospy.init_node('unrealcv_ros')
    unrealcv_bridge = UnrealCVPassthrough(config={})  # TODO: Get config from somewhere
    unrealcv_bridge.create_services()

    print("Ready!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down...')
    unrealcv_bridge.shutdown_services("Finished")


if __name__ == '__main__':
    main()
