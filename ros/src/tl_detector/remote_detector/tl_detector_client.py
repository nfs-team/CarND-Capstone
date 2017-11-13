from styx_msgs.msg import TrafficLight
import numpy as np
import sys
import rospy
import socket, os

try:
    import cPickle as pickle
except ImportError:
    import pickle

import threading
tl_detection_lock = threading.Lock()

"""
TLClassifier : is responsible for running SSD object detection and
returning the status of a traffic light signal.
"""
class TLClassifierClient(object):

    def __init__(self):

        self.size = 1024

    #detect traffic lights
    def get_classification(self, img):
        "Returns the traffic light status"
        with tl_detection_lock:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect(("", 5005))
            serialized_data = pickle.dumps(img, protocol=2)
            #l = len(serialized_data)
            #self.client_socket.sendall(str(l))

            self.client_socket.sendall(serialized_data)
            # self.client_socket.send(str(len(img)))
            # self.client_socket.send(img)
            light = self.client_socket.recv(1024)
            self.client_socket.close()
            status = self._convert_light_status([ord(light[0])])
            return status

    def _convert_light_status(self, classes):
        "Converts detection class to enum used in ROS Node"
        rospy.loginfo("num traffic light SSD detections %d", len(classes))
        if len(classes) > 0:
            #Take the classes which occurs max
            lightclass = classes[0]
            if lightclass == 1:
                status = TrafficLight.GREEN
            elif lightclass == 2:
                status = TrafficLight.RED
            elif lightclass == 3:
                status = TrafficLight.YELLOW
            else:
                status = TrafficLight.UNKNOWN
        else:
            status = TrafficLight.UNKNOWN

        return status
