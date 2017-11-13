import socket
import os
from tl_classifier import *

import sys

ssd_inception_sim_model = 'D:\\CarND-Capstone\\ros\\src\\tl_detector\\light_classification\\frozen_models\\frozen_sim_inception\\frozen_inference_graph.pb'
light_classifier = TLClassifierServer(ssd_inception_sim_model)

try:
    import cPickle as pickle
except ImportError:
    import pickle

s = socket.socket()
s.bind((b'',5005))
s.listen(1)
while True:
    c,a = s.accept()
    data = b''

    print("SERVER STARTED RECEIVING DATA ")
    #l = c.recv(1024)
    total = 1440138 #int(l)
    print("EXPECTED SIZE {} ".format(total))
    while True:
        block = c.recv(4096)
        if not block: break
        data += block
        total -= len(block)
        if total == 0: break

    print("SERVER RECEIVED DATA ")

    if sys.version_info.major < 3:
        unserialized_input = pickle.loads(data)
    else:
        unserialized_input = pickle.loads(data,encoding='bytes')

    light = light_classifier.get_classification(unserialized_input)

    print("SERVER DETECT ", light)

    c.send(bytes([light]))
    c.close()

