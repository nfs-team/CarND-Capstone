from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import sys
import rospy


"""
TLClassifier : is responsible for running SSD object detection and
returning the status of a traffic light signal.
"""
class TLClassifier(object):

    def __init__(self, model_file_path):

        #Initialize conv net
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_file_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        #Initialize tensorflow session
        with detection_graph.as_default():
            sess = tf.Session(graph=detection_graph)
        self._sess = sess

        #store tensors to run graph
        self._image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self._boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self._scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self._classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self._num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        #set threshold for detection
        self.THRESHOLD_SCORE = 0.6

    #format image numpy array
    def _load_image_as_nparray(self, img):
        return np.expand_dims(img, axis = 0)

    #score thresholding
    def _remove_boxes_criteria(self, boxes, scores, classes):
        #only keep boxes above a certain score
        keep_box = scores > self.THRESHOLD_SCORE
        boxes = boxes[keep_box]
        scores = scores[keep_box]
        classes = classes[keep_box]
        return boxes, scores, classes

    #detect traffic lights
    def get_classification(self, img):
        "Returns the traffic light status"

        image_batch = self._load_image_as_nparray(img)

        # Actual detection.
        (boxes, scores, classes, num_detections) = self._sess.run(
          [self._boxes, self._scores, self._classes, self._num_detections],
          feed_dict={self._image_tensor: image_batch})

        #remove singleton dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes =  np.squeeze(classes)

        [boxes, scores, classes] = self._remove_boxes_criteria(boxes, scores, classes)

        status = self._convert_light_status(classes.astype(np.int32))
        return status

    def _convert_light_status(self, classes):
        "Converts detection class to enum used in ROS Node"
        rospy.loginfo("num traffic light SSD detections %d", classes.size)
        if classes.size > 0:
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
