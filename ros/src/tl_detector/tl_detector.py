#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
from remote_detector.tl_detector_client import TLClassifierClient

# Path to frozen detection graph. This is the actual model that is used for the object detection.
#ssd_inception_sim_model = 'light_classification/frozen_models/frozen_sim_inception/frozen_inference_graph.pb'
#ssd_inception_real_model = 'light_classification/frozen_models/frozen_real_inception/frozen_inference_graph.pb'

#TODO: Change this model when running in read world
#PATH_TO_CKPT = ssd_inception_sim_model

LOOKAHEAD_WPS = 60
STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        ssd_inception_model = rospy.get_param('~ssd_inception_model')
        self.detection_mode = rospy.get_param('~mode') #normal, remote, test
        self.light_classifier = None
        if self.detection_mode == "normal":
            self.light_classifier = TLClassifier(ssd_inception_model)
        elif self.detection_mode == "remote":
            self.light_classifier = TLClassifierClient()

        #Classify once to activate XLA JIT compiler. This step takes time.
        #After this step the node detects lights faster and starts to publishes traffic light info
        if self.detection_mode != "test":
            dummyImg = 255 * np.ones((1096, 1368, 3)).astype(np.uint8)
            self.light_classifier.get_classification(dummyImg)

        rospy.loginfo("Setup TL Classifier")

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.wait_redlight = 0
        self.wait_redlight_limit = 50

        self.stop_line_wps = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.stop_line_wps = None

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement optimized version (OPTIONAL)
        #TODO: we already know this in waypoint_updater, so why not publish waypoint index and subscribe here
        if self.waypoints is None:
            return

        min_distance = None
        waypoint_index = None

        for i in range(len(self.waypoints)):
            distance = pow(self.waypoints[i].pose.pose.position.x - pose.position.x, 2) + pow(
                self.waypoints[i].pose.pose.position.y - pose.position.y, 2)
            if min_distance is None or distance < min_distance:
                min_distance = distance
                waypoint_index = i

        return waypoint_index

    def get_closest_traffic_light(self, stop_line_positions, current_car_wp):
        """Determines wp index of the closest traffic light. Probably we can assume that waypoints and traffic lights
        are not changing over time so we could make connections between waypoints and traffic lights once and then reuse it
        """

        # Init connection between stop lines and waypoints
        if self.stop_line_wps is None:
            self.init_stop_line_waypoint_positions(stop_line_positions)

        closest_tf_wp = None
        closest_tf = None

        # Look for closest traffic light ahead of the car
        for stop_line_position, tf_wp in self.stop_line_wps:
            # Check traffic lights only ahead of the car
            if tf_wp >= current_car_wp:
                if closest_tf_wp is None or tf_wp < closest_tf_wp:
                    closest_tf_wp = tf_wp
                    closest_tf = stop_line_position

        if closest_tf_wp is None or current_car_wp is None or closest_tf_wp-current_car_wp > LOOKAHEAD_WPS:
            return -1, None

        return closest_tf_wp, closest_tf

    def init_stop_line_waypoint_positions(self, stop_line_positions):
        self.stop_line_wps = []
        for stop_line_position in stop_line_positions:
            tf_pose = Pose()
            tf_pose.position.x = stop_line_position[0]
            tf_pose.position.y = stop_line_position[1]
            # Find waypoint index corresponding to traffic light
            tf_wp = self.get_closest_waypoint(tf_pose)
            self.stop_line_wps.append((stop_line_position, tf_wp))

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        #Get classification
        cv_img = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        np_img = np.array(cv_img)
        #beginT = rospy.get_time()
        status = self.light_classifier.get_classification(np_img)
        #endT = rospy.get_time()
        #rospy.loginfo("SSD Execution Time (ms): %f", (endT - beginT )* 1e3)
        return status

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = TrafficLight.UNKNOWN
        light_wp = -1
        debug = True
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position_wp = self.get_closest_waypoint(self.pose.pose)
            light_wp, _ = self.get_closest_traffic_light(stop_line_positions, car_position_wp)
            if self.detection_mode == "test":
                if light_wp is not None and car_position_wp is not None:
                    if abs(light_wp - car_position_wp) >= 5:
                        light = TrafficLight.RED
                        self.wait_redlight = 0
                    else:
                        if self.wait_redlight > self.wait_redlight_limit:
                            light = TrafficLight.GREEN
                        else:
                            light = TrafficLight.RED
                        self.wait_redlight += 1
            else:
                if light_wp != -1:
                    light = self.get_light_state()
                    if debug:
                        if light == TrafficLight.GREEN:
                            lightstr = 'green'
                        elif light == TrafficLight.RED:
                            lightstr = 'red'
                        elif light == TrafficLight.YELLOW:
                            lightstr = 'yellow'
                        elif light == TrafficLight.UNKNOWN:
                            lightstr = 'unknown'
                        rospy.loginfo("car light status: %s", lightstr)

        return light_wp, light

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
